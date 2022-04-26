# limit the number of cpus used by high performance libraries
from cmath import inf
from msilib.schema import Class
from multiprocessing.connection import wait
import os
from urllib import robotparser

from cv2 import circle
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["VECLIB_MAXIMUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"

import sys
sys.path.insert(0, './yolov5')

import argparse
import os
import platform
import shutil
import time
from pathlib import Path
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from mdvision import mdvision

from yolov5.models.experimental import attempt_load
from yolov5.utils.downloads import attempt_download
from yolov5.models.common import DetectMultiBackend
from yolov5.utils.datasets import LoadImages, LoadStreams
from yolov5.utils.general import (LOGGER, check_img_size, non_max_suppression, scale_coords, 
                                  check_imshow, xyxy2xywh, increment_path)
from yolov5.utils.torch_utils import select_device, time_sync
from yolov5.utils.plots import Annotator, colors
from deep_sort.utils.parser import get_config
from deep_sort.deep_sort import DeepSort
from yolov5.utils.augmentations import letterbox
import matplotlib.pyplot as plt 
import socket
import threading

#-------------------Running Settings------------------------#
## TCP settings
USE_TCPSERVER = 1
START_SELF_TCP_SERVER = 0
CONNECT_WITH_CAR1 = 0
CONNECT_WITH_CAR2 = 0
CONNECT_WITH_DECISION_TREE = 1

VIDEO_NAME = "7.mp4"

USE_CENTER_POINT = 1

confidence = 0.5

#与决策树节点的socket
goal_ip1 = "192.168.43.126" 
port1 = 1953

#与TCP server的socket
goal_ip2 = "127.0.0.1"
port2 = 2022

#与机器人的socket
goal_ip3 = "192.168.43.126"
port3 = 2002
#-----------------------------------------------------------#

need_to_send_flag = 0
need_to_send_msg = []

#------------Our robot information-------------------------
robot_x = 0
robot_y = 0
#--------------------------------------------------------

img_base = cv2.imread("map2.png")
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # yolov5 deepsort root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

class Server(object):
    def __init__(self, goal_ip, port):
        # 在线客户端
        self.online_pool = {}
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, True)
        self.server.bind((goal_ip, port))

    # 消息广播方法
    def broadcast(self, msg):
        for i in self.online_pool:
            print("send msg to {}".format(i))
            print(msg)

            #决策树发过来的就不要再发回去了，没有意义
            #if (msg[7] == 'G' and self.online_pool[i][1] == ('192.168.43.126')):
            #    continue

            #机器人发过来的就不要再发回去了，没有意义
            #if (msg[7] == 's' and self.online_pool[i][1] == ('192.168.43.48')):
            #    continue

            self.online_pool[i].send(msg.encode('utf-8'))

    # 客户端登录方法
    def login(self, client_socket, info):
        print('{} Login'.format(info))
        #if len(self.online_pool) >= 1:
        #    self.broadcast('{} get online...'.format(info))
        self.online_pool[info] = client_socket
        # 通知新用户当前在线列表
        #msg = 'Now online users:\n'
        #for i in self.online_pool:
        #   msg += (str(i) + '\n')
        #print(msg)
        #msg = msg.encode('utf-8')
        #client_socket.send(msg)

    # 客户端登出方法
    def logout(self, info):
        del self.online_pool[info]
        msg = '{} is disconnect'.format(info)
        self.broadcast(msg)

    # 发送消息方法
    def send_msg(self, info, msg):
        msg = msg.encode('utf-8')
        if info in self.online_pool:
            self.online_pool[info].send(msg)

    # 会话管理方法
    def session(self, client_socket, info):
        # 新用户登录,执行login()
        self.login(client_socket, info)
        while True:
            if (len(need_to_send_msg) > 0):
                self.broadcast(need_to_send_msg[0])
                need_to_send_msg.pop(0)

            try:
                data = client_socket.recv(65536).decode('utf-8')
                # 如果收到长度为0的数据包,说明客户端 调用了secket.close()
                if len(data) == 0:
                    print('{}robot is disconnect...'.format(info))
                    # 此时需要调通客户端 登出 方法
                    self.logout(info)
                    break
                else:
                    print(info, ':', data)
                    my_buffer = str(data)
                    if (my_buffer.find('s')!= -1):
                        true_end = my_buffer.find('s')
                        true_data = my_buffer[0: true_end]
                        rest_flag = true_data.find(' ')
                        global robot_y, robot_x, img_base  #类成员函数内引用全局变量
                        robot_x = int(true_data[0: rest_flag])
                        robot_x = robot_x
                        robot_y = int(true_data[rest_flag+1: len(true_data)])
                        robot_y = 408 - robot_y
                        cv2.circle(img_base, [robot_x, robot_y], 3, (0, 100, 0), -1)
                    # 此时服务端接收到正常的消息,广播给其他所有客户端
                    #content = str(info) + ': '
                    #data = content + data
                    for i in self.online_pool:
                        if i != info:
                            print("Alose send msg to {}".format(info))
                            self.send_msg(i, data)

            except Exception as err:
                print(err)
                # 如果抛出异常,说明客户端强制退出,并没有调用socket.close()
                # 此时需要调通客户端 登出 方法
                self.logout(info)
                break

    def listener_thread_function(self):
        while True:
            client_socket, info = self.server.accept()
            print("accept succed")
            thread = threading.Thread(target=self.session, args=(client_socket, info))
            thread.setDaemon(True)
            thread.start()

    def start(self, goal_ip, port, have_end_flag):
        print("Ready to listen port{}".format((goal_ip, port)))
        
        self.server.listen(port)
        
        listener_thread = threading.Thread(target=self.listener_thread_function)    #冷知识：thread创建进程的时候，如果target后面的函数带括号，那么代表阻塞式线程
        listener_thread.setDaemon(True)
        listener_thread.start()
        
        # thread0 = threading.Thread(target=self.server.accept(), args=(client_socket, info))
        # thread0.setDaemon(True)
        # thread0.start()

        #client_socket, info = self.server.accept()
        

        #self.login(client_socket, info)
        # thread = threading.Thread(target=self.session, args=(client_socket, info))
        # thread.setDaemon(True)
        # thread.start()


#广播方法
def wait_to_publish():
    global need_to_send_flag
    global need_to_send_msg
    while True:
        c = ''
        c = input()
        need_to_send_msg.append(c)  #写入待发送数组
        # if (ttt != 1):
        #     c = c + 'e'
        # self.broadcast(c)
        #print("send message to {} success".format(info))


if (USE_TCPSERVER == 1):
    thread2 = threading.Thread(target = wait_to_publish)
    thread2.setDaemon(True)
    thread2.start()

    if (CONNECT_WITH_DECISION_TREE == 1):
        server = Server(goal_ip1, port1)
        server.start(goal_ip1, port1, 1)
        print('Server {} is Online!'.format((goal_ip1, port1)))

    if (START_SELF_TCP_SERVER == 1):
        server2 = Server(goal_ip2, port2)
        server2.start(goal_ip2, port2)
        print("The server {} is online!".format((goal_ip2, port2)))

    if (CONNECT_WITH_CAR1 == 1):
        server3 = Server(goal_ip3, port3)
        server3.start(goal_ip3, port3, 2)
        print("The server {} is online!".format((goal_ip3, port3)))

print("Create TCP server success!!")

mapPerserPoints = []
def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        mapPerserPoints.append([x,y])


def detect(opt):
    out, yolo_model, deep_sort_model, show_vid, save_vid, save_txt, imgsz, evaluate, half, project, name, exist_ok= \
        opt.output, opt.yolo_model, opt.deep_sort_model, opt.show_vid, opt.save_vid, \
        opt.save_txt, opt.imgsz, opt.evaluate, opt.half, opt.project, opt.name, opt.exist_ok
    # webcam = source == '0' or source.startswith(
    #     'rtsp') or source.startswith('http') or source.endswith('.txt')

    device = select_device(opt.device)
    # initialize deepsort
    cfg = get_config()
    cfg.merge_from_file(opt.config_deepsort)
    deepsort = DeepSort(deep_sort_model,
                        device,
                        max_dist=cfg.DEEPSORT.MAX_DIST,
                        max_iou_distance=cfg.DEEPSORT.MAX_IOU_DISTANCE,
                        max_age=cfg.DEEPSORT.MAX_AGE, n_init=cfg.DEEPSORT.N_INIT, nn_budget=cfg.DEEPSORT.NN_BUDGET,
                        )
    if (CONNECT_WITH_DECISION_TREE == 1):
        global server
    global img_base
    # Initialize
    half &= device.type != 'cpu'  # half precision only supported on CUDA

    # The MOT16 evaluation runs multiple inference streams in parallel, each one writing to
    # its own .txt file. Hence, in that case, the output folder is not restored
    if not evaluate:
        if os.path.exists(out):
            pass
            shutil.rmtree(out)  # delete output folder
        os.makedirs(out)  # make new output folder

    # Directories
    exp_name = yolo_model.split(".")[0] if type(yolo_model) is str else "ensemble"
    exp_name = exp_name + "_" + deep_sort_model
    save_dir = increment_path(Path(project) / exp_name, exist_ok=exist_ok)  # increment run if project name exists
    save_dir.mkdir(parents=True, exist_ok=True)  # make dir

    # Load model
    device = select_device(device)
    model = DetectMultiBackend(yolo_model, device=device, dnn=opt.dnn)
    stride, names, pt, jit, _ = model.stride, model.names, model.pt, model.jit, model.onnx
    imgsz = check_img_size(imgsz, s=stride)  # check image size

    # Half
    half &= pt and device.type != 'cpu'  # half precision only supported by PyTorch on CUDA
    if pt:
        model.model.half() if half else model.model.float()

    # Set Dataloader
    vid_path, vid_writer = None, None
    # Check if environment supports image displays
    if show_vid:
        show_vid = check_imshow()

    # # Dataloader
    # if webcam:
    #     show_vid = check_imshow()
    #     cudnn.benchmark = True  # set True to speed up constant image size inference
    #     dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt and not jit)
    #     bs = len(dataset)  # batch_size
    # else:
    #     dataset = LoadImages(source, img_size=imgsz, stride=stride, auto=pt and not jit)
    #     bs = 1  # batch_size
    # vid_path, vid_writer = [None] * bs, [None] * bs

    # Get names and colors
    names = model.module.names if hasattr(model, 'module') else model.names

    # extract what is in between the last '/' and last '.'
    # txt_file_name = source.split('/')[-1].split('.')[0]
    # txt_path = str(Path(save_dir)) + '/' + txt_file_name + '.txt'

    if pt and device.type != 'cpu':
        model(torch.zeros(1, 3, *imgsz).to(device).type_as(next(model.model.parameters())))  # warmup
    dt, seen = [0.0, 0.0, 0.0, 0.0], 0

    #my add ######################################
    #my_video = cv2.VideoCapture("video/3.mp4")
    my_video = mdvision(200,640,480)#1280  720
    if(not my_video.camera_init()):
        return

    cv2.namedWindow('MapPerser')
    cv2.setMouseCallback('MapPerser', mouse_callback)

    dstPoints = np.float32([[0,0],[228,0],[228,408],[0,408]])

    frame = my_video.camera_read()
    if frame is None:
        return
    # for frame_idx, (path, img, im0s, vid_cap, s) in enumerate(dataset):
    #     # cv2.imshow("img",img)
    #     # cv2.waitKey(10000)
    #     #img = torch.from_numpy(img).to(device)
    #     # frame[:,:,:1] = img[:1,:,:]
    #     # frame[:,:,1:2] = img[1:2,:,:]
    #     # frame[:,:,1:2] = img[1:2,:,:]
    #     img = img.reshape((640,384,3))
    #     cv2.imshow("img",img)
    #     cv2.waitKey(10000)
    #     print(img.shape)
    #     print(img)
    #     frame=cv2.resize(img,dsize=(640,480))
    #     #break
    while len(mapPerserPoints) <= 4:
        if len(mapPerserPoints) > 0:
            cv2.circle(frame, mapPerserPoints[len(mapPerserPoints)-1], 5, 255, -1)
        cv2.imshow('MapPerser', frame)
        cv2.waitKey(1)

    srcPoints = np.float32(mapPerserPoints[0:4])
    matrix = cv2.getPerspectiveTransform(srcPoints, dstPoints)#计算透视映射矩阵

    #my add ######################################


    while (cv2.waitKey(1) & 0xFF) != ord('q'):
        s = ""
        #my add ############################################
        frame = my_video.camera_read()
        if frame is None:
            continue
        
        im0 = np.copy(frame) 
        # Padded resize
        img = letterbox(im0, my_video.width, 16, True)[0]

        # Convert
        img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        img = np.ascontiguousarray(img)

        img_base = cv2.imread("map2.png")
        map = cv2.warpPerspective(frame, matrix, (640,480))
        #my add ############################################


        t1 = time_sync()
        img = torch.from_numpy(img).to(device)

        #print(img.shape)

        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        t2 = time_sync()
        dt[0] += t2 - t1


        # Inference
        visualize = increment_path(save_dir / Path("").stem, mkdir=True) if opt.visualize else False
        pred = model(img, augment=opt.augment, visualize=visualize)
        t3 = time_sync()
        dt[1] += t3 - t2

        # Apply NMS
        pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, opt.classes, opt.agnostic_nms, max_det=opt.max_det)
        dt[2] += time_sync() - t3

        # Process detections
        for i, det in enumerate(pred):  # detections per image
            seen += 1
            # if webcam:  # batch_size >= 1
            #     p, im0, _ = path[i], im0s[i].copy(), dataset.count
            #     s += f'{i}: '
            # else:
            #     p, im0, _ = path, im0s.copy(), getattr(dataset, 'frame', 0)

            # p = Path(p)  # to Path
            # save_path = str(save_dir / p.name)  # im.jpg, vid.mp4, ...
            s += '%gx%g ' % img.shape[2:]  # print string

            annotator = Annotator(frame, line_width=2, pil=not ascii)

            if det is not None and len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(
                    img.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{names[int(c)]}{'s' * (n > 1)}, "  # add to string

                xywhs = xyxy2xywh(det[:, 0:4])
                confs = det[:, 4]
                clss = det[:, 5]

                # pass detections to deepsort
                t4 = time_sync()
                outputs = deepsort.update(xywhs.cpu(), confs.cpu(), clss.cpu(), im0)
                t5 = time_sync()
                dt[3] += t5 - t4

                # draw boxes for visualization
                if len(outputs) > 0:
                    all_car = []
                    for j, (output, conf) in enumerate(zip(outputs, confs)):

                        if conf < confidence:
                            continue

                        bboxes = output[0:4]
                        id = output[4]
                        cls = output[5]

                        c = int(cls)  # integer class
                        label = f'{names[c]} {conf:.2f}'
                        annotator.box_label(bboxes, label, color=colors(c, True))
                        
                        #my add #########################################
                        for k in range(len(det)):
                            pt2 = [int(det[k][0].item()),int(det[k][3].item())] #左下角
                            pt3 = [int(det[k][0].item()),int(det[k][1].item())] #左上角
                            pt4 = [int(det[k][2].item()),int(det[k][3].item())] #右下角

                            # pt = ((pt3 + pt4) / 2 + pt2) /2
                            #xyxy
                            # pt2 = [0.5*pt2[0], 0.5*pt2[1]]
                            # pt3 = [0.25*pt3[0],0.25*pt3[0]]
                            # pt4 = [0.25*pt4[0],0.25*pt4[0]]

                            # pt = pt2 + pt3 + pt4
                            # pt = [int(pt[0]),int(pt[1])]

                            pt = pt2
                            lu_point = pt3
                            rd_point = pt4

                            lu_x = lu_point[0]
                            lu_y = lu_point[1]
                            rd_x = rd_point[0]
                            rd_y = rd_point[1]

                            cv2.circle(frame, [(lu_x + rd_x) // 2, (lu_y + rd_y) // 2], 5, 255, -1)

                            translated_lupoint = (matrix @ np.array([[lu_x],[lu_y],[1]]))
                            translated_rdpoint = (matrix @ np.array([[rd_x],[rd_y],[1]]))


                            translated_lupoint_x = int(translated_lupoint[0][0]/translated_lupoint[2][0])
                            translated_lupoint_y = int(translated_lupoint[1][0]/translated_lupoint[2][0])

                            translated_rdpoint_x = int(translated_rdpoint[0][0]/translated_rdpoint[2][0])
                            translated_rdpoint_y = int(translated_rdpoint[1][0]/translated_rdpoint[2][0])

                            translated_center_point_x = (translated_lupoint_x + translated_rdpoint_x) // 2
                            translated_center_point_y = (translated_lupoint_y + translated_rdpoint_y) // 2

                            cv2.circle(frame, pt, 5, 255, -1)

                            x,y = pt[0], pt[1]
                            #x,y = (pt[0] + translated_center_point_x) / 2, (pt[1] + translated_center_point_y) / 2

                            PerserPoint = (matrix @ np.array([[x],[y],[1]]))

                            x,y = int(PerserPoint[0][0]/PerserPoint[2][0]), int(PerserPoint[1][0]/PerserPoint[2][0])
                            x1,y1 = (int(PerserPoint[0][0]/PerserPoint[2][0]) + translated_center_point_x) // 2, \
                                    (int(PerserPoint[1][0]/PerserPoint[2][0]) + translated_center_point_y) // 2

                            cv2.circle(map, [x,y], 3, 255, -1)
                            cv2.circle(map, [x1,y1], 3, 255, -1)
                            

                            
                            
                            x = 228 - x
                            if (x < 0):
                                x = 0

                            x1 = 228 - x1
                            if (x1 < 0):
                                 x1 = 0

                            cv2.circle(img_base, [x,y], 5, (0,0,255), -1)
                            cv2.circle(img_base, [x1,y1], 5, (0,0,255), -1)

                            all_car.append([x1, y1])
                            # if (USE_CENTER_POINT == 1):
                            #     x = x1
                            #     y = y1

                            # my_str1 = ''
                            # x = 228 - x
                            # if (x < 100):
                            #     my_str1 = '0' + str(x)
                            # else:
                            #     my_str1 = str(x)

                            # my_str2 = ''
                            
                            # if (y < 100):
                            #     my_str2 = '0' + str(y)
                            # else:
                            #     my_str2 = str(y)
                            
                            # need_to_send_msg = my_str2 + ' ' + my_str1 + 'p'
                            
                            # if (CONNECT_WITH_DECISION_TREE == 1):
                            #     server.broadcast(need_to_send_msg)

                        #print("Detected {} cars".format(np.array(all_car).shape))
                        cv2.circle(img_base, [robot_x, robot_y], 3, (0, 100, 0), -1)

                        temp_min = 9999999
                        temp_i = 0
                        for i in range(0, len(all_car)):
                            #计算该框距离
                            my_dist = (robot_x - all_car[i][0]) * (robot_x - all_car[i][0]) + (robot_y - all_car[i][1]) * (robot_y - all_car[i][1])
                            if (my_dist < temp_min):
                                temp_min = my_dist
                                temp_i = i

                        temp_max = -9999999
                        temp_i_max = 0
                        for i in range(0, len(all_car)):
                            #计算该框距离
                            my_dist = (robot_x - all_car[i][0]) * (robot_x - all_car[i][0]) + (robot_y - all_car[i][1]) * (robot_y - all_car[i][1])
                            if (my_dist > temp_max):
                                temp_max = my_dist
                                temp_i_max = i
                
                        if (len(all_car) > 0):
                            print("ready to send")
                            cv2.circle(img_base, [all_car[temp_i][0], all_car[temp_i][1]], 9, (0), -1)
                            cv2.circle(img_base, [all_car[temp_i_max][0], all_car[temp_i_max][1]], 9, (0, 0, 255), -1)

                            if (USE_CENTER_POINT == 1):
                                x = all_car[temp_i_max][0]
                                y = all_car[temp_i_max][1]

                            my_str1 = ''
                            x = 228 - x
                            if (x < 100):
                                my_str1 = '0' + str(x)
                            else:
                                my_str1 = str(x)

                            my_str2 = ''
                            
                            if (y < 100):
                                my_str2 = '0' + str(y)
                            else:
                                my_str2 = str(y)
                            
                            need_to_send_msg = my_str2 + ' ' + my_str1 + 'E'
                            
                            if (CONNECT_WITH_DECISION_TREE == 1):
                                #print(need_to_send_msg)
                                if (len(need_to_send_msg) == 8):
                                    server.broadcast(need_to_send_msg)
                        else:
                            if (CONNECT_WITH_DECISION_TREE == 1):
                                need_to_send_msg = "        "
                                print("send NULL")
                                server.broadcast(need_to_send_msg)

                        cv2.circle(img_base, [robot_x, robot_y], 9, (0, 255, 0), -1)
                            
                        #my add  ##########################################################

                        # if save_txt:
                        #     # to MOT format
                        #     bbox_left = output[0]
                        #     bbox_top = output[1]
                        #     bbox_w = output[2] - output[0]
                        #     bbox_h = output[3] - output[1]
                        #     # Write MOT compliant results to file
                        #     with open(txt_path, 'a') as f:
                        #         f.write(('%g ' * 10 + '\n') % (frame_idx + 1, id, bbox_left,  # MOT format
                        #                                        bbox_top, bbox_w, bbox_h, -1, -1, -1, -1))

                #LOGGER.info(f'{s}Done. YOLO:({t3 - t2:.3f}s), DeepSort:({t5 - t4:.3f}s)')

            else:
                deepsort.increment_ages()
                #LOGGER.info('No detections')

            # Stream results
            im0 = annotator.result()
            if show_vid:
                cv2.imshow("result", im0)
                if cv2.waitKey(1) == ord('q'):  # q to quit
                    raise StopIteration

            # # Save results (image with detections)
            # if save_vid:
            #     if vid_path != save_path:  # new video
            #         vid_path = save_path
            #         if isinstance(vid_writer, cv2.VideoWriter):
            #             vid_writer.release()  # release previous video writer
            #         if vid_cap:  # video
            #             fps = vid_cap.get(cv2.CAP_PROP_FPS)
            #             w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            #             h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            #         else:  # stream
            #             fps, w, h = 30, im0.shape[1], im0.shape[0]

            #         vid_writer = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
            #     vid_writer.write(im0)

        #my add    show frame #####################################################
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) == ord('q'):
            break
        # map = cv2.warpPerspective(frame, matrix, (640, 480))
        cv2.circle(map, [228,408], 5, 255, -1)
        cv2.imshow('MapPerser', map)
        if cv2.waitKey(1) == ord('q'):
            break
        
        cv2.circle(img_base, [robot_x, robot_y], 3, (0, 100, 0), -1)
        cv2.circle(img_base, [228,408], 10, 255, -1)
        cv2.imshow('MapBase', img_base)
        #print(robot_x, robot_y)
        if cv2.waitKey(1) == ord('q'):
            break
        #my add   ###################################################################

    # Print results
    t = tuple(x / seen * 1E3 for x in dt)  # speeds per image
    LOGGER.info(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS, %.1fms deep sort update \
        per image at shape {(1, 3, *imgsz)}' % t)
    
    my_video.camera_release()
    # if save_txt or save_vid:
    #     print('Results saved to %s' % save_path)
    #     if platform == 'darwin':  # MacOS
    #         os.system('open ' + save_path)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # parser.add_argument('--yolo_model', nargs='+', type=str, default='yolov5s.pt', help='model.pt path(s)')
    parser.add_argument('--yolo_model', nargs='+', type=str, default='327_03/best.pt', help='model.pt path(s)')
    parser.add_argument('--deep_sort_model', type=str, default='deep_sort/deep/checkpoint/osnet_ibn_x1_0_imagenet.pth')
    #parser.add_argument('--source', type=str, default='0', help='source')  # file/folder, 0 for webcam
    parser.add_argument('--output', type=str, default='inference/output', help='output folder')  # output folder
    parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
    parser.add_argument('--conf-thres', type=float, default=0.5, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.5, help='IOU threshold for NMS')
    parser.add_argument('--fourcc', type=str, default='mp4v', help='output video codec (verify ffmpeg support)')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--show-vid', action='store_true', help='display tracking video results')
    parser.add_argument('--save-vid', action='store_true', help='save video tracking results')
    parser.add_argument('--save-txt', action='store_true', help='save MOT compliant results to *.txt')
    # class 0 is person, 1 is bycicle, 2 is car... 79 is oven
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 16 17')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--evaluate', action='store_true', help='augmented inference')
    parser.add_argument("--config_deepsort", type=str, default="deep_sort/configs/deep_sort.yaml")
    parser.add_argument("--half", action="store_true", help="use FP16 half-precision inference")
    parser.add_argument('--visualize', action='store_true', help='visualize features')
    parser.add_argument('--max-det', type=int, default=1000, help='maximum detection per image')
    parser.add_argument('--dnn', action='store_true', help='use OpenCV DNN for ONNX inference')
    parser.add_argument('--project', default=ROOT / 'runs/track', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    opt = parser.parse_args()
    opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand

    with torch.no_grad():
        detect(opt)
