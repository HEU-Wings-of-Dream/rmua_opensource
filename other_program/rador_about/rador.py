from unicodedata import name
import opencv as cv

count = 0
point_save = [[0,0]]

def mouse_callback(event, x, y, flags, param):
    if event == cv.EVENT_LBUTTONDBLCLK:
        count = count + 1
        save.append([x,y])

if __name__ == '__main__':
    cv.namedWindow('img')
    cv.setMouseCallback('img', mouse_callback)
    
    while (count != 4):
        for i in point_save:
            if (i[0] != 0): #如果不是头一个
                cv.circle(img, (x,y), 5, 255, -1)  #画圆,最后一个参数本来是线宽，但是-1代表填充
        cv.imshow("img", img)
        cv.waitKey(10)
    
    for i in bounding_box_list:
        cv.rectangle()
        cv.imshow("img", img)
        cv.waitKey(10)