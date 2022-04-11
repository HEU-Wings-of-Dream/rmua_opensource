#define ROS 1

#if ROS
#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Accel.h>
//#include <port_listener_core/Posemsg.h>
#include "port_listener_core/velocity.h"
#include "port_listener_core/race_state.h"
#include "port_listener_core/my_control_frame.h"
#endif

#include "port_listener_core/head.h"
#include <chrono>

#define LINE_10 "----------"
#define LINE120 LINE_10 LINE_10 LINE_10 LINE_10 LINE_10 LINE_10 LINE_10 LINE_10 LINE_10 LINE_10 LINE_10 LINE_10
#define LINE_IND LINE120
#define LINE_TIP_FMT		"|%-10s|%-10s|%-48s|%-48s|\n"
#define LINE_CONTENT_FMT	"|%-10d|%-10s|%-48s|%-48s|\n"
#define PRINT_ALL_COM 1

#if ROS
port_listener_core :: velocity msg;
port_listener_core :: my_control_frame control_frame;
rm::true_autoaim_struct auto_aim_control_frame;
bool need_to_send_move_control_flag = 0;
bool need_to_send_autoaim_flag = 0;

void robot_control_callback(const port_listener_core :: my_control_frame :: ConstPtr& msg_received)
{
    control_frame.vx = msg_received->vx;
    control_frame.vy = msg_received->vy;
    control_frame.angle = msg_received->angle;
    need_to_send_move_control_flag = 1;
}

void autoaim_callback(const geometry_msgs :: Accel :: ConstPtr& msg_received)
{
    auto_aim_control_frame.my_SOF = 0x66;
    auto_aim_control_frame.flg = 1;
    auto_aim_control_frame.x = msg_received->linear.x;
    auto_aim_control_frame.y = msg_received->linear.y;
    auto_aim_control_frame.z = msg_received->linear.z;
    auto_aim_control_frame.reserve1 = msg_received->angular.x;
    auto_aim_control_frame.reserve2 = msg_received->angular.y;
    auto_aim_control_frame.time_stamp = msg_received->angular.z;
    auto_aim_control_frame.my_EOF = 0x88;
    need_to_send_autoaim_flag = 1;
}

#endif

int main(int argc, char **argv)
{

#if PRINT_ALL_COM
    vector<serial::PortInfo> devices_found = serial::list_ports();
    vector<serial::PortInfo>::iterator iter = devices_found.begin();

    // ѡ�񴮿�
    // ��ס��߿��?, ÿ30���ַ�Ϊһ����ʾ�ֶ�
    printf("%s\n", LINE_IND);
    printf(LINE_TIP_FMT, "port_sn", "port_name", "port_desc", "port_hd_id");
    printf("%s\n", LINE_IND);

    int dev_port = 0;
    while (iter != devices_found.end())
    {
        serial::PortInfo device = *iter++;

        printf(LINE_CONTENT_FMT, dev_port++, device.port.c_str(), device.description.c_str(), device.hardware_id.c_str());
        printf("%s\n", LINE_IND);
    }
#endif // !USE

    rm::my_sleep(1000);

#ifdef _WIN32
    serial::Serial my_serial("COM6", 460800, serial::Timeout::simpleTimeout(1000));
#else
    serial::Serial my_serial("/dev/ttyUSB1", 1500000, serial::Timeout::simpleTimeout(1000));
#endif

#if ROS
    ros::init(argc, argv, "portlistener");
    ros::NodeHandle communication_handle_obj;

    ros::Publisher velocity_publisher =
        communication_handle_obj.advertise <port_listener_core :: velocity>("/velocity", 10);

    ros::Publisher race_state_publisher =
        communication_handle_obj.advertise <port_listener_core :: race_state>("/race_state", 10);

    ros::Publisher autoaim_publisher =
        communication_handle_obj.advertise <port_listener_core::velocity>("/auto_aim", 10);

    ros::Subscriber autoaim_listener = communication_handle_obj.subscribe("/robot_control", 10, robot_control_callback);

    ros::Subscriber true_autoaim_listener = communication_handle_obj.subscribe("/auto_aim_task", 10, autoaim_callback);
#endif

    serial::Serial* sp = &my_serial;
    if (my_serial.isOpen())cout << "Serial open succed!" << endl;
    //cout << my_serial.getStopbits() << endl;
    cout << "Now Baudrate is:  " << my_serial.getBaudrate() << endl;
    cout << sizeof(rm::ControlFrame) << endl;
    //rm::openport(my_serial);

    int which_bag = 0;
    rm::test_struct data;
    rm::ControlFrame ControlFrame_send_bag;

    rm::autoaim_feedback_struct autoaim_feedback_struct_sned_bag; 
    rm::race_state_struct race_state_struct_send_bag; port_listener_core::race_state race_state_sned_buffer;
    rm::move_control_struct move_control_struct_send_bag;  port_listener_core::velocity velocity_send_buffer;

    uint8_t send_buffer[25];//only auto_aim frame can use it
   uint8_t send_buffer2[25];//only move_control frame can use it
    
    memset(&ControlFrame_send_bag, 0, sizeof(rm::test_bag));

    bool need_to_end_flag = 0;
    int lost_goal = 0;
    int lost_move = 0;
    rm::union_send_bag temp;
    int USE_UNION_STRUCT = 1;
    while (ros::ok())
    {
#if ROS
if( USE_UNION_STRUCT == 0){
        if (need_to_send_move_control_flag == 1){
            rm::move_control_struct send_temp_struct;
            memset(&send_temp_struct, 0, sizeof(send_temp_struct));
            send_temp_struct.my_SOF = 0x77;
            send_temp_struct.my_EOF = 0x99;
            send_temp_struct.vx = control_frame.vx*1000; 
            send_temp_struct.vy = control_frame.vy*1000;
            send_temp_struct.omiga = control_frame.angle;
            //printf("vx == %f   vy == %f,  omiga == %f", control_frame.vx, control_frame.vy, send_temp_struct.omiga);
            memcpy(&send_buffer2, &send_temp_struct, sizeof(rm::test_struct));
            need_to_send_move_control_flag = 0;
            //for (int i = 0; i <= 24; i++) {
            my_serial.write(send_buffer2, sizeof(send_buffer2));
                //printf("%02x ", send_buffer2[i]);
            
            printf("send move_control message succed!!\n");
            lost_move = 0;
        }
        else{
            if (lost_move <= 50) lost_move ++;
            else{
                rm::move_control_struct send_temp_struct;
                memset(&send_temp_struct, 0, sizeof(send_temp_struct));
                send_temp_struct.my_SOF = 0x77;
                send_temp_struct.my_EOF = 0x99;
                send_temp_struct.vx = control_frame.vx*1000 * 0;
                send_temp_struct.vy = control_frame.vy*1000 * 0;
                send_temp_struct.omiga = control_frame.angle * 0;
                //printf("vx == %f   vy == %f,  omiga == %f", control_frame.vx, control_frame.vy, send_temp_struct.omiga);
                memcpy(&send_buffer2, &send_temp_struct, sizeof(rm::test_struct));
                need_to_send_move_control_flag = 0;
                my_serial.write(send_buffer2, sizeof(send_buffer2));
                printf("lost_move = %d\n", lost_move);
            }
        }
}
else{
    if (need_to_send_autoaim_flag == 1 && need_to_send_move_control_flag == 1){
        lost_goal = 0;
        rm::union_send_bag send_struct; 
        send_struct.vx = control_frame.vx;
        send_struct.vy = control_frame.vy;
        send_struct.omiga = control_frame.angle;
        send_struct.flg = 1;
        send_struct.reserve1 = auto_aim_control_frame.reserve1;
        send_struct.reserve2 = auto_aim_control_frame.reserve2;
        send_struct.x = auto_aim_control_frame.x;
        send_struct.y = auto_aim_control_frame.y;
        send_struct.z = auto_aim_control_frame.z;
        send_struct.time_stamp = auto_aim_control_frame.time_stamp;
        
        memcpy(&send_buffer2, &send_struct, sizeof(send_struct));
        my_serial.write(send_buffer2, sizeof(send_buffer2));

        need_to_send_move_control_flag = 0;
        need_to_send_autoaim_flag = 0;
        printf("send union success flag = 1!\n");
    }
    if (need_to_send_move_control_flag == 1){
        rm::union_send_bag send_struct;
        send_struct.vx = control_frame.vx;
        send_struct.vy = control_frame.vy;
        send_struct.omiga = control_frame.angle;
        send_struct.flg = 3;
        temp.vx = control_frame.vx;
        temp.vy = control_frame.vy;
        temp.omiga = control_frame.angle;

        memcpy(&send_buffer2, &send_struct, sizeof(send_struct));
        my_serial.write(send_buffer2, sizeof(send_buffer2));

        need_to_send_move_control_flag = 0;
        printf("send move_control success! flag = 3\n");
    }
    if (need_to_send_autoaim_flag == 1){
        lost_goal = 0;
        rm::union_send_bag send_struct;
        send_struct.flg = 2;
        send_struct.reserve1 = auto_aim_control_frame.reserve1;
        send_struct.reserve2 = auto_aim_control_frame.reserve2;
        send_struct.x = auto_aim_control_frame.x;
        send_struct.y = auto_aim_control_frame.y;
        send_struct.z = auto_aim_control_frame.z;
        send_struct.time_stamp = auto_aim_control_frame.time_stamp;
        
        memcpy(&send_buffer2, &send_struct, sizeof(send_struct));
        my_serial.write(send_buffer2, sizeof(send_buffer2));

        need_to_send_autoaim_flag = 0;
        printf("sizeof = %d\n", sizeof(send_buffer));
        printf("send autoaim success! flag  2\n");
    }
    else{
        lost_goal++;
    if (lost_goal >= 100){
        rm::union_send_bag send_struct;
        send_struct.vx = temp.vx;
        send_struct.vy = temp.vy;
        send_struct.omiga = temp.omiga;
        send_struct.flg = 0;
        send_struct.reserve1 = auto_aim_control_frame.reserve1;
        send_struct.reserve2 = auto_aim_control_frame.reserve2;
        send_struct.x = auto_aim_control_frame.x;
        send_struct.y = auto_aim_control_frame.y;
        send_struct.z = auto_aim_control_frame.z;
        send_struct.time_stamp = auto_aim_control_frame.time_stamp;
        
        memcpy(&send_buffer2, &send_struct, sizeof(send_struct));
        my_serial.write(send_buffer2, sizeof(send_buffer2));

        printf("send autoaim success! flag = 0\n");
    }
    }
    
}
        which_bag = rm::try_read(my_serial, sp, &data);

        if (which_bag == 0) {
            rm::move_control_struct send_temp_struct;
            send_temp_struct.my_SOF = 0x77;
            send_temp_struct.empty[0] = 1;
            send_temp_struct.my_EOF = 0X99;

            memcpy(&send_buffer2, &send_temp_struct, sizeof(rm::test_struct));
            my_serial.write(send_buffer2, sizeof(send_buffer2));

            cout << "bad bag!!, i had reset it !" << endl;
        }

        if (which_bag == 3) {
            rm::move_control_struct move_control_struct_msg;
            memcpy(&move_control_struct_msg, &data, sizeof(rm::test_struct));
            //printf("Received move_coltrol bag!\n");

            velocity_send_buffer.vx = move_control_struct_msg.vy;
            velocity_send_buffer.vy = move_control_struct_msg.vx;
            velocity_send_buffer.omiga = move_control_struct_msg.omiga;

            velocity_publisher.publish(velocity_send_buffer);
        }

        if (which_bag == 1) {
            rm::race_state_struct race_state_struct_msg;
            memcpy(&race_state_struct_msg, &data, sizeof(rm::test_struct));
            //printf("Received race_state_struct bag!\n");

            race_state_sned_buffer.myteam = race_state_struct_send_bag.myteam;
            race_state_sned_buffer.robot_id = race_state_struct_send_bag.robot_id;
            race_state_sned_buffer.enemy_1_bullet_left = race_state_struct_send_bag.enemy_1_bullet_left;
            race_state_sned_buffer.enemy_1_HP_left = race_state_struct_send_bag.enemy_1_HP_left;
            race_state_sned_buffer.enemy_2_bullet_left = race_state_struct_send_bag.enemy_2_bullet_left;
            race_state_sned_buffer.enemy_2_HP_left = race_state_struct_send_bag.enemy_2_HP_left;
            race_state_sned_buffer.self_1_bullet_left = race_state_struct_send_bag.self_1_bullet_left;
            race_state_sned_buffer.self_1_HP_left = race_state_struct_send_bag.self_1_HP_left;
            race_state_sned_buffer.self_2_bullet_left = race_state_struct_send_bag.self_2_bullet_left;
            race_state_sned_buffer.self_2_HP_left = race_state_struct_send_bag.self_2_HP_left;
            race_state_sned_buffer.lurk_mode = race_state_struct_send_bag.lurk_mode;
            for (int i = 0; i <=5 ; i++){
                race_state_sned_buffer.zone[i] = race_state_struct_send_bag.zone[i];
                race_state_sned_buffer.zone_status[i] = race_state_struct_send_bag.zone_status[i];
            }

            race_state_publisher.publish(race_state_sned_buffer);
        }

        if (which_bag == 2) {
            rm::autoaim_feedback_struct autoaim_feedback_struct_msg;
            memcpy(&autoaim_feedback_struct_msg, &data, sizeof(rm::test_struct));
            printf("Received autoaim_feedback_struct bag!\n");
        }
if ((USE_UNION_STRUCT == 0)){
        if (need_to_send_autoaim_flag == 1){
            // rm::move_control_struct send_temp_struct;
            // send_temp_struct.my_SOF = 0x66;
            // send_temp_struct.my_EOF = 0x88;
            // send_temp_struct.vx = control_frame.vx*100;
            // send_temp_struct.vy = control_frame.vy*100;
            // send_temp_struct.omiga = control_frame.angle;
            // printf("vx == %f   vy == %f,  omiga == %f", control_frame.vx, control_frame.vy, send_temp_struct.omiga);
            memcpy(&send_buffer2, &auto_aim_control_frame, sizeof(rm::test_struct));

            my_serial.write(send_buffer2, sizeof(send_buffer2));
            need_to_send_autoaim_flag = 0;
                 
            printf("send autoaim message succed!!\n");
            lost_goal = 0;
        }
        else{
            //printf("%d\n", ((static_cast<std::chrono::duration<double, std::milli>>(std::chrono::high_resolution_clock::now() - lasttime)).count()));
            if (lost_goal <= 50) lost_goal++;
            else{
                memset(&auto_aim_control_frame, 0, sizeof(auto_aim_control_frame));
                auto_aim_control_frame.my_SOF = 0x66;
                auto_aim_control_frame.my_EOF = 0x88;
                memcpy(&send_buffer2, &auto_aim_control_frame, sizeof(rm::test_struct));
                my_serial.write(send_buffer2, sizeof(send_buffer2));
            }
            printf("lost count = %d\n", lost_goal);
        }
}
        ros::spinOnce();
#endif
    }

    return 0;
}
