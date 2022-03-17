#pragma once
#pragma pack(1)
#include <iostream>
#include <stddef.h>
#include <cstdio>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <serial/serial.h>

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

namespace rm {
    struct move_control_struct //18
    {
        uint8_t my_SOF;  //1
        uint8_t myteam; //1  EE DD
        int16_t vx;    //2
        int16_t vy;    //2
        float omiga;    //4
        uint8_t empty;
        int32_t empty1;
        double empty2;
        uint8_t my_EOF;    //1
    };

    struct race_state_struct //18
    {
        uint8_t my_SOF;  //1
        uint8_t myteam; //1  EE DD
        uint8_t robot_id;
        uint8_t zone[6];
        uint8_t zone_status[6];
        uint8_t self_1_bullet_left;
        uint8_t self_1_HP_left;
        uint8_t self_2_bullet_left;
        uint8_t self_2_HP_left;
        uint8_t enemy_1_bullet_left;
        uint8_t enemy_1_HP_left;
        uint8_t enemy_2_bullet_left;
        uint8_t enemy_2_HP_left;
        uint8_t lurk_mode;
        uint8_t my_EOF;    //1
    };

    struct autoaim_feedback_struct //66 88
    {
        uint8_t FDBF_SOF;
        uint8_t myteam; //  EE DD
        short pitch;    //  10k*rad
        short yaw;
        uint16_t bullet_speed; // m/s
        uint32_t time_stamp;   // ms
        short reserve;
        uint8_t mode;
        double empty;
        uint8_t FDBF_EOF;
    };

    struct test_struct
    {
        uint8_t my_SOF;
        uint8_t byte2[22];
        uint8_t my_EOF;
    }test_bag;

    void my_sleep(unsigned long milliseconds) {
#ifdef _WIN32
        Sleep(milliseconds); // 100 ms
#else
        usleep(milliseconds * 1000); // 100 ms
#endif
    }

    void openport(serial::Serial& my_serial)
    {
        while (my_serial.isOpen())
        {
            my_serial.open();
            my_sleep(1000);
            cout << "Serial open failed!" << endl;
        }
        cout << "Serial open successed!!" << endl;
    }

    int try_read(serial::Serial& my_serial, serial::Serial* sp, rm::test_struct* data)
    {
        if (my_serial.isOpen() != true) {
            cout << "Serial is already closed, please check it!" << endl;
            return;
        }

        uint8_t* my_buffer = new uint8_t[sizeof(race_state_struct)];
        memset(my_buffer, 0, sizeof(move_control_struct));
        //serial::Serial::ScopedReadLock aa(sp);
        my_serial.flush();
        my_serial.read(my_buffer, sizeof(test_struct));
        
        memcpy(&test_bag, my_buffer, sizeof(test_struct));

        //~my_readlock;
        //delete(my_buffer);

        if (test_bag.my_SOF == (uint8_t)0x11 && test_bag.my_EOF == (uint8_t)0x22) {
            cout << "Read race inf success  "<<endl;
            printf("%02x,  %02x\n", test_bag.my_SOF, test_bag.my_EOF);
            memcpy(data, my_buffer, sizeof(test_struct));
            memset(my_buffer, 0, sizeof(test_struct));
            delete[] my_buffer;
            return 1;
        }

        if (test_bag.my_SOF == (uint8_t)0x66 && test_bag.my_EOF == (uint8_t)0x88) {
            cout << "Read navgiation success  "<<endl;
            printf("%02x,  %02x\n", test_bag.my_SOF, test_bag.my_EOF);
            memcpy(data, my_buffer, sizeof(test_struct));
            memset(my_buffer, 0, sizeof(test_struct));
            delete[] my_buffer;
            return 2;
        }

        if (test_bag.my_SOF == (uint8_t)0x22 && test_bag.my_EOF == (uint8_t)0x33) {
            cout << "Read auto aim success  " << endl;
            printf("%02x,  %02x\n", test_bag.my_SOF, test_bag.my_EOF);
            memcpy(data, my_buffer, sizeof(test_struct));
            memset(my_buffer, 0, sizeof(test_struct));
            delete[] my_buffer;
            return 3;
        }
        //printf("data1 :   %02x,  %02x   ", receiveddata->my_SOF, receiveddata->my_EOF);
        //printf("%02x,  %02x\n", receiveddata2->robot_id, receiveddata2->enemy_2_HP_left);

        for (int i=0;i<=sizeof(move_control_struct)-1;i++)
            printf("%02x ",my_buffer[i]);
        printf("\n\n");

        memset(my_buffer, 0, sizeof(move_control_struct));
        delete[] my_buffer;
        //printf("%f\n", receiveddata->omiga);
        return;
    }
}
//今后改成多线程，一个线程专门监听串口，一个线程专门在ROS话题上发布信息