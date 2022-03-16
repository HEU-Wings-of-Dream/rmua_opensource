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

    struct ReceivedData
        {
            uint8_t my_SOF;  //1
            uint8_t myteam; //1  EE DD
            int16_t vx;    //2
            int16_t vy;    //2
            float omiga;    //4
            uint8_t my_EOF;    //1
        };//__attribute__((_packed));


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

    void try_read(serial::Serial& my_serial, serial::Serial* sp, ReceivedData* receiveddata, size_t buffer_size)
    {
        if (my_serial.isOpen() != true){
            cout << "Serial is already closed, please check it!" << endl;
            return;
        }

        uint8_t* my_buffer = new uint8_t[sizeof(ReceivedData)];
        memset(my_buffer, 0, sizeof(ReceivedData));
        serial::Serial::ScopedReadLock aa(sp);
        my_serial.flush();
        my_serial.read(my_buffer, buffer_size);//????
        memcpy(receiveddata, my_buffer, buffer_size);//??????
        //~my_readlock;
        //delete(my_buffer);

        if (receiveddata->my_SOF != (uint8_t)0x66){
            cout << "SOF error     "  ;
            printf("%02x\n",receiveddata->my_SOF);
            memset(my_buffer, 0, sizeof(ReceivedData));
            delete[] my_buffer;
            return;
        }
        if (receiveddata->my_EOF != (uint8_t)0x88) {
            cout << "EOF error     " ;
            printf("%02x\n",receiveddata->my_EOF);
            memset(my_buffer, 0, sizeof(ReceivedData));
            delete[] my_buffer;
            return;
        }
        memset(my_buffer, 0, sizeof(ReceivedData));
        delete[] my_buffer;
        printf("%f\n",receiveddata->omiga);
        return;
    }
}
