#include "head.h"

#define LINE_10 "----------"
#define LINE120 LINE_10 LINE_10 LINE_10 LINE_10 LINE_10 LINE_10 LINE_10 LINE_10 LINE_10 LINE_10 LINE_10 LINE_10
#define LINE_IND LINE120
#define LINE_TIP_FMT		"|%-10s|%-10s|%-48s|%-48s|\n"
#define LINE_CONTENT_FMT	"|%-10d|%-10s|%-48s|%-48s|\n"
#define USE

int main()
{

#ifndef USE
    vector<serial::PortInfo> devices_found = serial::list_ports();
    vector<serial::PortInfo>::iterator iter = devices_found.begin();

    // 选择串口
    // 顶住左边开始, 每30个字符为一个显示字段
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

    rm::ReceivedData data;

#ifdef _WIN32
    serial::Serial my_serial("COM5", 9600, serial::Timeout::simpleTimeout(1000));
#else
    serial::Serial my_serial("/dev/ttyUSB0", 460800, serial::Timeout::simpleTimeout(1000));
#endif
    serial::Serial* sp = &my_serial;
    if (my_serial.isOpen())cout<<"Serial open succed!"<<endl;
    cout<<my_serial.getStopbits()<<endl;
    cout<<my_serial.getBaudrate()<<endl;
    //rm::openport(my_serial);
    while (1)
    {
        rm::try_read(my_serial,sp, &data, sizeof(rm::ReceivedData));
    }
    return 0;
}
