TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp   \
    ./src/serial.cc     \
    ./src/impl/unix.cc     \
    ./src/impl/win.cc    \
    ./src/impl/list_ports/list_ports_linux.cc  \
    ./src/impl/list_ports/list_ports_win.cc  \
    ./src/impl/list_ports/list_ports_osx.cc  \


HEADERS += \
    head.h    \
    ./serial/serial.h   \
    ./serial/v8stdint.h  \
    ./serial/impl/unix.h  \
    ./serial/impl/win.h
