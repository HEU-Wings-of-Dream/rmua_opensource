#Configs
TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += c++14

#Libraries
unix: CONFIG += link_pkgconfig

#OpenCV
unix: PKGCONFIG += opencv

#CUDA
#unix: LIBS += -L$$PWD/../../../usr/local/cuda/lib64/-lcudart
#unix: LIBS += -L$$PWD/../../../usr/local/cuda/lib64/-lcuda
#unix: LIBS += -L$$PWD/../../../usr/local/cuda/lib64/-lcublas
#unix: LIBS += -L$$PWD/../../../usr/local/cuda/lib64/-lcurand
#INCLUDEPATH += $$PWD/../../../usr/local/cuda/include
#DEPENDPATH += $$PWD/../../../usr/local/cuda/include

#V4L2 摄像头驱动
unix: LIBS += -lpthread
unix: LIBS += -lv4l2

#MDVS Driver
LIBS += -lMVSDK

#Darknet
#unix: LIBS += -L$$PWD/Darknet/ -ldarknet
#INCLUDEPATH += $$PWD/Darknet
#DEPENDPATH += $$PWD/Darknet

#Source and header files
SOURCES += \
	Armor/armor.cpp \
	Armor/ArmorPredict/armor_predict.cpp \
	Armor/ArmorTrack/armor_track.cpp \
	Armor/FindLightBar/find_light_bar.cpp \
	Armor/RectangleMacth/rectangle_match.cpp \
	Others/Camera/RMVideoCapture.cpp \
	Others/DataFrame/data_frame.cpp \
	Others/SolvePosition/solve_position.cpp \
	Others/Trajectory/trajectory.cpp \
	Run/run.cpp \
	Rune/Wind.cpp \
	Serials/Serial.cpp \
	main.cpp \
    number_detector/svm.cpp


HEADERS += \
	Armor/armor.h \
	Armor/ArmorPredict/armor_predict.h \
	Others/Camera/RMVideoCapture.hpp \
	Others/DataFrame/data_frame.h \
	Others/SolvePosition/solve_position.h \
	Others/Trajectory/trajectory.h \
	Run/run.h \
	Rune/Wind.hpp \
	Serials/Serial.h \
	define.h \
	Others/General/opencv_owns.hpp \
    number_detector/svm.h

DISTFILES += \
	Others/SolvePosition/angle_solver_params.xml

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/glog/lib/release/ -lglog
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/glog/lib/debug/ -lglog
else:unix: LIBS += -L$$PWD/glog/lib/ -lglog

INCLUDEPATH += $$PWD/glog/include
DEPENDPATH += $$PWD/glog/include

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/glog/lib/release/libglog.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/glog/lib/debug/libglog.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/glog/lib/release/glog.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/glog/lib/debug/glog.lib
else:unix: PRE_TARGETDEPS += $$PWD/glog/lib/libglog.a



