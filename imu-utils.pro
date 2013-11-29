#-------------------------------------------------
#
# Project created by QtCreator 2013-09-18T13:57:44
#
#-------------------------------------------------

QT       += core gui network

TARGET = imu-utils
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

target.path = /usr/bin/
INSTALLS += target

INCLUDEPATH += /home/timon/opt/rasp-pi-rootfs/opt/ros/groovy/include/

LIBS += -L/home/timon/opt/rasp-pi-rootfs/opt/ros/groovy/lib/ \
        -lcpp_common\
        -lroslib \
        -lrospack \
        -lroscpp \
        -lrosconsole \
        -lrospack \
        -lrostime \
        -lxmlrpcpp \
        -lroscpp_serialization

SOURCES += main.cpp \
    core.cpp \
    imusensor.cpp \
    tcpserver.cpp \
    dataprocessor.cpp \
    imusensorcalibrator.cpp \
    dcmfilter.cpp \
    qnode.cpp \
    rosodometrylistener.cpp

HEADERS += \
    core.h \
    imusensor.h \
    tcpserver.h \
    dataprocessor.h \
    imusensorcalibrator.h \
    dcmfilter.h \
    qnode.h \
    rosodometrylistener.h
