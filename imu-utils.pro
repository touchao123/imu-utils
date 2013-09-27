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

SOURCES += main.cpp \
    core.cpp \
    imusensor.cpp \
    tcpserver.cpp \
    dataprocessor.cpp \
    imusensorcalibrator.cpp \
    kalmanfilter.cpp

HEADERS += \
    core.h \
    imusensor.h \
    tcpserver.h \
    dataprocessor.h \
    imusensorcalibrator.h \
    kalmanfilter.h
