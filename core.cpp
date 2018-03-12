/*
 *    This application is able to read the IMU 9-DOF sensor stick
 *    from Sparfunk SEN-10724.
 *
 *    Copyright (C) 2013 Simon Stürz (stuerz.simon@gmail.com)
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <QDebug>

#include "core.h"
#include <math.h>

Core::Core(QStringList args, int argc, char **argv, QObject *parent) :
    m_args(args),m_argc(argc),m_argv(argv),QObject(parent)
{
    m_device = "/dev/i2c-1";
    m_delay = 20;
    m_port = 55555;

    m_dataProcessor = new DataProcessor(this);


    calibration();
    setupSensor();
    setupTcpServer();


    // global connections
    connect(m_sensor,SIGNAL(sensorDataAvailable(QVector3D,QVector3D,QVector3D,int)),m_dataProcessor,SLOT(processData(QVector3D,QVector3D,QVector3D,int)));

    // user specifys which data get printed to the console
    if(m_args.contains("-vr") && !m_args.contains("-vc") && !m_args.contains("-va")){
        connect(m_sensor,SIGNAL(sensorDataAvailable(QVector3D,QVector3D,QVector3D,int)),this,SLOT(printData(QVector3D,QVector3D,QVector3D,int)));
    }
    if(m_args.contains("-vc") && !m_args.contains("-vr") && !m_args.contains("-va")){
        connect(m_dataProcessor,SIGNAL(calibratedDataReady(QVector3D,QVector3D,QVector3D,int)),this,SLOT(printData(QVector3D,QVector3D,QVector3D,int)));
    }
    if(m_args.contains("-va") && !m_args.contains("-vc") && !m_args.contains("-vr")){
        connect(m_dataProcessor,SIGNAL(anglesReady(QVector3D,QVector3D)),this,SLOT(printAngles(QVector3D,QVector3D)));
    }

    m_sensor->enableSensor();
    setupRos();

}

Core::~Core()
{
    m_rosThread->quit();
    m_rosThread->wait();
}

void Core::calibration()
{
    // check if user wants to calibrate acc sensor
    if((m_args.contains("-ca"))){
        if(m_args.contains("-v") || m_args.contains("--verbose")){
            qDebug() << "verbose parameter not allowed in calibration";
            exit(1);
        }
        if(m_args.contains("-f") || m_args.contains("--frequency")){
            qDebug() << "specify the frequency parameter not allowed in calibration";
            exit(1);
        }
        // calibrate...
        setupSensor();
        qDebug() << "start calibrating accelerometer sensors...";
        m_calibrator = new ImuSensorCalibrator(m_sensor,this);
        m_calibrator->calibrateAcc();
        exit(0);
    }

    // check if user wants to calibrate gyroscope sensor
    if((m_args.contains("-cg"))){
        if(m_args.contains("-v") || m_args.contains("--verbose")){
            qDebug() << "verbose parameter not allowed in calibration";
            exit(1);
        }
        if(m_args.contains("-f") || m_args.contains("--frequency")){
            qDebug() << "specify the frequency parameter not allowed in calibration";
            exit(1);
        }
        // calibrate...
        setupSensor();
        qDebug() << "start calibrating gyro sensors...";
        m_calibrator = new ImuSensorCalibrator(m_sensor,this);
        m_calibrator->calibrateGyr();
        exit(0);
    }

    // check if user wants to calibrate magnetometer sensor
    if((m_args.contains("-cm"))){
        if(m_args.contains("-vc") || m_args.contains("-vr") || m_args.contains("-va")){
            qDebug() << "verbose parameter not allowed in calibration";
            exit(1);
        }
        if(m_args.contains("-f") || m_args.contains("--frequency")){
            qDebug() << "specify the frequency parameter not allowed in calibration";
            exit(1);
        }
        // calibrate...
        setupSensor();
        qDebug() << "start calibrating gyro sensors...";
        m_calibrator = new ImuSensorCalibrator(m_sensor,this);
        m_calibrator->calibrateMag();
        exit(0);
    }

}



void Core::setupSensor()
{
    // check if user wants a certain device
    if(m_args.contains("-d") || m_args.contains("--device")){
        QString device;
        int pos;
        if(m_args.contains("-d")){
            pos = m_args.indexOf("-d");
        }
        if(m_args.contains("--device")){
            pos = m_args.indexOf("--device");
        }
        device = m_args.at(pos+1);
        if (!device.startsWith("/")){
            qDebug() << "please give the absolute path to the I2C-device.";
            qDebug() << "example: /dev/i2c-1";
        }else{
            m_device = device;
        }
    }

    //check if user wants a certain measurement frequency
    if(m_args.contains("-f") || m_args.contains("--frequency")){
        QString freq;
        int pos;
        if(m_args.contains("-f")){
            pos = m_args.indexOf("-f");
        }
        if(m_args.contains("--frequency")){
            pos = m_args.indexOf("--frequency");
        }
        if(m_args.at(pos+1).startsWith("-")){
            qDebug() << "please use a frequency between 1 [Hz] and 75 [Hz]";
            qDebug() << "example: imu-utils -f 10";
            exit(1);
        }
        freq = m_args.at(pos+1);
        bool ok;
        int freq_int = freq.toInt(&ok,10);
        if(ok && freq_int > 0 && freq_int <= 75){
            m_delay = 1000/freq_int;
        }else{
            qDebug() << "please use a frequency between 1 [Hz] and 75 [Hz]";
            qDebug() << "example: imu-utils -f 10";
            exit(1);
        }
    }

    m_sensor = new ImuSensor(m_device,m_delay,this);

}

void Core::setupTcpServer()
{
    //check if user wants a certain port for the TCP server
    if(m_args.contains("-p") || m_args.contains("--port")){
        QString port;
        int pos;
        if(m_args.contains("-p")){
            pos = m_args.indexOf("-p");
        }
        if(m_args.contains("--port")){
            pos = m_args.indexOf("--port");
        }
        if(m_args.at(pos+1).startsWith("-")){
            qDebug() << "please use a port between 49152 - 65535";
            qDebug() << "example: imu-utils -t -p 525252";
            exit(1);
        }
        port = m_args.at(pos+1);
        bool ok;
        int port_int = port.toInt(&ok,10);
        // 49152–65535 contains dynamic or private ports that cannot be registered with IANA
        if(ok && port_int >=49152 && port_int <= 65535){
            m_port = port_int;
        }else{
            qDebug() << "please use a port between 49152 - 65535";
            qDebug() << "example: imu-utils -t -p 525252";
            exit(1);
        }
    }

    //check if the user wants a TCP Server
    if(m_args.contains("-t") || m_args.contains("--tcp")){
        m_server = new TcpServer(m_port,this);
        m_server->startServer();
        connect(m_dataProcessor,SIGNAL(dataTcpReady(QByteArray)),m_server,SLOT(sendToAll(QByteArray)));
    }
}

void Core::setupRos()
{
    //check if the user wants enable ROS support
    if(m_args.contains("-r") || m_args.contains("--ros")){
        m_rosNode = new RosNode(this,m_argc,m_argv,"imu_utils");
        m_rosNode->start();
        connect (m_dataProcessor,SIGNAL(anglesReady(QVector3D,QVector3D)),m_rosNode,SLOT(publishData(QVector3D,QVector3D)));
        connect(m_rosNode,SIGNAL(finished()),this,SLOT(rosFinished()));
    }
}

void Core::rosFinished()
{
    qDebug() << "ROS thread finished.";
    exit(0);
}

void Core::printData(const QVector3D &accData, const QVector3D &gyroData, const QVector3D &magData, const int &dt)
{
    printf("\r                                                                                                                                                         ");
    printf("\r  Acc = [ %0.2f | %0.2f | %0.2f ]",accData.x(), accData.y(),accData.z());
    printf("    Gyro = [ %0.2f | %0.2f | %0.2f ]",gyroData.x(),gyroData.y(),gyroData.z());
    printf("    Mag = [ %0.2f | %0.2f | %0.2f ]",magData.x(),magData.y(),magData.z());
    printf("    dt = %i\r",dt);
    fflush(stdout);

}

void Core::printAngles(const QVector3D &angles, const QVector3D &anglesVel)
{
    printf("\r                                                                                                                                                         ");
    printf("\r  Roll =  %0.2f     Pitch = %0.2f     Yaw = %0.2f",angles.x()*180/M_PI, angles.y()*180/M_PI,angles.z()*180/M_PI);
    fflush(stdout);
}
