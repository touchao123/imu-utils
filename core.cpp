#include <QDebug>

#include "core.h"

Core::Core(QStringList args, QObject *parent) :
    m_args(args),QObject(parent)
{

    m_device = "/dev/i2c-1";
    m_delay = 20;
    m_port = 55555;

    m_dataProcessor = new DataProcessor(this);

    calibration();
    setupSensor();
    setupTcpServer();

    // global connections
    connect(m_sensor,SIGNAL(sensorDataAvailable(QVector3D,QVector3D,QVector3D)),m_dataProcessor,SLOT(calculateCalibration(QVector3D,QVector3D,QVector3D)));

    // user specifys which data get printed to the console
    if(m_args.contains("-vr") || m_args.contains("--verboseRaw")){
        connect(m_sensor,SIGNAL(sensorDataAvailable(QVector3D,QVector3D,QVector3D)),this,SLOT(printData(QVector3D,QVector3D,QVector3D)));
    }
    if(m_args.contains("-vc") || m_args.contains("--verboseCal")){
        connect(m_dataProcessor,SIGNAL(calibDataReady(QVector3D,QVector3D,QVector3D)),this,SLOT(printData(QVector3D,QVector3D,QVector3D)));
    }

    m_sensor->enableSensor();
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

void Core::printData(const QVector3D &accData, const QVector3D &gyroData, const QVector3D &magData)
{
    printf("\r                                                                                             ");
    printf("\rAcc = [ %0.2f | %0.2f | %0.2f ]",accData.x(), accData.y(),accData.z());
    printf("\tGyro = [ %0.2f | %0.2f | %0.2f ]",gyroData.x(),gyroData.y(),gyroData.z());
    printf("\tMag = [ %0.2f | %0.2f | %0.2f ]",magData.x(),magData.y(),magData.z());
    fflush(stdout);

}
