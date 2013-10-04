#include <QCoreApplication>
#include <QStringList>
#include <QDebug>
#include <core.h>

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    QStringList arguments = a.arguments();
    if(arguments.contains("--help") || arguments.contains("-h")) {
        qDebug() << "Usage:" << arguments.first() << "[OPTION]...";
        qDebug() << "This application is able to read the IMU 9-DOF sensor stick";
        qDebug() << "from Sparfunk SEN-10724. The sensor contiains:";
        qDebug() << "   3-DOF accelerometer  -> ADXL345";
        qDebug() << "   3-DOF magnetometer   -> HMC5883L";
        qDebug() << "   3-DOF gyroscope      -> ITG-3200";
        qDebug() << "This application works for any Linux device that supports I2C communication,";
        qDebug() << "but it was written for the Raspberry Pi in Qt. To connect the sensor to the";
        qDebug() << "Raspberry Pi connect the sensor pin\'s as followes:";
        qDebug() << "   VDD ->             GPIO PIN 1 [3V3]";
        qDebug() << "   GND ->             GPIO PIN 6 [Ground]";
        qDebug() << "   SDA ->             GPIO PIN 3 [GPIO2]";
        qDebug() << "   SCL ->             GPIO PIN 5 [GPIO3]";
        qDebug();
        qDebug() << "options:";
        qDebug() << "   -h,     --help              print this help message";
        qDebug() << "   -vr,                        print sensor raw data to console";
        qDebug() << "   -vc,                        print calibrated sensor data to console";
        qDebug() << "   -va,                        print roll, pitch yaw angles to console";
        qDebug() << "   -d,     --device            specify the I2C-device where the sensor is connected";
        qDebug() << "                               default = /dev/i2c-1 on the Raspberry Pi";
        qDebug() << "   -ca,                        calibrate the acceleration sensor";
        qDebug() << "   -cg,                        calibrate the gyroscope";
        qDebug() << "   -cm,                        calibrate the magnetometer";
        qDebug() << "                               only allowed parameters for calibration are: -d";
        qDebug() << "   -f,     --frequency         specyfy measurement frequency 1 - 75 [Hz]";
        qDebug() << "                               maximum 75 [Hz] because its the max. rate of Magnetometer";
        qDebug() << "                               default value = 50 [Hz]";
        qDebug() << "   -t,     --tcp               enable TCP Server to default port 55555";
        qDebug() << "   -p,     --port              specify the port for the TCP Server (49152 - 65535)";
        qDebug() << "   -r,     --ros               enable ROS node";
        qDebug() << "   -T,     --topic             specify topic to subscribe the ROS node";
        qDebug();
        qDebug() << "examples:";
        qDebug() << "   - if we want to setup a TCP server that streams the data over default port 55555 with a frequency of 60 [Hz]:";
        qDebug() << "     imu-utils -t -f 60";
        qDebug() << "   - if we just want to read the sensordata without any ROS support or TCP server:";
        qDebug() << "     imu-utils -vc";
        qDebug() << "   - if we have multiple I2C devices available (ls -l /sys/class/i2c-adapter/) and want to";
        qDebug() << "     specify the I2C device where the sensor is connected:";
        qDebug() << "     imu-utils -d /dev/i2c-X           (X stands for the device number)";
        qDebug();
        exit(0);
    }

    Core *core = new Core(arguments,0);

    return a.exec();
}
