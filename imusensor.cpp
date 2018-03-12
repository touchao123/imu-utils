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
#include <QtGui/QVector3D>
#include <QString>
#include <QTime>
#include <stdlib.h>
#include <unistd.h>
//#include <linux/i2c-dev.h>
//#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>

#include "imusensor.h"

// Accelerometer Register's [ADXL345]
#define ACC_ADDRESS             0x53    // Address on I2C port
#define ACC_DEVICE_ID           0x00    // Power Control Register
#define ACC_BW_RATE             0x2C    // Data rate and power mode control.
#define ACC_POWER_CTL           0x2D    // Power Control Register
#define ACC_DATA_FORMAT         0x31
#define ACC_DATAX0              0x32    // X-Axis Data 0
#define ACC_DATAX1              0x33    // X-Axis Data 1
#define ACC_DATAY0              0x34    // Y-Axis Data 0
#define ACC_DATAY1              0x35    // Y-Axis Data 1
#define ACC_DATAZ0              0x36    // Z-Axis Data 0
#define ACC_DATAZ1              0x37    // Z-Axis Data 1

// Gyroscope Register's [ITG3200]
#define GYRO_ADDRESS            0x68    // R    Address on I2C port
#define GYRO_WHO_AM_I           0x00    // RW   SETUP: I2C address
#define GYRO_SMPLRT_DIV         0x15    // RW   SETUP: Sample Rate Divider
#define GYRO_DLPF_FS            0x16    // RW   SETUP: Digital Low Pass Filter/ Full Scale range
#define GYRO_INT_CFG            0x17    // RW   Interrupt: Configuration
#define GYRO_INT_STATUS         0x1A    // R    Interrupt: Status
#define GYRO_DATATEMP_H         0x1B    // R    SENSOR: Temperature 2bytes
#define GYRO_DATATEMP_L         0x1C    // R
#define GYRO_DATAX_H            0x1D    // R    SENSOR: Gyro X 2bytes
#define GYRO_DATAX_L            0x1E    // R
#define GYRO_DATAY_H            0x1F    // R    SENSOR: Gyro y 2bytes
#define GYRO_DATAY_L            0x20    // R
#define GYRO_DATAZ_H            0x21    // R    SENSOR: Gyro Z 2bytes
#define GYRO_DATAZ_L            0x22    // R
#define GYRO_PWR_MGM            0x3E    // RW   Power Management

// Magnetometer Register's [HMC5883L]
#define MAG_ADDRESS             0x1e    // R
#define MAG_CONFIG_REG_A        0x00    // RW
#define MAG_CONFIG_REG_B        0x01    // RW
#define MAG_MODE_REG            0x02    // RW
#define MAG_DATA_OUT_X_MSB_REG  0x03    // R
#define MAG_DATA_OUT_X_LSB_REG  0x04    // R
#define MAG_DATA_OUT_Z_MSB_REG  0x05    // R
#define MAG_DATA_OUT_Z_LSB_REG  0x06    // R
#define MAG_DATA_OUT_Y_MSB_REG  0x07    // R
#define MAG_DATA_OUT_Y_LSB_REG  0x08    // R
#define MAG_STATUS_REG          0x09    // R
#define MAG_ID_REG_A            0x10    // R
#define MAG_ID_REG_B            0x11    // R
#define MAG_ID_REG_C            0x12    // R


ImuSensor::ImuSensor(QString deviceFile, int delay, QObject *parent) :
    m_deviceFile(deviceFile),m_delay(delay),QObject(parent)
{
    m_device = -1;
    m_frequency = (float)1000/m_delay;

    time = new QTime();
    m_timer = new QTimer(this);
    m_timer->setInterval(m_delay);
    init();

     m_qrandom =  new QRandomGenerator();

    connect(m_timer,SIGNAL(timeout()),this,SLOT(measure()));
}

void ImuSensor::init()
{
    using namespace std;
#if 0
    m_device = open(m_deviceFile.toLocal8Bit(),O_RDWR);
    if(m_device < 0){
        qDebug() << "ERROR: could not open" << m_deviceFile;
        qDebug() << "please check if the i2c modules are loaded (on Raspberry Pi: i2c_bcm2708, i2c-dev).";
        exit(1);
    }

    //qDebug() << m_deviceFile << "open.";

    // scanning for I2C-Sensors and safe the addresses of acc, gyr, mag
    detectDevices();
#endif
    initAcc();
    initGyr();
    initMag();

}

void ImuSensor::detectDevices()
{
    qDebug() << "--------------------------------------------";
    qDebug() << "scanning for I2C devices on" << m_deviceFile << "...";
    qDebug() << "--------------------------------------------";
    QList<int> devices;
#if 0
    for(int address = 0x3; address <= 0x77; address++){
        if(ioctl(m_device, I2C_SLAVE, address) < 0) {
            qDebug() << "ERROR: could not open address" << address;
        }

        i2c_smbus_write_byte(m_device,0x00);
        int id = i2c_smbus_read_byte(m_device);
        QString output;
        if(id > 0){
            output.append("  device   = ");
            switch (address) {
            case ACC_ADDRESS:
                output.append("Accelerometer\n");
                devices.append(address);
                break;
            case GYRO_ADDRESS:
                output.append("Gyroscope\n");
                devices.append(address);
                break;
            case MAG_ADDRESS:
                output.append("Magnetometer\n");
                devices.append(address);
                break;
            default:
                break;
            }

            output.append(QString("   address  = 0x%1\n").arg(address,0,16));
            output.append(QString("   id       = 0b%1").arg(address,0,2));
            output.append(QString(" = %1").arg(address,0,10));
            qDebug() << output;
            qDebug() << "-----------------------------";
        }
    }
#endif
    if(devices.isEmpty()){
        qDebug() << "ERROR: no imu-sensor devices found on " << m_deviceFile;
        exit(1);
    }
    qDebug() << "--------------------------------------------";
}

QVector3D ImuSensor::readAcc()
{
    uchar bytes[6];
    memset(bytes,0,sizeof(bytes));
    int acc_data[3];

    QVector3D vector;

#if 0
    // set I/O controll to acc address
    ioctl(m_device, I2C_SLAVE, ACC_ADDRESS);

    uint length = i2c_smbus_read_i2c_block_data(m_device,ACC_DATAX0,6,bytes);
    if(length != sizeof(bytes)){
        qDebug() << "ERROR: could not read Acc data";
        return vector;
    }
    // register bytes -> [X0,X1,Y0,Y1,Z0,Z1]
    acc_data[0] = (((int) bytes[3]) << 8) | bytes[2];  // x = y (sensor)
    acc_data[1] = (((int) bytes[1]) << 8) | bytes[0];  // y = x (sensor)
    acc_data[2] = (((int) bytes[5]) << 8) | bytes[4];  // z = z (sensor)

    vector.setX(float(toSignedInt(acc_data[0],16)));
    vector.setY(float(toSignedInt(acc_data[1],16)));
    vector.setZ(float(toSignedInt(acc_data[2],16)));
#endif
    vector.setX(m_qrandom->bounded(980));
    vector.setY(m_qrandom->bounded(990));
    vector.setZ(m_qrandom->bounded(988));
    //qDebug() << "Acc data: x= " << vector.x() << "y= " << vector.y() << "z= " <<  vector.z();
    return vector;
}

QVector3D ImuSensor::readGyr()
{
    uchar bytes[6];
    memset(bytes,0,sizeof(bytes));
    int gyro_data[3];
//    QRandomGenerator qrandom;

    QVector3D vector;

#if 0
    // set I/O controll to acc address
    ioctl(m_device, I2C_SLAVE, GYRO_ADDRESS);

    uint length = i2c_smbus_read_i2c_block_data(m_device,GYRO_DATAX_H,6,bytes);
    if(length != sizeof(bytes)){
        qDebug() << "ERROR: could not read Gyr data";
        return vector;
    }
    // register bytes -> [X1,X0,Y1,Y0,Z1,Z0]
    gyro_data[0] = ((((int) bytes[2]) << 8) | bytes[3]); // x = -y (sensor)
    gyro_data[1] = ((((int) bytes[0]) << 8) | bytes[1]); // y = -x (sensor)
    gyro_data[2] = ((((int) bytes[4]) << 8) | bytes[5]); // z = -z (sensor)

    vector.setX(float(toSignedInt(gyro_data[0],16))*(-1));
    vector.setY(float(toSignedInt(gyro_data[1],16))*(-1));
    vector.setZ(float(toSignedInt(gyro_data[2],16))*(-1));
#endif


    vector.setX(m_qrandom->bounded(100.0));
    vector.setY(m_qrandom->bounded(200.0));
    vector.setZ(m_qrandom->bounded(300.0));

//    qDebug() << "Gyro data: x= " << gyro_data[0] << "y= " << gyro_data[1] << "z= " <<  gyro_data[2];
//    qDebug() << "Gyro data: x= " << vector.x() << "y= " <<vector.y() << "z= " <<  vector.z();

    return vector;
}

QVector3D ImuSensor::readMag()
{
    uchar bytes[6];
    memset(bytes,0,sizeof(bytes));
    int mag_data[3];

    QVector3D vector;

    // set I/O controll to acc address
#if 0
    ioctl(m_device, I2C_SLAVE, MAG_ADDRESS);

    uint length = i2c_smbus_read_i2c_block_data(m_device,MAG_DATA_OUT_X_MSB_REG,6,bytes);
    if(length != sizeof(bytes)){
        qDebug() << "ERROR: could not read Gyr data";
        return vector;
    }
    // register bytes -> [X1,X0,Z1,Z0,Y1,Y0]
    // Attention -> register bytes are: xxzzyy -> mag_data[x,z,y]
    mag_data[0] = (((int) bytes[0]) << 8) | bytes[1];
    mag_data[1] = (((int) bytes[4]) << 8) | bytes[5];
    mag_data[2] = (((int) bytes[2]) << 8) | bytes[3];

    vector.setX(float(toSignedInt(mag_data[0],16)));
    vector.setY(float(toSignedInt(mag_data[1],16))*(-1));
    vector.setZ(float(toSignedInt(mag_data[2],16))*(-1));
#endif
    vector.setX(m_qrandom->bounded(100.0));
    vector.setY(m_qrandom->bounded(200.0));
    vector.setZ(m_qrandom->bounded(300.0));
    return vector;
}


bool ImuSensor::writeI2C(int address, uchar reg, uchar data)
{
#if 0
    if(ioctl(m_device, I2C_SLAVE, address)<0){
        qDebug() << "ERROR: could not open address" << address;
        return false;
    }
    int ok = i2c_smbus_write_byte_data(m_device, reg, data);

    if(ok < 0){
        qDebug() << "ERROR: could not write to I2C";
        return false;
    }
#endif
    return true;
}

void ImuSensor::initAcc()
{
    //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
    if(!writeI2C(ACC_ADDRESS,ACC_POWER_CTL,0x08)){
        qDebug() << "ERROR: could not put ACC into measurement mode";
        return;
    }
    usleep(50000);
    
    //Put the ADXL345 into full resolution 0b1000
    if(!writeI2C(ACC_ADDRESS,ACC_DATA_FORMAT,0x08)){
        qDebug() << "ERROR: could not set full resolution of ACC";
        return;
    }
    usleep(50000);
    
    /* set output data rate frequency
     *  400  Hz  ->   1100    = 0x0C
     *  200  Hz  ->   1011    = 0x0B
     *  100  Hz  ->   1010    = 0x0A
     *  50   Hz  ->   1001    = 0x09
     *  25   Hz  ->   1000    = 0x08
     *  12.5 Hz  ->   0111    = 0x07
     */

    //Set the ADXL345 datarate to 100 Hz -> 0x0A = default
    if(!writeI2C(ACC_ADDRESS,ACC_BW_RATE,0x0A)){
        qDebug() << "ERROR: could not set ACC to 100 Hz";
        return;
    }
    usleep(50000);

    qDebug() << "Acc initialized.  (100Hz output rate)";
}

void ImuSensor::initGyr()
{
    // Power up reset defaults
    if(!writeI2C(GYRO_ADDRESS,GYRO_PWR_MGM,0x80)){
        qDebug() << "ERROR: could not power up reset defaults of GYRO";
        return;
    }
    usleep(50000);

    // Select full-scale range of the gyro sensors
    // set DLPF and Fullscale -> FS_SEL = 3, DLPF_CFG = 3 = 42 Hz = 11011 = 0x1B
    // set DLPF and Fullscale -> FS_SEL = 3, DLPF_CFG = 4 = 20 Hz = 11100 = 0x1C
    if(!writeI2C(GYRO_ADDRESS,GYRO_DLPF_FS,0x1B)){
        qDebug() << "ERROR: could not set DLPF-FS of GYRO";
        return;
    }
    usleep(50000);

    // Set sample rate
    // rate = 1kHz (depending in FS_SEL) / (divider+1)
    // rate = 1kHz/(9+1) = 100Hz    (devicer can be 0-255)
    //    if(!writeI2C(GYRO_ADDRESS,GYRO_SMPLRT_DIV,0x09)){
    //        qDebug() << "ERROR: could not set rate devider of GYRO";
    //        return;
    //    }
    //    usleep(50000);

    // Set clock to PLL with z gyro reference
    if(!writeI2C(GYRO_ADDRESS,GYRO_PWR_MGM,0x00)){
        qDebug() << "ERROR: could not set to PLL with z-reference of GYRO";
        return;
    }
    usleep(50000);

    qDebug() << "Gyro initialized. (100Hz output rate)";
}

void ImuSensor::initMag()
{
    /* put mag to continuous measurment mode to mode register
     *  0x00 = continuous measurement
     *  0x01 = single measurement (default)
     *  0x02/ 0x03 = Idle mode
     */
    if(!writeI2C(MAG_ADDRESS,MAG_MODE_REG,0x00)){
        qDebug() << "ERROR: could not initialize MAG";
        return;
    }
    usleep(50000);

    // set sample rate to 75Hz (maximum)
    if(!writeI2C(MAG_ADDRESS,0x00,0b00011000)){
        qDebug() << "ERROR: could not initialize MAG";
        return;
    }
    usleep(50000);

    qDebug() << "Mag initialized.  (75Hz output rate)";

}


int ImuSensor::toSignedInt(unsigned int value, int bitLength)
{
    int signedValue = value;
    if (value >> (bitLength - 1))
        signedValue |= -1 << bitLength;
    return signedValue;
}

void ImuSensor::enableSensor()
{
    m_timer->start();
    time->start();
    qDebug() << "--------------------------------------------";
    qDebug() << "measurement frequency:" << m_frequency << "[Hz]" << "  -> (" << m_delay << " ms)";
    qDebug() << "Start measuring....";
}

void ImuSensor::disableSensor()
{
    m_timer->stop();
    qDebug() << "Stop measuring....";

}

void ImuSensor::measure()
{
    QVector3D acc = readAcc();
    QVector3D gyr = readGyr();
    QVector3D mag = readMag();
//qDebug()<<"faking data";
    // measeure time since last restart of time;
    int dt = time->elapsed();
    time->restart();
    emit sensorDataAvailable(acc,gyr,mag,dt);

}



