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

#ifndef IMUSENSOR_H
#define IMUSENSOR_H

#include <QObject>
#include <QString>
#include <QTimer>
#include <QVector3D>
#include<QRandomGenerator>

class ImuSensor : public QObject
{
    Q_OBJECT
private:
    QString m_deviceFile;
    QTimer *m_timer;
    QTime *time;

    QRandomGenerator *m_qrandom;
    int m_delay;
    float m_frequency;
    int m_device;
    // the I2C adresses of the 9 DOF IMU (result of bash command "i2cdetect -y 1")

    bool writeI2C(int address, uchar reg, uchar data);

    void initAcc();
    void initGyr();
    void initMag();

    int toSignedInt(unsigned int value, int bitLength);

public:
    explicit ImuSensor(QString deviceFile = "/dev/i2c-1", int delay = 20, QObject *parent = 0);
    void init();
    void detectDevices();
    QVector3D readAcc();
    QVector3D readGyr();
    QVector3D readMag();


public slots:
    void enableSensor();
    void disableSensor();
    void measure();

signals:
    void sensorDataAvailable(const QVector3D &accData,const QVector3D &gyroData, const QVector3D &magData, const int &dt);
};

#endif // IMUSENSOR_H
