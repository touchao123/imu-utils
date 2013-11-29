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

#ifndef CORE_H
#define CORE_H

#include <QObject>
#include <QStringList>

#include <imusensor.h>
#include <tcpserver.h>
#include <dataprocessor.h>
#include <imusensorcalibrator.h>
#include <rosodometrylistener.h>

class Core : public QObject
{
    Q_OBJECT
public:
    explicit Core(QStringList args, int argc, char **argv, QObject *parent = 0);

private:
    QStringList m_args;
    int m_argc;
    char** m_argv;
    QString m_device;
    int m_delay;
    int m_port;

    ImuSensor *m_sensor;
    TcpServer *m_server;
    DataProcessor *m_dataProcessor;
    ImuSensorCalibrator *m_calibrator;
    RosOdometryListener *m_rosListener;

    void calibration();
    void setupSensor();
    void setupTcpServer();


signals:

public slots:
    void printData(const QVector3D &accData,const QVector3D &gyroData, const QVector3D &magData, const int &dt);
    void printAngles(const QVector3D &angles);
};

#endif // CORE_H
