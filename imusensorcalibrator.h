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

#ifndef IMUSENSORCALIBRATOR_H
#define IMUSENSORCALIBRATOR_H

#include <QObject>
#include <QTimer>
#include <QTextStream>

#include <imusensor.h>


class ImuSensorCalibrator : public QObject
{
    Q_OBJECT
public:
    explicit ImuSensorCalibrator(ImuSensor *sensor = 0, QObject *parent = 0);

    void calibrateAcc();
    void calibrateGyr();
    void calibrateMag();

private:
    QTimer *m_timer;
    ImuSensor *m_sensor;
    QTextStream *m_input;

    int acc_x_max;
    int acc_x_min;
    int acc_y_max;
    int acc_y_min;
    int acc_z_max;
    int acc_z_min;

    int mag_x_max;
    int mag_x_min;
    int mag_y_max;
    int mag_y_min;
    int mag_z_max;
    int mag_z_min;

    float gyr_x_offset;
    float gyr_y_offset;
    float gyr_z_offset;

    QList<int> gyr_x_list;
    QList<int> gyr_y_list;
    QList<int> gyr_z_list;

    void resetCalibration();

signals:

private slots:
    void measureAcc();
    void measureGyr();
    void measureMag();


public slots:

};

#endif // IMUSENSORCALIBRATOR_H
