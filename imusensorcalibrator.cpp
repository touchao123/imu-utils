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
#include <QVector3D>
#include <QSettings>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>

#include "imusensorcalibrator.h"

ImuSensorCalibrator::ImuSensorCalibrator(ImuSensor *sensor, QObject *parent) :
    m_sensor(sensor),QObject(parent)
{
    m_timer = new QTimer(this);
    m_input = new QTextStream(stdin);
    resetCalibration();
}

void ImuSensorCalibrator::calibrateAcc()
{
    qDebug() << "------------------------------------------------------------------------------------------";
    qDebug() << "                           ACCELEROMETER CALIBATION";
    qDebug() << "------------------------------------------------------------------------------------------";
    qDebug() << "Lets calibrate the accelerometer. To calibrate the accelerometer you need to move";
    qDebug() << "the sensor and try to get the highest and lowest value in each axis. ";
    qDebug() << "For example, move the sensor x-axis directing down (+x), because we want the maximum";
    qDebug() << "gravity vector in that direction. If the value doesnt geat bigger, do the same in the";
    qDebug() << "other direction (-x). If the value doesnt get smaller this is the maximal/minimal";
    qDebug() << "value of this axis.";
    qDebug() << "Repeat this procedure with y - and z axis.";
    qDebug() << "TIPP: move the sensor very slow on without any fast movements. try to maximize/minimize each axis.";
    qDebug() << "      if you make to fast movements it will create a bigger acceleration than gravity and you get a";
    qDebug() << "      wrong calibration";
    qDebug() << "";
    qDebug() << "please press ENTER to start the calibration....";

    QTextStream input(stdin);
    while(true){
        QString line = input.readLine();
        if(!line.isNull()){
            qDebug() << "------------------------------------------------------------------------------------------";
            qDebug() << " please press Ctrl + C when you are finished.";
            qDebug() << "------------------------------------------------------------------------------------------";
            break;
        }
    }


    while(true){
        measureAcc();
        usleep(50000);
    }
}

void ImuSensorCalibrator::calibrateGyr()
{
    qDebug() << "------------------------------------------------------------------------------------------";
    qDebug() << "                           GYROSCOPE CALIBATION";
    qDebug() << "------------------------------------------------------------------------------------------";
    qDebug() << "making 200 measurements...please wait ~ 10[s]";
    qDebug();
    qDebug() << "------------------------------------------------------------------------------------------";

    int i = 0;
    while(true){
        if(i >= 200){
            QSettings settings("imu-utils");
            settings.beginGroup("Gyr_calibration");
            settings.setValue("gyr_x_offset",QString::number(gyr_x_offset));
            settings.setValue("gyr_y_offset",QString::number(gyr_y_offset));
            settings.setValue("gyr_z_offset",QString::number(gyr_z_offset));
            settings.endGroup();
            qDebug();
            qDebug() << "------------------------------------------------------------------------------------------";
            qDebug() << "Saved values:";
            qDebug() << "   x = " << gyr_x_offset;
            qDebug() << "   y = " << gyr_y_offset;
            qDebug() << "   z = " << gyr_z_offset;
            break;
        }
        measureGyr();
        usleep(50000);
        i++;
    }
}

void ImuSensorCalibrator::calibrateMag()
{
    qDebug() << "------------------------------------------------------------------------------------------";
    qDebug() << "                           MAGNETOMETER CALIBATION";
    qDebug() << "------------------------------------------------------------------------------------------";
    qDebug();
    qDebug() << "please press ENTER to start the calibration....";


    QTextStream input(stdin);
    while(true){
        QString line = input.readLine();
        if(!line.isNull()){
            qDebug() << "------------------------------------------------------------------------------------------";
            qDebug() << " please press Ctrl + C when you are finished.";
            qDebug() << "------------------------------------------------------------------------------------------";
            break;
        }
    }

    while(true){
        measureMag();
        usleep(50000);
    }

}

void ImuSensorCalibrator::resetCalibration()
{
    acc_x_max = 0;
    acc_x_min = 0;
    acc_y_max = 0;
    acc_y_min = 0;
    acc_z_max = 0;
    acc_z_min = 0;

    gyr_x_offset = 0;
    gyr_y_offset = 0;
    gyr_z_offset = 0;

    mag_x_max = 0;
    mag_x_min = 0;
    mag_y_max = 0;
    mag_y_min = 0;
    mag_z_max = 0;
    mag_z_min = 0;



}

void ImuSensorCalibrator::measureAcc()
{
    QVector3D accData = m_sensor->readAcc();
    // check X min max
    if(accData.x() > acc_x_max){
        acc_x_max = (int)accData.x();
    }
    if(accData.x() < acc_x_min){
        acc_x_min = (int)accData.x();
    }

    // check Y min max
    if(accData.y() > acc_y_max){
        acc_y_max = (int)accData.y();
    }
    if(accData.y() < acc_y_min){
        acc_y_min = (int)accData.y();
    }

    // check Z min max
    if(accData.z() > acc_z_max){
        acc_z_max = (int)accData.z();
    }
    if(accData.z() < acc_z_min){
        acc_z_min = (int)accData.z();
    }
    QSettings settings("imu-utils");
    settings.beginGroup("Acc_calibration");
    settings.setValue("acc_x_max",acc_x_max);
    settings.setValue("acc_x_min",acc_x_min);
    settings.setValue("acc_y_max",acc_y_max);
    settings.setValue("acc_y_min",acc_y_min);
    settings.setValue("acc_z_max",acc_z_max);
    settings.setValue("acc_z_min",acc_z_min);
    settings.endGroup();

    printf("\r                                                                                  ");
    printf("\r    Acc X (min/max) = %i / %i",acc_x_min,acc_x_max);
    printf("\t\tAcc Y (min/max) = %i / %i",acc_y_min,acc_y_max);
    printf("\t\tAcc Z (min/max) = %i / %i",acc_z_min,acc_z_max);
    fflush(stdout);
}

void ImuSensorCalibrator::measureGyr()
{


    QVector3D gyrData = m_sensor->readGyr();
    gyr_x_list.append((int)gyrData.x());
    gyr_y_list.append((int)gyrData.y());
    gyr_z_list.append((int)gyrData.z());

    long x_average = 0;
    long y_average = 0;
    long z_average = 0;

    foreach (int value, gyr_x_list) {
        x_average += value;
    }
    gyr_x_offset = (float)x_average/gyr_x_list.length();

    foreach (int value, gyr_y_list) {
        y_average += value;
    }
    gyr_y_offset = (float)y_average/gyr_y_list.length();

    foreach (int value, gyr_z_list) {
        z_average += value;
    }
    gyr_z_offset = (float)z_average/gyr_z_list.length();

    printf("\r                                                                                                 ");
    printf("\r    Gyr X (current/average) = %i / %.3f",(int)gyrData.x(),(float)gyr_x_offset);
    printf("\t\tGyr Y (current/average) = %i / %.3f"  ,(int)gyrData.y(),(float)gyr_y_offset);
    printf("\t\tGyr Z (current/average) = %i / %.3f"  ,(int)gyrData.z(),(float)gyr_z_offset);
    fflush(stdout);
}

void ImuSensorCalibrator::measureMag()
{
    QVector3D magData = m_sensor->readMag();
    // check X min max
    if(magData.x() > mag_x_max){
        mag_x_max = (int)magData.x();
    }
    if(magData.x() < mag_x_min){
        mag_x_min = (int)magData.x();
    }

    // check Y min max
    if(magData.y() > mag_y_max){
        mag_y_max = (int)magData.y();
    }
    if(magData.y() < mag_y_min){
        mag_y_min = (int)magData.y();
    }

    // check Z min max
    if(magData.z() > mag_z_max){
        mag_z_max = (int)magData.z();
    }
    if(magData.z() < mag_z_min){
        mag_z_min = (int)magData.z();
    }
    QSettings settings("imu-utils");
    settings.beginGroup("Mag_calibration");
    settings.setValue("mag_x_max",mag_x_max);
    settings.setValue("mag_x_min",mag_x_min);
    settings.setValue("mag_y_max",mag_y_max);
    settings.setValue("mag_y_min",mag_y_min);
    settings.setValue("mag_z_max",mag_z_max);
    settings.setValue("mag_z_min",mag_z_min);
    settings.endGroup();

    printf("\r                                                                                  ");
    printf("\r    Mag X (min/max) = %i / %i",mag_x_min,mag_x_max);
    printf("\t\tMag Y (min/max) = %i / %i"  ,mag_y_min,mag_y_max);
    printf("\t\tMag Z (min/max) = %i / %i"  ,mag_z_min,mag_z_max);
    fflush(stdout);
}
