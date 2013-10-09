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

#ifndef DCMFILTER_H
#define DCMFILTER_H

/*
 * Source from https://github.com/ptrbrtz/razor-9dof-ahrs/blob/master/Arduino/Razor_AHRS/
 */

#include <QObject>
#include <QVector3D>

// Gain for gyroscope (ITG-3200)
#define GYRO_GAIN 0.06957 // Same gain on all axes
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN)) // Calculate the scaled gyro readings in radians per second

// DCM parameters
#define Kp_ROLLPITCH 0.04f
#define Ki_ROLLPITCH 0.00002f
#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f

#define GRAVITY 256.0f // "1G reference" used for DCM filter and accelerometer calibration
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

class DcmFilter : public QObject
{
    Q_OBJECT
public:
    explicit DcmFilter(QObject *parent = 0);

private:
    void normalize();
    void driftCorrection();
    void matrixUpdate();
    void eulerAngles();
    void compassHeading();

    float vectorDotProduct(const float v1[3], const float v2[3]);
    void vectorCrossProduct(float out[3], const float v1[3], const float v2[3]);
    void vectorScale(float out[3], const float v[3], float scale);
    void vectorAdd(float out[3], const float v1[3], const float v2[3]);
    void matrixMultiply(const float a[3][3], const float b[3][3], float out[3][3]);
    void matrixVectorMultiply(const float a[3][3], const float b[3], float out[3]);
    void initRotationMatrix(float m[3][3], float yaw, float pitch, float roll);


    float accel[3];
    float magnetom[3];
    float gyro[3];

    // DCM variables
    float MAG_Heading;
    float Accel_Vector[3];  // Store the acceleration in a vector
    float Gyro_Vector[3];   // Store the gyros turn rate in a vector
    float Omega_Vector[3];  // Corrected Gyro_Vector data
    float Omega_P[3];       // Omega Proportional correction
    float Omega_I[3];       // Omega Integrator
    float Omega[3];
    float errorRollPitch[3];
    float errorYaw[3];
    float DCM_Matrix[3][3];
    float Update_Matrix[3][3];
    float Temporary_Matrix[3][3];
    float m_dt;

    // Euler angles
    float yaw;
    float pitch;
    float roll;


signals:

public slots:
    QVector3D updateData(const QVector3D &accData, const QVector3D &gyroData, const QVector3D &magData, const int &dt);

};

#endif // DCMFILTER_H
