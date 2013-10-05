/*
 * Source from https://github.com/ptrbrtz/razor-9dof-ahrs/blob/master/Arduino/Razor_AHRS/Math.pde
 */


#include "dcmfilter.h"
#include <math.h>
#include <QDebug>

DcmFilter::DcmFilter(QObject *parent) :
    QObject(parent)
{
    initRotationMatrix(DCM_Matrix,yaw,pitch,roll);
}

void DcmFilter::normalize()
{
    float error=0;
    float temporary[3][3];
    float renorm=0;

    error= -vectorDotProduct(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19

    vectorScale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
    vectorScale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19

    vectorAdd(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
    vectorAdd(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19

    vectorCrossProduct(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20

    renorm= .5 *(3 - vectorDotProduct(&temporary[0][0],&temporary[0][0])); //eq.21
    vectorScale(&DCM_Matrix[0][0], &temporary[0][0], renorm);

    renorm= .5 *(3 - vectorDotProduct(&temporary[1][0],&temporary[1][0])); //eq.21
    vectorScale(&DCM_Matrix[1][0], &temporary[1][0], renorm);

    renorm= .5 *(3 - vectorDotProduct(&temporary[2][0],&temporary[2][0])); //eq.21
    vectorScale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}

void DcmFilter::driftCorrection()
{
    float mag_heading_x;
    float mag_heading_y;
    float errorCourse;
    //Compensation the Roll, Pitch and Yaw drift.
    static float Scaled_Omega_P[3];
    static float Scaled_Omega_I[3];
    float Accel_magnitude;
    float Accel_weight;


    //*****Roll and Pitch***************

    // Calculate the magnitude of the accelerometer vector
    Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
    Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
    // Dynamic weighting of accelerometer info (reliability filter)
    // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
    Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);  //

    vectorCrossProduct(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference
    vectorScale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight);

    vectorScale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight);
    vectorAdd(Omega_I,Omega_I,Scaled_Omega_I);

    //*****YAW***************
    // We make the gyro YAW drift correction based on compass magnetic heading

    mag_heading_x = cos(MAG_Heading);
    mag_heading_y = sin(MAG_Heading);
    errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
    vectorScale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

    vectorScale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);//.01proportional of YAW.
    vectorAdd(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.

    vectorScale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);//.00001Integrator
    vectorAdd(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
}

void DcmFilter::matrixUpdate()
{
    Gyro_Vector[0]=GYRO_SCALED_RAD(gyro[0]); //gyro x roll
    Gyro_Vector[1]=GYRO_SCALED_RAD(gyro[1]); //gyro y pitch
    Gyro_Vector[2]=GYRO_SCALED_RAD(gyro[2]); //gyro z yaw

    Accel_Vector[0]=accel[0];
    Accel_Vector[1]=accel[1];
    Accel_Vector[2]=accel[2];

    vectorAdd(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
    vectorAdd(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term

    // dont use drift correction
    //    Update_Matrix[0][0]=0;
    //    Update_Matrix[0][1]=-m_dt*Gyro_Vector[2];//-z
    //    Update_Matrix[0][2]=m_dt*Gyro_Vector[1];//y
    //    Update_Matrix[1][0]=m_dt*Gyro_Vector[2];//z
    //    Update_Matrix[1][1]=0;
    //    Update_Matrix[1][2]=-m_dt*Gyro_Vector[0];
    //    Update_Matrix[2][0]=-m_dt*Gyro_Vector[1];
    //    Update_Matrix[2][1]=m_dt*Gyro_Vector[0];
    //    Update_Matrix[2][2]=0;

    // Use drift correction
    Update_Matrix[0][0]=0;
    Update_Matrix[0][1]=-m_dt*Omega_Vector[2];//-z
    Update_Matrix[0][2]=m_dt*Omega_Vector[1];//y
    Update_Matrix[1][0]=m_dt*Omega_Vector[2];//z
    Update_Matrix[1][1]=0;
    Update_Matrix[1][2]=-m_dt*Omega_Vector[0];//-x
    Update_Matrix[2][0]=-m_dt*Omega_Vector[1];//-y
    Update_Matrix[2][1]=m_dt*Omega_Vector[0];//x
    Update_Matrix[2][2]=0;

    matrixMultiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c

    for(int x=0; x<3; x++) //Matrix Addition (update)
    {
        for(int y=0; y<3; y++)
        {
            DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
        }
    }
}

void DcmFilter::eulerAngles()
{
    pitch = -asin(DCM_Matrix[2][0]);
    roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
    yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
}

void DcmFilter::compassHeading()
{
    float mag_x;
    float mag_y;
    float cos_roll;
    float sin_roll;
    float cos_pitch;
    float sin_pitch;

    cos_roll = cos(roll);
    sin_roll = sin(roll);
    cos_pitch = cos(pitch);
    sin_pitch = sin(pitch);

    // Tilt compensated magnetic field X
    mag_x = magnetom[0] * cos_pitch + magnetom[1] * sin_roll * sin_pitch + magnetom[2] * cos_roll * sin_pitch;
    // Tilt compensated magnetic field Y
    mag_y = magnetom[1] * cos_roll - magnetom[2] * sin_roll;
    // Magnetic Heading
    MAG_Heading = atan2(-mag_y, mag_x);
    //qDebug() << MAG_Heading* 180 / M_PI;
}

float DcmFilter::vectorDotProduct(const float v1[], const float v2[])
{
    float result = 0;

    for(int c = 0; c < 3; c++)
    {
        result += v1[c] * v2[c];
    }

    return result;
}

void DcmFilter::vectorCrossProduct(float out[], const float v1[], const float v2[])
{
    out[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
    out[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
    out[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
}

void DcmFilter::vectorScale(float out[], const float v[], float scale)
{
    for(int c = 0; c < 3; c++)
    {
        out[c] = v[c] * scale;
    }
}

void DcmFilter::vectorAdd(float out[], const float v1[], const float v2[])
{
    for(int c = 0; c < 3; c++)
    {
        out[c] = v1[c] + v2[c];
    }
}

void DcmFilter::matrixMultiply(const float a[3][3], const float b[3][3], float out[3][3])
{
    for(int x = 0; x < 3; x++)  // rows
    {
        for(int y = 0; y < 3; y++)  // columns
        {
            out[x][y] = a[x][0] * b[0][y] + a[x][1] * b[1][y] + a[x][2] * b[2][y];
        }
    }
}



void DcmFilter::matrixVectorMultiply(const float a[3][3], const float b[3], float out[3])
{
    for(int x = 0; x < 3; x++)
    {
        out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2];
    }
}

void DcmFilter::initRotationMatrix(float m[3][3], float yaw, float pitch, float roll)
{
    float c1 = cos(roll);
    float s1 = sin(roll);
    float c2 = cos(pitch);
    float s2 = sin(pitch);
    float c3 = cos(yaw);
    float s3 = sin(yaw);

    // Euler angles, right-handed, intrinsic, XYZ convention
    // (which means: rotate around body axes Z, Y', X'')
    m[0][0] = c2 * c3;
    m[0][1] = c3 * s1 * s2 - c1 * s3;
    m[0][2] = s1 * s3 + c1 * c3 * s2;

    m[1][0] = c2 * s3;
    m[1][1] = c1 * c3 + s1 * s2 * s3;
    m[1][2] = c1 * s2 * s3 - c3 * s1;

    m[2][0] = -s2;
    m[2][1] = c2 * s1;
    m[2][2] = c1 * c2;
}

QVector3D DcmFilter::updateData(const QVector3D &accData, const QVector3D &gyroData, const QVector3D &magData, const int &dt)
{
    accel[0] = accData.x();
    accel[1] = accData.y();
    accel[2] = accData.z();

    gyro[0] = gyroData.x();
    gyro[1] = gyroData.y();
    gyro[2] = gyroData.z();

    magnetom[0] = magData.x();
    magnetom[1] = magData.y();
    magnetom[2] = magData.z();

    m_dt = (float)dt/1000;

    // Run DCM algorithm
    compassHeading();
    matrixUpdate();
    normalize();
    driftCorrection();
    eulerAngles();

    //qDebug() << roll << pitch << yaw;

    return QVector3D(roll,pitch,yaw);
}

