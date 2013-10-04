#include <QJsonDocument>
#include <QVariantMap>
#include <QVector3D>
#include <QMatrix>
#include <QMatrix3x3>
#include <QDebug>
#include <QSettings>
#include <math.h>

#include "dataprocessor.h"

#define gravity 256

DataProcessor::DataProcessor(QObject *parent) :
    QObject(parent)
{
    m_gyrXangle = 0;
    m_gyrYangle = 0;

    loadCalibrationParameters();
}

// this method gets the rawdata from the sensor, compensates sensorerrors, filter the data and calculates the angles...
void DataProcessor::processData(const QVector3D &accData, const QVector3D &gyroData, const QVector3D &magData, const int &dt)
{
    // ==============================================================================
    // first need to compensate the sensorerrors by using the calibrationdata...
    calibrateData(accData,gyroData,magData,dt);
    emit calibratedDataReady(m_acc,m_gyr,m_mag,dt);

    // ==============================================================================
    // now we can calculate the roll pitch angles...
    complementaryFilter();
    emit anglesReady(m_angles* 180/M_PI);

    // send all data in JSON format to TCp Server
    serializeAllData(m_acc,m_gyr,m_mag,m_angles* 180/M_PI,m_dt);

}


bool DataProcessor::loadCalibrationParameters()
{
    // TODO: check if data is ok  .toInt(&ok); when not return false and exit(1)

    // load all calibration parameters...
    QSettings settings("imu-utils");
    settings.beginGroup("Acc_calibration");
    int acc_x_max = settings.value("acc_x_max",999).toInt();
    int acc_x_min = settings.value("acc_x_min",999).toInt();
    int acc_y_max = settings.value("acc_y_max",999).toInt();
    int acc_y_min = settings.value("acc_y_min",999).toInt();
    int acc_z_max = settings.value("acc_z_max",999).toInt();
    int acc_z_min = settings.value("acc_z_min",999).toInt();
    settings.endGroup();

    settings.beginGroup("Gyr_calibration");
    gyr_x_offset = settings.value("gyr_x_offset",999).toFloat();
    gyr_y_offset = settings.value("gyr_y_offset",999).toFloat();
    gyr_z_offset = settings.value("gyr_z_offset",999).toFloat();
    settings.endGroup();

    settings.beginGroup("Mag_calibration");
    int mag_x_max = settings.value("mag_x_max",999).toInt();
    int mag_x_min = settings.value("mag_x_min",999).toInt();
    int mag_y_max = settings.value("mag_y_max",999).toInt();
    int mag_y_min = settings.value("mag_y_min",999).toInt();
    int mag_z_max = settings.value("mag_z_max",999).toInt();
    int mag_z_min = settings.value("mag_z_min",999).toInt();
    settings.endGroup();
    qDebug() << "--------------------------------------------";
    qDebug() << "-> calibration data loaded from" << settings.fileName();
    qDebug() << "   Acc  (min/max):     X  =" << acc_x_min << "/" << acc_x_max << "\tY =" << acc_y_min << "/" << acc_y_max << "\tZ =" << acc_z_min << "/" << acc_z_max ;
    qDebug() << "   Mag  (min/max):     X  =" << mag_x_min << "/" << mag_x_max << "\tY =" << mag_y_min << "/" << mag_y_max << "\tZ =" << mag_z_min << "/" << mag_z_max ;
    qDebug() << "   Gyro (offset) :     X  =" << gyr_x_offset << "\tY =" << gyr_y_offset << "\tZ =" << gyr_z_offset;

    // calculate acc offsets and scale factor
    acc_x_offset = (float)(acc_x_min + acc_x_max)/2;
    acc_y_offset = (float)(acc_y_min + acc_y_max)/2;
    acc_z_offset = (float)(acc_z_min + acc_z_max)/2;
    acc_x_scale  = gravity / (acc_x_max-acc_x_offset);
    acc_y_scale  = gravity / (acc_y_max-acc_y_offset);
    acc_z_scale  = gravity / (acc_z_max-acc_z_offset);

    // calculate mag offsets and scale factor
    mag_x_offset = (float)(mag_x_min + mag_x_max)/2;
    mag_y_offset = (float)(mag_y_min + mag_y_max)/2;
    mag_z_offset = (float)(mag_z_min + mag_z_max)/2;
    mag_x_scale  = (float)100 / (mag_x_max-mag_x_offset);
    mag_y_scale  = (float)100 / (mag_y_max-mag_y_offset);
    mag_z_scale  = (float)100 / (mag_z_max-mag_z_offset);

    // TODO: evaluate the calibration: is it good or bad...needs a new calibration?
    return true;
}

void DataProcessor::calibrateData(const QVector3D &accData, const QVector3D &gyroData, const QVector3D &magData, const int &dt)
{
    // compensating  the data using the values from
    QVector3D accVector;
    QVector3D magVector;
    QVector3D gyrVector;
    // compensate the acceleration vector (so the vector has the same length in both directions)
    accVector.setX((accData.x() - acc_x_offset) - acc_x_scale);
    accVector.setY((accData.y() - acc_y_offset) - acc_y_scale);
    accVector.setZ((accData.z() - acc_z_offset) - acc_z_scale);

    // compensate the magnetometer vector (so the vector has the same length in both directions)
    magVector.setX((magData.x() - mag_x_offset) - mag_x_scale);
    magVector.setY((magData.y() - mag_y_offset) - mag_y_scale);
    magVector.setZ((magData.z() - mag_z_offset) - mag_z_scale);

    // compensate the gyroscop vector (bring it to 0 when not moving)
    gyrVector.setX(gyroData.x() - gyr_x_offset);
    gyrVector.setY(gyroData.y() - gyr_y_offset);
    gyrVector.setZ(gyroData.z() - gyr_z_offset);

    // save normalized acc - mag vector, and the zeroed gyr vector and the dt fpr this data
    m_acc = accVector.normalized();
    m_gyr = gyrVector;
    m_mag = magVector.normalized();
    m_dt = dt;

}

void DataProcessor::complementaryFilter()
{
    float gyrosensitivity = 1;
    float alpha = 0.2;


    // angle from acc
    m_accXangle = (atan2(m_acc.y(),m_acc.z()) + M_PI);
    m_accYangle = (atan2(m_acc.x(),m_acc.z()) + M_PI);

    //angle from gyr
    m_gyrXangle = ((m_gyr.x()/gyrosensitivity) * ((float)m_dt / 1000));
    m_gyrYangle = ((m_gyr.y()/gyrosensitivity) * ((float)m_dt / 1000));

    //qDebug() << m_gyrXangle << m_gyrYangle;

    // Low pass filtering
    // x...roll
    //m_roll = m_accXangle;
    m_roll = (alpha * (m_roll + m_gyrXangle)) + ((1-alpha)*m_accXangle);
    // y...pitch
    //m_pitch = m_accYangle;
    m_pitch = (alpha * (m_pitch + m_gyrYangle)) + ((1-alpha)*m_accYangle);

    // z...yaw -> magnetic heading ...
    float mag_x;
    float mag_y;
    float cos_roll = cos(m_roll);
    float sin_roll = sin(m_roll);
    float cos_pitch = cos(m_pitch);
    float sin_pitch = sin(m_pitch);

    mag_x = (m_mag.x() * cos_pitch) + (m_mag.y() * sin_roll * sin_pitch) + (m_mag.z() * cos_roll * sin_pitch);
    mag_y = m_mag.y() * cos_roll - m_mag.z() * sin_roll;

    m_yaw = atan2(mag_y,mag_x);

    m_angles.setX(m_roll);
    m_angles.setY(m_pitch);
    m_angles.setZ(m_yaw);

}

float DataProcessor::toDeg(float rad)
{
    return rad * 180 / M_PI;
}

float DataProcessor::toRad(float deg)
{
    return deg * M_PI /180;
}

void DataProcessor::serializeAllData(const QVector3D &accData, const QVector3D &gyroData, const QVector3D &magData, const QVector3D &angles, const int &dt)
{
    QVariantMap message;

    QVariantMap accMap;
    accMap.insert("x",accData.x());
    accMap.insert("y",accData.y());
    accMap.insert("z",accData.z());

    QVariantMap gyroMap;
    gyroMap.insert("x",gyroData.x());
    gyroMap.insert("y",gyroData.y());
    gyroMap.insert("z",gyroData.z());

    QVariantMap magMap;
    magMap.insert("x",magData.x());
    magMap.insert("y",magData.y());
    magMap.insert("z",magData.z());

    QVariantMap angleMap;
    angleMap.insert("roll",angles.x());
    angleMap.insert("pitch",angles.y());
    angleMap.insert("yaw",angles.z());

    message.insert("acc",accMap);
    message.insert("gyr",gyroMap);
    message.insert("mag",magMap);
    message.insert("angles",angleMap);
    message.insert("dt",dt);

    QByteArray data = QJsonDocument::fromVariant(message).toJson();
    data.append("\n");

    emit dataTcpReady(data);
}
