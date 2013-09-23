#include <QJsonDocument>
#include <QVariantMap>
#include <QVector3D>
#include <QDebug>
#include <QSettings>

#include "dataprocessor.h"

#define gravity 256

DataProcessor::DataProcessor(QObject *parent) :
    QObject(parent)
{
    loadCalibrationParameters();
    // calibdata to json
    connect(this,SIGNAL(calibDataReady(QVector3D,QVector3D,QVector3D)),this,SLOT(processDataToJson(QVector3D,QVector3D,QVector3D)));



}

bool DataProcessor::loadCalibrationParameters()
{
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

void DataProcessor::calculateCalibration(const QVector3D &accData, const QVector3D &gyroData, const QVector3D &magData)
{
    QVector3D accVector;
    QVector3D magVector;
    QVector3D gyrVector;

    // correct the acceleration vector depending
    accVector.setX((accData.x() - acc_x_offset) - acc_x_scale);
    accVector.setY((accData.y() - acc_y_offset) - acc_y_scale);
    accVector.setZ((accData.z() - acc_z_offset) - acc_z_scale);

    // correct the magnetometer vector depending
    magVector.setX((magData.x() - mag_x_offset) - mag_x_scale);
    magVector.setY((magData.y() - mag_y_offset) - mag_y_scale);
    magVector.setZ((magData.z() - mag_z_offset) - mag_z_scale);

    // compensate the gyroscop vector
    gyrVector.setX(gyroData.x() - gyr_x_offset);
    gyrVector.setY(gyroData.y() - gyr_y_offset);
    gyrVector.setZ(gyroData.z() - gyr_z_offset);

    emit calibDataReady(accVector.normalized(),gyrVector,magVector.normalized());
}

void DataProcessor::processDataToJson(const QVector3D &accData, const QVector3D &gyroData, const QVector3D &magData)
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

    message.insert("acc",accMap);
    message.insert("gyr",gyroMap);
    message.insert("mag",magMap);

    QByteArray data = QJsonDocument::fromVariant(message).toJson();
    //qDebug() << data;
    data.append("\n");

    emit dataTcpReady(data);
}
