#ifndef DATAPROCESSOR_H
#define DATAPROCESSOR_H

#include <QObject>
#include <QVector3D>

class DataProcessor : public QObject
{
    Q_OBJECT
public:
    explicit DataProcessor(QObject *parent = 0);

private:
    // Sensor calibration scale and offset values
    float acc_x_offset;
    float acc_y_offset;
    float acc_z_offset;
    float acc_x_scale;
    float acc_y_scale;
    float acc_z_scale;

    float mag_x_offset;
    float mag_y_offset;
    float mag_z_offset;
    float mag_x_scale;
    float mag_y_scale;
    float mag_z_scale;

    float gyr_x_offset;
    float gyr_y_offset;
    float gyr_z_offset;

    bool loadCalibrationParameters();

signals:
    void dataTcpReady(const QByteArray &data);
    void calibDataReady(const QVector3D &accData,const QVector3D &gyroData, const QVector3D &magData);

public slots:
    void calculateCalibration(const QVector3D &accData, const QVector3D &gyroData, const QVector3D &magData);
    void processDataToJson(const QVector3D &accData,const QVector3D &gyroData, const QVector3D &magData);

};

#endif // DATAPROCESSOR_H
