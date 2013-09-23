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
