#ifndef DATAPROCESSOR_H
#define DATAPROCESSOR_H

#include <QObject>
#include <QVector3D>
#include <dcmfilter.h>

class DataProcessor : public QObject
{
    Q_OBJECT
public:
    explicit DataProcessor(QObject *parent = 0);



private:
    // loads the saved calibration parameters from ~/.config/imu-utils.conf
    bool loadCalibrationParameters();
    void calibrateData(const QVector3D &accData, const QVector3D &gyroData, const QVector3D &magData, const int &dt);
    void complementaryFilter();
    float toDeg(float rad);
    float toRad(float deg);


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

    // calibrated data
    QVector3D m_acc;
    QVector3D m_gyr;
    QVector3D m_mag;
    int m_dt;
    
    // complementary filter
    float m_accXangle;
    float m_accYangle;

    float m_gyrXangle;
    float m_gyrYangle;

    // angles
    QVector3D m_angles;
    float m_roll;
    float m_pitch;
    float m_yaw;

    DcmFilter *m_dcmFilter;

signals:
    void dataTcpReady(const QByteArray &data);
    void calibratedDataReady(const QVector3D &accData,const QVector3D &gyroData, const QVector3D &magData, const int &dt);
    void anglesReady(const QVector3D & angles);

public slots:
    void processData(const QVector3D &accData,const QVector3D &gyroData, const QVector3D &magData, const int &dt);
    void serializeAllData(const QVector3D &accData,const QVector3D &gyroData, const QVector3D &magData, const QVector3D &angles, const int &dt);
};

#endif // DATAPROCESSOR_H
