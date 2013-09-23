#ifndef IMUSENSOR_H
#define IMUSENSOR_H

#include <QObject>
#include <QString>
#include <QTimer>
#include <QVector3D>

class ImuSensor : public QObject
{
    Q_OBJECT
public:
    explicit ImuSensor(QString deviceFile = "/dev/i2c-1", int delay = 20, QObject *parent = 0);
    void init();
    void detectDevices();
    QVector3D readAcc();
    QVector3D readGyr();
    QVector3D readMag();

private:
    QString m_deviceFile;
    QTimer *m_timer;
    int m_delay;
    float m_frequency;
    int m_device;
    // the I2C adresses of the 9 DOF IMU (result of bash command "i2cdetect -y 1")

    bool writeI2C(int address, uchar reg, uchar data);

    void initAcc();
    void initGyr();
    void initMag();

    int toSignedInt(unsigned int value, int bitLength);

public slots:
    void enableSensor();
    void disableSensor();
    void measure();

signals:
    void sensorDataAvailable(const QVector3D &accData,const QVector3D &gyroData, const QVector3D &magData);
};

#endif // IMUSENSOR_H
