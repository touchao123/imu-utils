#ifndef CORE_H
#define CORE_H

#include <QObject>
#include <QStringList>

#include <imusensor.h>
#include <tcpserver.h>
#include <dataprocessor.h>
#include <imusensorcalibrator.h>

class Core : public QObject
{
    Q_OBJECT
public:
    explicit Core(QStringList args,QObject *parent = 0);

    ImuSensor *m_sensor;
    TcpServer *m_server;
    DataProcessor *m_dataProcessor;
    ImuSensorCalibrator *m_calibrator;

private:
    QStringList m_args;
    QString m_device;
    int m_delay;
    int m_port;

    void calibration();
    void setupSensor();
    void setupTcpServer();


signals:

public slots:
    void printData(const QVector3D &accData,const QVector3D &gyroData, const QVector3D &magData, const int &dt);
    void printAngles(const QVector3D &angles);
};

#endif // CORE_H
