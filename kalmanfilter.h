#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <QObject>
#include <QVector2D>
#include <QMatrix2x2>


class KalmanFilter : public QObject
{
    Q_OBJECT
public:
    explicit KalmanFilter(QObject *parent = 0);

    float getAngle(float newAngle, float newRate, float dt);
    void setAngle(float newAngle);
    float getRate();
    void setQangle(double newQ_angle);
    void setQbias(double newQ_bias);
    void setRmeasure(double newR_measure);


private:
    /* Kalman filter variables */
    float Q_angle; // Process noise variance for the accelerometer
    float Q_bias; // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float angle; // The angle calculated by the Kalman filter - part of the 2x1 state matrix
    float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state matrix
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
    float K[2]; // Kalman gain - This is a 2x1 matrix
    float y; // Angle difference - 1x1 matrix
    float S; // Estimate error - 1x1 matrix

signals:

public slots:

};

#endif // KALMANFILTER_H
