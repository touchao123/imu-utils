#ifndef ROSNODE_H
#define ROSNODE_H

#include <QObject>
#include <QThread>
#include <QMutex>

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

class RosNode : public QThread
{
    Q_OBJECT
public:
    explicit RosNode(QObject *parent = 0, int argc = 0, char **argv = 0, const QString &name = 0);
    ~RosNode();
    void run();


private:
    int m_argc;
    char** m_argv;
    QString m_nodeName;
    QString m_topic;
    QString m_masterURI;
    QString m_hostIP;

    nav_msgs::Odometry m_odom;

    QMutex odomMutex;

signals:

public slots:
    void publishData(const QVector3D &angles,const QVector3D &anglesVel);

};

#endif // ROSNODE_H
