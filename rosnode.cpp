#include "rosnode.h"
#include <QDebug>
#include <QVector3D>

RosNode::RosNode(QObject *parent, int argc, char **argv, const QString &name) :
    QThread(parent),m_argc(argc), m_argv(argv), m_nodeName(name)
{
}

RosNode::~RosNode()
{
//    ros::shutdown();
}

void RosNode::run()
{
#if 0
    ros::init(m_argc,m_argv,m_nodeName.toStdString());
//    if (!ros::master::check()){
//        qDebug() << "--------------------------------------------";
//        qDebug() << "ROS -> ERROR: could not init ROS.";
//        return;
//    }
    qDebug() << "--------------------------------------------";
    qDebug() << "ROS -> init OK.";

    qDebug() << "--------------------------------------------";
    qDebug() << "ROS -> Node " << m_nodeName << "started.";
    qDebug() << "--------------------------------------------";

    ros::NodeHandle node;
    ros::Publisher imuPublisher = node.advertise<nav_msgs::Odometry>("imu_odometry", 1000);

    ros::Rate r(50.0);

    while(ros::ok()){


        odomMutex.lock();
        m_odom.header.stamp = ros::Time::now() ;
        //qDebug() << "send -> " << m_odom.pose.pose.orientation.z;
        imuPublisher.publish(m_odom);
        odomMutex.unlock();

        ros::spinOnce();
        r.sleep();
    }
    ros::shutdown();
#endif
    while(1){
        sleep(1);
//        publishData();
        QVector3D accVector;
        QVector3D gyrVector;
        QVector3D magVector;
//        emit anglesReady(accVector,gyrVector)
        qDebug()<<"looping...";

    }
    return;
}


void RosNode::publishData(const QVector3D &angles, const QVector3D &anglesVel)
{
#if 0
    nav_msgs::Odometry odom;

    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation.x = angles.x();
    odom.pose.pose.orientation.y = angles.y();
    odom.pose.pose.orientation.z = angles.z();

    //set the velocity
    //(x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    odom.child_frame_id = "base_link";
    odom.twist.twist.angular.x = anglesVel.x();
    odom.twist.twist.angular.y = anglesVel.y();
    odom.twist.twist.angular.z = anglesVel.z();

    odomMutex.lock();
    m_odom = odom;
    odomMutex.unlock();
#endif
}
