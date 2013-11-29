/*
 *    This application is able to read the IMU 9-DOF sensor stick
 *    from Sparfunk SEN-10724.
 *
 *    Copyright (C) 2013 Simon Stürz (stuerz.simon@gmail.com)
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "rosodometrylistener.h"

#include <QDebug>
#include <ros/ros.h>
#include <ros/time.h>
#include <string>
#include <std_msgs/String.h>
#include <turtlesim/Velocity.h>
#include <QString>

RosOdometryListener::RosOdometryListener(int argc, char **argv):
    QNode(argc,argv,"imu_utils")
{
}

void RosOdometryListener::run()
{
    ros::spin();
    qDebug()<< "Ros shutdown.";
    emit rosShutdown();
}

void RosOdometryListener::ros_comms_init()
{
    std::string topic = "/turtle1/pose";
    ros::NodeHandle node;
    m_subscriber = node.subscribe(topic, 1000, &RosOdometryListener::callbackFuction, this);
    qDebug() << "ROS -> subscribet to topic" << QString::fromStdString(topic);
}

void RosOdometryListener::callbackFuction(const turtlesim::Pose::ConstPtr &message)
{
    qDebug() << "--------------------------------";
    qDebug() << "turtlesim:";
    qDebug() << "   " << message->x;
    qDebug() << "   " << message->y;
    qDebug() << "   " << message->theta;
}
