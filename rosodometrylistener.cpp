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
#include <ros/header.h>
#include <ros/message.h>
#include <string>
#include <std_msgs/String.h>
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
    std::string topic = "/rosout/rosgraph_msgs/Log";
    ros::NodeHandle node;
    m_subscriber = node.subscribe(topic, 1000, &RosOdometryListener::callbackFuction, this);
    qDebug() << "ROS -> subscribet to topic" << QString::fromStdString(topic);
}

void RosOdometryListener::callbackFuction(const std_msgs::String::ConstPtr &msg)
{
    qDebug() << "got something: " << msg;
//    ROS_INFO("I heard: [%s]", msg->data.c_str());
//    logging.insertRows(0,1);
//    std::stringstream logging_msg;
//    logging_msg << "[ INFO] [" << ros::Time::now() << "]: I heard: " << msg->data;
//    QVariant new_row(QString(logging_msg.str().c_str()));
//    logging.setData(logging.index(0),new_row);

}
