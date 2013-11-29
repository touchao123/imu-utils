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

#include "qnode.h"

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <QDebug>
#include <QString>


QNode::QNode(int argc, char **argv, const std::string &name):
    init_argc(argc), init_argv(argv), node_name(name)
{

}

QNode::~QNode() {
    shutdown();
}

bool QNode::onInit()
{
    ros::init(init_argc,init_argv,node_name);
    if ( ! ros::master::check() ) {
        qDebug() << "--------------------------------------------";
        qDebug() << "ERROR: could not init ROS.";
        qDebug() << "--------------------------------------------";
        return false;
    }
    ros::start();
    ros_comms_init();
    start();
    qDebug() << "--------------------------------------------";
    qDebug() << "ROS -> Node "<< QString::fromStdString(node_name) << "started.";
    qDebug() << "--------------------------------------------";
    return true;
}

bool QNode::onInit(const std::string &master_url, const std::string &host_url)
{
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,node_name);
    if ( ! ros::master::check() ) {
        qDebug() << "--------------------------------------------";
        qDebug() << "ERROR: could not init ROS.";
        qDebug() << "--------------------------------------------";
        return false;
    }
    // start ros
    ros::start();
    ros_comms_init();

    // start thread
    start();
    qDebug() << "--------------------------------------------";
    qDebug() << "ROS -> Node " << QString::fromStdString(node_name) << "started.";
    qDebug() << "       MASTER   = " << QString::fromStdString(master_url);
    qDebug() << "       HOSTNAME = " << QString::fromStdString(host_url);
    qDebug() << "--------------------------------------------";

    return true;
}

void QNode::shutdown()
{
    if(ros::isStarted()) {
        ros::shutdown();
        ros::waitForShutdown();
        terminate();
        quit();
        exit(0);


    }
    //wait();
    ros::shutdown();
    ros::waitForShutdown();
    terminate();
    quit();
    exit(0);

}

