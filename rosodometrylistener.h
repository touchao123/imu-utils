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

#ifndef ROSODOMETRYLISTENER_H
#define ROSODOMETRYLISTENER_H

#include <qnode.h>
#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>

class RosOdometryListener : public QNode
{
public:
    RosOdometryListener(int argc, char** argv);
    //virtual ~Listener() {}
    void run();
    void ros_comms_init();


private:
    void callbackFuction(const std_msgs::String::ConstPtr &msg);
    ros::Subscriber m_subscriber;

};

#endif // ROSODOMETRYLISTENER_H
