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

#ifndef QNODE_H
#define QNODE_H

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>

class QNode : public QThread
{
    Q_OBJECT
public:
    explicit QNode(int argc, char** argv, const std::string &name );
    virtual ~QNode();

    bool onInit();
    bool onInit(const std::string &master_url, const std::string &host_url);

    void shutdown();
    virtual void run() = 0;

    QStringListModel* loggingModel() { return &logging; }
    const std::string& nodeName() { return node_name; }

signals:
    void loggingUpdated();
    void rosShutdown();

protected:
    virtual void ros_comms_init() = 0;
    int init_argc;
    char** init_argv;
    QStringListModel logging;
    const std::string node_name;

public slots:

};

#endif // QNODE_H
