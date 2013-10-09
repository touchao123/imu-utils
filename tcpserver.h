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

#ifndef TCPSERVER_H
#define TCPSERVER_H

#include <QObject>
#include <QList>
#include <QNetworkInterface>
#include <QTcpServer>
#include <QTcpSocket>
#include <QByteArray>

class TcpServer : public QObject
{
    Q_OBJECT
public:
    explicit TcpServer(int port = 55555, QObject *parent = 0);

private:
    int m_port;
    QList<QTcpServer*> m_serverList;
    QList<QTcpSocket*> m_clientList;

signals:
    void jsonDataAvailable(const QByteArray &data);

private slots:
    void newClientConnected();
    void readPackage();
    void clientDisconnected();

public slots:
    bool startServer();
    bool stopServer();
    void sendToAll(QByteArray data);
};

#endif // TCPSERVER_H
