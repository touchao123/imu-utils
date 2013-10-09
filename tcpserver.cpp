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

#include <QDebug>
#include <QJsonDocument>

#include "tcpserver.h"

TcpServer::TcpServer(int port, QObject *parent) :
    m_port(port),QObject(parent)
{
    qDebug() << "--------------------------------------------";
    qDebug() << "searching network interfaces...";
    foreach(const QNetworkInterface &interface, QNetworkInterface::allInterfaces()){
        qDebug() << "-----------------------------";
        qDebug() << "   name:" << interface.name();
        qDebug() << "   mac: " << interface.hardwareAddress();
    }
    qDebug() << "--------------------------------------------";

}

void TcpServer::newClientConnected()
{
    // got a new client connected
    QTcpServer *server = qobject_cast<QTcpServer*>(sender());
    QTcpSocket *newConnection = server->nextPendingConnection();
    qDebug() << "\nserver -> new client connected:" << newConnection->peerAddress().toString();

    // append the new client to the client list
    m_clientList.append(newConnection);

    connect(newConnection, SIGNAL(readyRead()),this,SLOT(readPackage()));
    connect(newConnection,SIGNAL(disconnected()),this,SLOT(clientDisconnected()));

}

void TcpServer::readPackage()
{
    QTcpSocket *client = qobject_cast<QTcpSocket*>(sender());
    //qDebug() << "-----------> data comming from" << client->peerAddress().toString();
    QByteArray message;
    while(client->canReadLine()){
        QByteArray dataLine = client->readLine();
        message.append(dataLine);
        if(dataLine == "}\n"){
            //qDebug() << message;
            emit jsonDataAvailable(message);
            message.clear();
        }
    }
}

void TcpServer::clientDisconnected()
{
    QTcpSocket *client = qobject_cast<QTcpSocket*>(sender());
    qDebug() << "\nserver -> client disconnected:" << client->peerAddress().toString();
}

bool TcpServer::startServer()
{
    // Listen on all Networkinterfaces
    foreach(const QHostAddress &address, QNetworkInterface::allAddresses()){
        QTcpServer *server = new QTcpServer(this);
        if(server->listen(address, m_port)) {
            qDebug() << "server -> listening on" << address.toString() << m_port;
            connect(server, SIGNAL(newConnection()), SLOT(newClientConnected()));
            m_serverList.append(server);
        } else {
            qDebug() << "ERROR: can not listening to" << address.toString() << m_port;
            delete server;
        }
    }
    qDebug() << "--------------------------------------------";
    if(m_serverList.empty()){
        qDebug() << "could not start TCP server on any interface.";
        exit(1);
    }
    return true;
}

bool TcpServer::stopServer()
{
    // Listen on all Networkinterfaces
    foreach(QTcpServer *server, m_serverList){
        qDebug() << "server -> closed" << server->serverAddress().toString() << m_port;
        server->close();
        delete server;
    }
    if(!m_serverList.empty()){
        return false;
    }
    return true;
}

void TcpServer::sendToAll(QByteArray data)
{
    foreach(QTcpSocket *client,m_clientList){
        //qDebug() << QJsonDocument::fromBinaryData(data).toJson();
        client->write(data);
    }
}
