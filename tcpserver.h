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
