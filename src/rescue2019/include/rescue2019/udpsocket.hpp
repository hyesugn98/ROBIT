#ifndef UDPSOCKET_H
#define UDPSOCKET_H

#endif // UDPSOCKET_H
#include <iostream>
#include <QObject>
#include <QtNetwork/qudpsocket.h>
#include <QtNetwork/qnetworkinterface.h>
#include <QStringList>
#include <QList>
#include <QBuffer>

class UdpSocket : public QObject
{
    Q_OBJECT
public:
    explicit UdpSocket(int port, QObject *parent = 0);

    QByteArray getBuffer(){return buffer;}
    QHostAddress getIPAddresses();
    int getPort(){return mPort;}
    inline int getDataSize(){return dataSize;}
    void setAddress(QHostAddress address){mAddress = address;}
    void setPort(int port){mPort = port;}
    void ChangeSendAddress(QHostAddress Address);
    static QHostAddress ARRIBAL_IP;

Q_SIGNALS:
    void recvBuffer();

public Q_SLOTS:
    void readData();
    void sendData(QByteArray *send_data);

private:
    QUdpSocket* socket;
    QByteArray buffer;
    QList<QHostAddress> ipAddressesList;
    QHostAddress mAddress;
    int dataSize;
    int mPort;
};


