#include "../include/rescue2019/udpsocket.hpp"

using namespace std;

UdpSocket::UdpSocket(int port, QObject *parent) :
    QObject(parent)
   ,mPort(port)
{
    socket = new QUdpSocket(this);
    ipAddressesList = QNetworkInterface::allAddresses();

    if(socket->bind(mPort , QUdpSocket::ShareAddress))
        connect(socket, SIGNAL(readyRead()), this, SLOT(readData()));
}

void UdpSocket::readData()
{
    buffer.resize(socket->pendingDatagramSize());

    QHostAddress sender;
    quint16 senderPort;

    socket->readDatagram(buffer.data(), buffer.size(), &sender, &senderPort);
//    emit recvBuffer();
}

void UdpSocket::ChangeSendAddress(QHostAddress Address)
{
    UdpSocket::ARRIBAL_IP = Address;
}

void UdpSocket::sendData(QByteArray *send_data)
{
    socket->writeDatagram(send_data->data(),send_data->size(), UdpSocket::ARRIBAL_IP, this->mPort);
}

QHostAddress UdpSocket::getIPAddresses()
{
    mAddress = QHostAddress::LocalHost;
    QStringList ipList;

    if(!ipAddressesList.isEmpty())
    {
        for(int i = 0 ; i < ipAddressesList.size() ; i ++)
        {
            ipList = ipAddressesList[i].toString().split(".") ;
            if(ipList[0] == "192" && ipList[1] == "168")
            {
                mAddress = ipAddressesList[i];
                break;
            }
        }
    }
    return mAddress;
}

