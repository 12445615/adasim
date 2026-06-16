#include "UdpLidarReceiver.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

UdpLidarReceiver::UdpLidarReceiver(QObject* parent)
    : QObject(parent) {
    connect(&socket_, &QUdpSocket::readyRead, this, &UdpLidarReceiver::onReadyRead);
}

bool UdpLidarReceiver::start(quint16 port) {
    const bool ok = socket_.bind(QHostAddress::Any, port, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
    emit logMessage(ok ? QString("UDP LiDAR receiver listening on %1").arg(port)
                       : QString("UDP LiDAR receiver failed: %1").arg(socket_.errorString()));
    return ok;
}

void UdpLidarReceiver::onReadyRead() {
    while (socket_.hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(static_cast<int>(socket_.pendingDatagramSize()));
        socket_.readDatagram(datagram.data(), datagram.size());
        parseDatagram(datagram);
    }
}

void UdpLidarReceiver::parseDatagram(const QByteArray& datagram) {
    const QJsonDocument doc = QJsonDocument::fromJson(datagram);
    if (!doc.isObject()) {
        emit logMessage("invalid UDP LiDAR packet");
        return;
    }

    const QJsonObject obj = doc.object();
    if (obj.value("type").toString() != "LIDAR_POINTS") {
        return;
    }

    QVector<LidarPoint> points;
    const QJsonArray array = obj.value("points").toArray();
    points.reserve(array.size());
    for (const QJsonValue& value : array) {
        const QJsonArray item = value.toArray();
        if (item.size() < 2) {
            continue;
        }
        LidarPoint point;
        point.x = static_cast<float>(item.at(0).toDouble());
        point.y = static_cast<float>(item.at(1).toDouble());
        point.intensity = item.size() >= 3 ? static_cast<float>(item.at(2).toDouble()) : 1.0f;
        points.append(point);
    }

    if (!points.isEmpty()) {
        emit pointsReceived(points);
    }
}
