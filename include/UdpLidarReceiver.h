#pragma once

#include "DataManager.h"

#include <QObject>
#include <QUdpSocket>

class UdpLidarReceiver : public QObject {
    Q_OBJECT

public:
    explicit UdpLidarReceiver(QObject* parent = nullptr);
    bool start(quint16 port = 8850);

signals:
    void pointsReceived(const QVector<LidarPoint>& points);
    void logMessage(const QString& message);

private slots:
    void onReadyRead();

private:
    void parseDatagram(const QByteArray& datagram);

    QUdpSocket socket_;
};
