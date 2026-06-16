#pragma once

#include <QObject>
#include <QJsonObject>
#include <QTcpServer>
#include <QTcpSocket>

class TcpControlServer : public QObject {
    Q_OBJECT

public:
    explicit TcpControlServer(QObject* parent = nullptr);
    bool start(quint16 port = 8848);
    bool isListening() const;
    bool hasClient() const;
    void sendJson(const QJsonObject& object);

signals:
    void controlReceived(double speed, double steering);
    void logMessage(const QString& message);

private slots:
    void onNewConnection();
    void onReadyRead();
    void onDisconnected();

private:
    void parseLine(const QByteArray& line);

    QTcpServer server_;
    QTcpSocket* socket_ = nullptr;
    QByteArray buffer_;
};
