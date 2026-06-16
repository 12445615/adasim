#include "TcpControlServer.h"

#include <QJsonDocument>
#include <QJsonObject>

TcpControlServer::TcpControlServer(QObject* parent)
    : QObject(parent) {
    connect(&server_, &QTcpServer::newConnection, this, &TcpControlServer::onNewConnection);
}

bool TcpControlServer::start(quint16 port) {
    if (server_.isListening()) {
        return true;
    }
    const bool ok = server_.listen(QHostAddress::Any, port);
    emit logMessage(ok ? QString("TCP server listening on %1").arg(port)
                       : QString("TCP server failed: %1").arg(server_.errorString()));
    return ok;
}

bool TcpControlServer::isListening() const {
    return server_.isListening();
}

bool TcpControlServer::hasClient() const {
    return socket_ && socket_->state() == QAbstractSocket::ConnectedState;
}

void TcpControlServer::sendJson(const QJsonObject& object) {
    if (!hasClient()) {
        return;
    }

    socket_->write(QJsonDocument(object).toJson(QJsonDocument::Compact));
    socket_->write("\n");
    socket_->flush();
}

void TcpControlServer::onNewConnection() {
    if (socket_) {
        socket_->disconnectFromHost();
        socket_->deleteLater();
    }

    socket_ = server_.nextPendingConnection();
    buffer_.clear();
    connect(socket_, &QTcpSocket::readyRead, this, &TcpControlServer::onReadyRead);
    connect(socket_, &QTcpSocket::disconnected, this, &TcpControlServer::onDisconnected);
    emit logMessage(QString("algorithm node connected: %1:%2")
                        .arg(socket_->peerAddress().toString())
                        .arg(socket_->peerPort()));
}

void TcpControlServer::onReadyRead() {
    if (!socket_) {
        return;
    }

    buffer_.append(socket_->readAll());
    int newline = buffer_.indexOf('\n');
    while (newline >= 0) {
        const QByteArray line = buffer_.left(newline).trimmed();
        buffer_.remove(0, newline + 1);
        if (!line.isEmpty()) {
            parseLine(line);
        }
        newline = buffer_.indexOf('\n');
    }
}

void TcpControlServer::onDisconnected() {
    emit logMessage("algorithm node disconnected");
    if (socket_) {
        socket_->deleteLater();
        socket_ = nullptr;
    }
    buffer_.clear();
}

void TcpControlServer::parseLine(const QByteArray& line) {
    const QJsonDocument doc = QJsonDocument::fromJson(line);
    if (!doc.isObject()) {
        emit logMessage("invalid json packet");
        return;
    }

    const QJsonObject obj = doc.object();
    if (obj.value("type").toString() != "CONTROL_KINEMATIC") {
        emit logMessage(QString("ignored packet: %1").arg(QString::fromUtf8(line)));
        return;
    }

    const double speed = obj.value("speed").toDouble(5.0);
    const double steering = obj.value("steering").toDouble(0.0);
    emit controlReceived(speed, steering);
    emit logMessage(QString("control speed=%1 steering=%2").arg(speed, 0, 'f', 2).arg(steering, 0, 'f', 3));
}
