#include "AppConfig.h"

#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>

bool AppConfig::load(const QString& path) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        return false;
    }

    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    if (!doc.isObject()) {
        return false;
    }

    const QJsonObject obj = doc.object();
    tcpPort_ = static_cast<quint16>(obj.value("tcp_port").toInt(tcpPort_));
    udpLidarPort_ = static_cast<quint16>(obj.value("udp_lidar_port").toInt(udpLidarPort_));
    canInterface_ = obj.value("can_interface").toString(canInterface_);
    openDriveMap_ = obj.value("opendrive_map").toString(openDriveMap_);
    autoBrakeDistance_ = obj.value("auto_brake_distance").toDouble(autoBrakeDistance_);
    autoBrakeWidth_ = obj.value("auto_brake_width").toDouble(autoBrakeWidth_);
    stateLogIntervalMs_ = obj.value("state_log_interval_ms").toInt(stateLogIntervalMs_);
    canTxIntervalMs_ = obj.value("can_tx_interval_ms").toInt(canTxIntervalMs_);
    return true;
}

quint16 AppConfig::tcpPort() const {
    return tcpPort_;
}

quint16 AppConfig::udpLidarPort() const {
    return udpLidarPort_;
}

QString AppConfig::canInterface() const {
    return canInterface_;
}

QString AppConfig::openDriveMap() const {
    return openDriveMap_;
}

double AppConfig::autoBrakeDistance() const {
    return autoBrakeDistance_;
}

double AppConfig::autoBrakeWidth() const {
    return autoBrakeWidth_;
}

int AppConfig::stateLogIntervalMs() const {
    return stateLogIntervalMs_;
}

int AppConfig::canTxIntervalMs() const {
    return canTxIntervalMs_;
}
