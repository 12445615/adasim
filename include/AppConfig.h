#pragma once

#include <QString>

class AppConfig {
public:
    bool load(const QString& path);

    quint16 tcpPort() const;
    quint16 udpLidarPort() const;
    QString canInterface() const;
    QString openDriveMap() const;
    double autoBrakeDistance() const;
    double autoBrakeWidth() const;
    int stateLogIntervalMs() const;
    int canTxIntervalMs() const;

private:
    quint16 tcpPort_ = 8848;
    quint16 udpLidarPort_ = 8850;
    QString canInterface_ = "vcan0";
    QString openDriveMap_ = "maps/demo.xodr";
    double autoBrakeDistance_ = 15.0;
    double autoBrakeWidth_ = 4.0;
    int stateLogIntervalMs_ = 200;
    int canTxIntervalMs_ = 50;
};
