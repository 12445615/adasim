#pragma once

#include <QLineF>
#include <QString>
#include <QVector>

class OpenDriveMap {
public:
    bool load(const QString& path);
    bool isLoaded() const;
    QString errorString() const;
    QVector<QLineF> centerLines() const;
    QVector<QLineF> laneLines() const;

private:
    void appendLine(double x, double y, double heading, double length);
    void appendArc(double x, double y, double heading, double length, double curvature);
    void rebuildLaneLines();

    QVector<QLineF> centerLines_;
    QVector<QLineF> laneLines_;
    QString errorString_;
    double laneHalfWidth_ = 3.5;
};
