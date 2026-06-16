#pragma once

#include "VehicleModel.h"

#include <QDateTime>
#include <QLineF>
#include <QPointF>
#include <QReadWriteLock>
#include <QRectF>
#include <QString>
#include <QVector>

struct LidarPoint {
    float x = 0.0f;
    float y = 0.0f;
    float intensity = 1.0f;
    int clusterId = -1;
};

struct LidarCluster {
    QPointF center;
    QRectF bounds;
    int pointCount = 0;
};

struct FrameSnapshot {
    VehicleState vehicle;
    QVector<QPointF> trajectory;
    QVector<QPointF> virtualObstacles;
    QVector<LidarPoint> lidarPoints;
    QVector<LidarCluster> clusters;
    QString latestLog;
    qint64 timestampMs = 0;
};

class DataManager {
public:
    static DataManager& instance();

    VehicleState vehicleState() const;
    QVector<QPointF> trajectory() const;
    QVector<LidarPoint> lidarPoints() const;
    QVector<LidarCluster> clusters() const;
    QVector<QPointF> virtualObstacles() const;
    QVector<QLineF> mapCenterLines() const;
    QVector<QLineF> mapLaneLines() const;
    QVector<QPointF> navigationPath() const;
    QPointF navigationGoal() const;
    QVector<QPointF> navigationGoals() const;
    bool hasNavigationGoal() const;
    QString latestLog() const;
    FrameSnapshot snapshot() const;

    void setVehicleState(const VehicleState& state);
    void appendTrajectoryPoint(const QPointF& point);
    void setLidarData(const QVector<LidarPoint>& points, const QVector<LidarCluster>& clusters);
    void setVirtualObstacles(const QVector<QPointF>& obstacles);
    void setMapData(const QVector<QLineF>& centerLines, const QVector<QLineF>& laneLines);
    void setNavigationPath(const QVector<QPointF>& path, const QVector<QPointF>& goals);
    void clearNavigationPath();
    void setLatestLog(const QString& log);
    void restoreSnapshot(const FrameSnapshot& snapshot);
    void reset();

private:
    DataManager() = default;

    mutable QReadWriteLock lock_;
    VehicleState vehicle_;
    QVector<QPointF> trajectory_;
    QVector<LidarPoint> lidarPoints_;
    QVector<LidarCluster> clusters_;
    QVector<QPointF> virtualObstacles_;
    QVector<QLineF> mapCenterLines_;
    QVector<QLineF> mapLaneLines_;
    QVector<QPointF> navigationPath_;
    QPointF navigationGoal_;
    QVector<QPointF> navigationGoals_;
    bool hasNavigationGoal_ = false;
    QString latestLog_ = "system ready";
};
