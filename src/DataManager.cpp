#include "DataManager.h"

#include <QReadLocker>
#include <QWriteLocker>

DataManager& DataManager::instance() {
    static DataManager manager;
    return manager;
}

VehicleState DataManager::vehicleState() const {
    QReadLocker locker(&lock_);
    return vehicle_;
}

QVector<QPointF> DataManager::trajectory() const {
    QReadLocker locker(&lock_);
    return trajectory_;
}

QVector<LidarPoint> DataManager::lidarPoints() const {
    QReadLocker locker(&lock_);
    return lidarPoints_;
}

QVector<LidarCluster> DataManager::clusters() const {
    QReadLocker locker(&lock_);
    return clusters_;
}

QVector<QPointF> DataManager::virtualObstacles() const {
    QReadLocker locker(&lock_);
    return virtualObstacles_;
}

QVector<QLineF> DataManager::mapCenterLines() const {
    QReadLocker locker(&lock_);
    return mapCenterLines_;
}

QVector<QLineF> DataManager::mapLaneLines() const {
    QReadLocker locker(&lock_);
    return mapLaneLines_;
}

QVector<QPointF> DataManager::navigationPath() const {
    QReadLocker locker(&lock_);
    return navigationPath_;
}

QPointF DataManager::navigationGoal() const {
    QReadLocker locker(&lock_);
    return navigationGoal_;
}

QVector<QPointF> DataManager::navigationGoals() const {
    QReadLocker locker(&lock_);
    return navigationGoals_;
}

bool DataManager::hasNavigationGoal() const {
    QReadLocker locker(&lock_);
    return hasNavigationGoal_;
}

QString DataManager::latestLog() const {
    QReadLocker locker(&lock_);
    return latestLog_;
}

FrameSnapshot DataManager::snapshot() const {
    QReadLocker locker(&lock_);
    FrameSnapshot snapshot;
    snapshot.vehicle = vehicle_;
    snapshot.trajectory = trajectory_;
    snapshot.virtualObstacles = virtualObstacles_;
    snapshot.lidarPoints = lidarPoints_;
    snapshot.clusters = clusters_;
    snapshot.latestLog = latestLog_;
    snapshot.timestampMs = QDateTime::currentMSecsSinceEpoch();
    return snapshot;
}

void DataManager::setVehicleState(const VehicleState& state) {
    QWriteLocker locker(&lock_);
    vehicle_ = state;
}

void DataManager::appendTrajectoryPoint(const QPointF& point) {
    QWriteLocker locker(&lock_);
    trajectory_.append(point);
    if (trajectory_.size() > 2000) {
        trajectory_.remove(0, trajectory_.size() - 2000);
    }
}

void DataManager::setLidarData(const QVector<LidarPoint>& points, const QVector<LidarCluster>& clusters) {
    QWriteLocker locker(&lock_);
    lidarPoints_ = points;
    clusters_ = clusters;
}

void DataManager::setVirtualObstacles(const QVector<QPointF>& obstacles) {
    QWriteLocker locker(&lock_);
    virtualObstacles_ = obstacles;
}

void DataManager::setMapData(const QVector<QLineF>& centerLines, const QVector<QLineF>& laneLines) {
    QWriteLocker locker(&lock_);
    mapCenterLines_ = centerLines;
    mapLaneLines_ = laneLines;
}

void DataManager::setNavigationPath(const QVector<QPointF>& path, const QVector<QPointF>& goals) {
    QWriteLocker locker(&lock_);
    navigationPath_ = path;
    navigationGoals_ = goals;
    navigationGoal_ = goals.isEmpty() ? QPointF() : goals.last();
    hasNavigationGoal_ = !goals.isEmpty();
}

void DataManager::clearNavigationPath() {
    QWriteLocker locker(&lock_);
    navigationPath_.clear();
    navigationGoals_.clear();
    hasNavigationGoal_ = false;
}

void DataManager::setLatestLog(const QString& log) {
    QWriteLocker locker(&lock_);
    latestLog_ = log;
}

void DataManager::restoreSnapshot(const FrameSnapshot& snapshot) {
    QWriteLocker locker(&lock_);
    vehicle_ = snapshot.vehicle;
    trajectory_ = snapshot.trajectory;
    virtualObstacles_ = snapshot.virtualObstacles;
    lidarPoints_ = snapshot.lidarPoints;
    clusters_ = snapshot.clusters;
    latestLog_ = snapshot.latestLog;
}

void DataManager::reset() {
    QWriteLocker locker(&lock_);
    vehicle_ = VehicleState{};
    trajectory_.clear();
    lidarPoints_.clear();
    clusters_.clear();
    virtualObstacles_.clear();
    navigationPath_.clear();
    navigationGoals_.clear();
    hasNavigationGoal_ = false;
    latestLog_ = "system reset";
}
