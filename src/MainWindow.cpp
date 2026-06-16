#include "MainWindow.h"

#include "DataManager.h"
#include "LidarWidget.h"
#include "SimulationWidget.h"

#include <QApplication>
#include <QDir>
#include <QDateTime>
#include <QFileDialog>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QRandomGenerator>
#include <QSizeF>
#include <QSplitter>
#include <QTime>
#include <QVBoxLayout>
#include <cmath>
#include <limits>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent),
      canBusManager_(this),
      replay_(this),
      tcpServer_(this),
      udpLidarReceiver_(this) {
    buildUi();

    connect(&timer_, &QTimer::timeout, this, &MainWindow::tick);
    connect(&canBusManager_, &CanBusManager::logMessage, this, &MainWindow::appendLog);
    connect(&tcpServer_, &TcpControlServer::controlReceived, this, &MainWindow::applyControl);
    connect(&tcpServer_, &TcpControlServer::logMessage, this, &MainWindow::appendLog);
    connect(&udpLidarReceiver_, &UdpLidarReceiver::pointsReceived, this, &MainWindow::applyExternalLidar);
    connect(&udpLidarReceiver_, &UdpLidarReceiver::logMessage, this, &MainWindow::appendLog);
    connect(simulationWidget_, &SimulationWidget::virtualObstacleRequested, this, &MainWindow::injectVirtualObstacle);
    connect(simulationWidget_, &SimulationWidget::navigationGoalRequested, this, &MainWindow::planRouteToGoal);
    connect(&replay_, &ReplayManager::restored, simulationWidget_, QOverload<>::of(&QWidget::update));
    connect(&replay_, &ReplayManager::restored, lidarWidget_, QOverload<>::of(&QWidget::update));

    const QString configPath = QCoreApplication::applicationDirPath() + "/config/adasim.json";
    if (config_.load(configPath)) {
        appendLog(QString("config loaded: %1").arg(QDir::toNativeSeparators(configPath)));
    } else {
        appendLog("config not found, using defaults");
    }

    DataManager::instance().setVehicleState(vehicle_.state());
    if (logDatabase_.open(QCoreApplication::applicationDirPath() + "/adasim_run.db")) {
        logDatabase_.insertEvent("SYSTEM", "database opened");
    } else {
        appendLog("SQLite database open failed");
    }
    loadOpenDriveMap();
    tcpServer_.start(config_.tcpPort());
    udpLidarReceiver_.start(config_.udpLidarPort());
    canBusManager_.open(config_.canInterface());
    elapsed_.start();
    timer_.start(16);
}

void MainWindow::buildUi() {
    setWindowTitle("ADASim - Embedded Vehicle HMI");
    resize(1280, 780);

    auto* root = new QWidget(this);
    auto* rootLayout = new QVBoxLayout(root);
    rootLayout->setSpacing(10);
    rootLayout->setContentsMargins(12, 12, 12, 12);

    auto* topBar = new QHBoxLayout();
    auto* loadMapButton = new QPushButton("Load XODR");
    startButton_ = new QPushButton("启动");
    pauseButton_ = new QPushButton("暂停");
    auto* resetButton = new QPushButton("重置");
    auto* clearObstacleButton = new QPushButton("清空障碍物");
    statusLabel_ = new QLabel("算法状态: 就绪 | TCP:8848");
    topBar->addWidget(startButton_);
    topBar->addWidget(pauseButton_);
    topBar->addWidget(resetButton);
    topBar->addWidget(clearObstacleButton);
    topBar->addWidget(loadMapButton);
    topBar->addStretch();
    topBar->addWidget(statusLabel_);
    rootLayout->addLayout(topBar);

    auto* telemetry = new QHBoxLayout();
    speedLabel_ = new QLabel("速度: 0.0 m/s");
    mileageLabel_ = new QLabel("里程: 0.0 m");
    fpsLabel_ = new QLabel("FPS: 0");
    pointCountLabel_ = new QLabel("点云: 0");
    clusterCountLabel_ = new QLabel("聚类: 0");
    packetCountLabel_ = new QLabel("报文: 0");
    replayCountLabel_ = new QLabel("回放帧: 0");
    telemetry->addWidget(speedLabel_);
    telemetry->addWidget(mileageLabel_);
    telemetry->addWidget(fpsLabel_);
    telemetry->addWidget(pointCountLabel_);
    telemetry->addWidget(clusterCountLabel_);
    telemetry->addWidget(packetCountLabel_);
    telemetry->addWidget(replayCountLabel_);
    telemetry->addStretch();
    rootLayout->addLayout(telemetry);

    simulationWidget_ = new SimulationWidget();
    lidarWidget_ = new LidarWidget();

    auto* rightPanel = new QWidget();
    auto* rightLayout = new QVBoxLayout(rightPanel);
    rightLayout->setContentsMargins(0, 0, 0, 0);
    rightLayout->addWidget(lidarWidget_);

    auto* algoBox = new QGroupBox("算法与通信");
    auto* algoLayout = new QVBoxLayout(algoBox);
    algoLayout->addWidget(new QLabel("Python 节点: TCP JSON 控制"));
    algoLayout->addWidget(new QLabel("感知模块: BFS 欧式聚类"));
    algoLayout->addWidget(new QLabel("控制模块: PID/MPC 接口预留"));
    rightLayout->addWidget(algoBox);

    auto* splitter = new QSplitter();
    splitter->addWidget(simulationWidget_);
    splitter->addWidget(rightPanel);
    splitter->setStretchFactor(0, 3);
    splitter->setStretchFactor(1, 1);
    rootLayout->addWidget(splitter, 1);

    logEdit_ = new QTextEdit();
    logEdit_->setReadOnly(true);
    logEdit_->setMaximumHeight(130);
    rootLayout->addWidget(logEdit_);

    auto* replayRow = new QHBoxLayout();
    auto* replayLabel = new QLabel("回放");
    auto* realtimeLabel = new QLabel("实时");
    replaySlider_ = new QSlider(Qt::Horizontal);
    replaySlider_->setRange(0, 100);
    replaySlider_->setValue(100);
    replaySlider_->setMinimumHeight(28);
    replaySlider_->setTickPosition(QSlider::TicksBelow);
    replaySlider_->setTickInterval(10);
    replayRow->addWidget(replayLabel);
    replayRow->addWidget(replaySlider_, 1);
    replayRow->addWidget(realtimeLabel);
    rootLayout->addLayout(replayRow);

    setCentralWidget(root);
    setStyleSheet(
        "QMainWindow, QWidget { background: #11161b; color: #e8edf2; font-size: 14px; }"
        "QPushButton { background: #2f7dd1; color: white; border: 0; padding: 8px 14px; border-radius: 4px; }"
        "QPushButton:hover { background: #3d8ce3; }"
        "QLabel { color: #d9e2ea; }"
        "QTextEdit, QGroupBox { background: #171d23; border: 1px solid #2a3640; border-radius: 6px; }"
        "QGroupBox { padding-top: 18px; }"
        "QSlider::groove:horizontal { height: 8px; background: #2b3742; border-radius: 4px; }"
        "QSlider::handle:horizontal { background: #38b778; width: 18px; margin: -7px 0; border-radius: 9px; }"
        "QSlider::sub-page:horizontal { background: #2f7dd1; border-radius: 4px; }");

    connect(startButton_, &QPushButton::clicked, this, &MainWindow::startSimulation);
    connect(pauseButton_, &QPushButton::clicked, this, &MainWindow::pauseSimulation);
    connect(resetButton, &QPushButton::clicked, this, &MainWindow::resetSimulation);
    connect(clearObstacleButton, &QPushButton::clicked, this, &MainWindow::clearVirtualObstacles);
    connect(loadMapButton, &QPushButton::clicked, this, &MainWindow::chooseOpenDriveMap);
    connect(replaySlider_, &QSlider::valueChanged, this, &MainWindow::replayChanged);
}

void MainWindow::loadOpenDriveMap(const QString& selectedPath) {
    const QString configuredPath = selectedPath.isEmpty() ? config_.openDriveMap() : selectedPath;
    const QString mapPath = QDir::isAbsolutePath(configuredPath)
        ? configuredPath
        : QCoreApplication::applicationDirPath() + "/" + configuredPath;
    if (openDriveMap_.load(mapPath)) {
        const QVector<QLineF> centerLines = openDriveMap_.centerLines();
        DataManager::instance().setMapData(centerLines, openDriveMap_.laneLines());
        DataManager::instance().clearNavigationPath();
        navigationGoals_.clear();
        injectedObstacles_.clear();
        routeFollowingEnabled_ = false;
        routePathIndex_ = 1;
        activeGoalIndex_ = 0;
        if (!centerLines.isEmpty()) {
            const QLineF firstRoad = centerLines.first();
            vehicle_.setPose(firstRoad.p1().x(), firstRoad.p1().y(), -firstRoad.angle() * 3.14159265358979323846 / 180.0);
            replay_.clear();
            DataManager::instance().reset();
            DataManager::instance().setMapData(centerLines, openDriveMap_.laneLines());
            DataManager::instance().setVehicleState(vehicle_.state());
            DataManager::instance().appendTrajectoryPoint(firstRoad.p1());
            replaySlider_->setValue(100);
        }
        appendLog(QString("OpenDRIVE map loaded: %1 center segments")
            .arg(centerLines.size()));
        logDatabase_.insertEvent("MAP_LOAD", QString("loaded %1").arg(QDir::toNativeSeparators(mapPath)));
        simulationWidget_->update();
    } else {
        appendLog(openDriveMap_.errorString());
        logDatabase_.insertEvent("MAP_LOAD_FAILED", openDriveMap_.errorString());
    }
}

void MainWindow::chooseOpenDriveMap() {
    const QString startDir = QCoreApplication::applicationDirPath() + "/maps";
    const QString filePath = QFileDialog::getOpenFileName(
        this,
        "Load OpenDRIVE Map",
        startDir,
        "OpenDRIVE (*.xodr);;XML (*.xml);;All Files (*)");

    if (filePath.isEmpty()) {
        return;
    }

    loadOpenDriveMap(filePath);
}

void MainWindow::startSimulation() {
    running_ = true;
    replaying_ = false;
    replaySlider_->setValue(100);
    appendLog("simulation started");
}

void MainWindow::pauseSimulation() {
    running_ = false;
    appendLog("simulation paused");
}

void MainWindow::resetSimulation() {
    running_ = false;
    vehicle_.reset();
    replay_.clear();
    injectedObstacles_.clear();
    controlPacketCount_ = 0;
    braking_ = false;
    routeFollowingEnabled_ = false;
    routePathIndex_ = 1;
    activeGoalIndex_ = 0;
    navigationGoals_.clear();
    DataManager::instance().reset();
    DataManager::instance().setVehicleState(vehicle_.state());
    replaySlider_->setValue(100);
    simulationWidget_->update();
    lidarWidget_->update();
    appendLog("world reset");
}

void MainWindow::tick() {
    const double dt = qMax(0.001, elapsed_.restart() / 1000.0);
    if (running_ && !replaying_) {
        if (hasBlockingObstacle()) {
            const VehicleState state = vehicle_.state();
            vehicle_.setControl(0.0, state.steering);
            if (!braking_) {
                braking_ = true;
                appendLog("AUTO BRAKE: virtual obstacle detected ahead");
                logDatabase_.insertEvent("AUTO_BRAKE", "virtual obstacle detected ahead");
            }
        } else if (braking_) {
            braking_ = false;
            appendLog("AUTO BRAKE released");
            logDatabase_.insertEvent("AUTO_BRAKE_RELEASE", "front safety zone cleared");
        }

        if (!braking_ && routeFollowingEnabled_) {
            updatePathFollowing();
        }

        vehicle_.update(dt);
        const VehicleState state = vehicle_.state();
        DataManager::instance().setVehicleState(state);
        DataManager::instance().appendTrajectoryPoint(QPointF(state.x, state.y));
        if (!externalLidarActive_ || QDateTime::currentMSecsSinceEpoch() - lastExternalLidarMs_ > 1000) {
            externalLidarActive_ = false;
            generateLidar();
        }
        replay_.record(DataManager::instance().snapshot());

        const qint64 now = QDateTime::currentMSecsSinceEpoch();
        if (now - lastCanTxMs_ > config_.canTxIntervalMs()) {
            lastCanTxMs_ = now;
            canBusManager_.sendControl(state.speed, state.steering, braking_);
        }
        if (now - lastStateLogMs_ > config_.stateLogIntervalMs()) {
            lastStateLogMs_ = now;
            logDatabase_.insertVehicleState(
                state,
                DataManager::instance().lidarPoints().size(),
                DataManager::instance().clusters().size(),
                braking_);
        }
    }

    updateTelemetry(dt);
    simulationWidget_->update();
    lidarWidget_->update();
}

void MainWindow::applyControl(double speed, double steering) {
    ++controlPacketCount_;
    vehicle_.setControl(speed, steering);
    logDatabase_.insertControl(speed, steering);
}

void MainWindow::appendLog(const QString& message) {
    const QString line = QString("[%1] %2")
        .arg(QTime::currentTime().toString("HH:mm:ss.zzz"))
        .arg(message);
    DataManager::instance().setLatestLog(line);
    logEdit_->append(line);
}

void MainWindow::replayChanged(int value) {
    if (value >= 100) {
        replaying_ = false;
        return;
    }
    replaying_ = true;
    running_ = false;
    replay_.restoreByPercent(value);
}

void MainWindow::injectVirtualObstacle(const QPointF& worldPoint) {
    injectedObstacles_.append(worldPoint);
    if (injectedObstacles_.size() > 32) {
        injectedObstacles_.removeFirst();
    }
    DataManager::instance().setVirtualObstacles(injectedObstacles_);
    appendLog(QString("virtual obstacle injected world=(%1,%2)")
        .arg(worldPoint.x(), 0, 'f', 1)
        .arg(worldPoint.y(), 0, 'f', 1));
    logDatabase_.insertEvent(
        "VIRTUAL_OBSTACLE",
        QString("world=(%1,%2)").arg(worldPoint.x(), 0, 'f', 1).arg(worldPoint.y(), 0, 'f', 1));
    generateLidar();
    simulationWidget_->update();
    lidarWidget_->update();
}

void MainWindow::planRouteToGoal(const QPointF& worldPoint) {
    navigationGoals_.append(worldPoint);
    if (navigationGoals_.size() == 1 || activeGoalIndex_ >= navigationGoals_.size()) {
        activeGoalIndex_ = qMax(0, navigationGoals_.size() - 1);
    }

    activateGoalRoute();

    appendLog(QString("goal queued: %1 goals, active=%2, latest=(%3,%4)")
        .arg(navigationGoals_.size())
        .arg(activeGoalIndex_ + 1)
        .arg(worldPoint.x(), 0, 'f', 1)
        .arg(worldPoint.y(), 0, 'f', 1));
    logDatabase_.insertEvent(
        "ROUTE_PLAN",
        QString("queued goals=%1 active=%2 latest=(%3,%4)")
            .arg(navigationGoals_.size())
            .arg(activeGoalIndex_ + 1)
            .arg(worldPoint.x(), 0, 'f', 1)
            .arg(worldPoint.y(), 0, 'f', 1));
    simulationWidget_->update();
}

void MainWindow::activateGoalRoute() {
    if (activeGoalIndex_ < 0 || activeGoalIndex_ >= navigationGoals_.size()) {
        routeFollowingEnabled_ = false;
        DataManager::instance().setNavigationPath({}, navigationGoals_);
        return;
    }

    const VehicleState state = DataManager::instance().vehicleState();
    const QPointF start(state.x, state.y);
    const QPointF goal = navigationGoals_.at(activeGoalIndex_);
    const QVector<QPointF> centerPath = routePlanner_.plan(DataManager::instance().mapCenterLines(), start, goal);
    const QVector<QPointF> rightLanePath = offsetPathToRightLane(centerPath);

    DataManager::instance().setNavigationPath(rightLanePath, navigationGoals_);
    routeFollowingEnabled_ = rightLanePath.size() >= 2;
    routePathIndex_ = 1;

    appendLog(QString("active route planned: target %1/%2, waypoints=%3")
        .arg(activeGoalIndex_ + 1)
        .arg(navigationGoals_.size())
        .arg(rightLanePath.size()));
}

void MainWindow::clearVirtualObstacles() {
    injectedObstacles_.clear();
    braking_ = false;
    DataManager::instance().setVirtualObstacles(injectedObstacles_);
    generateLidar();
    simulationWidget_->update();
    lidarWidget_->update();
    appendLog("virtual obstacles cleared");
    logDatabase_.insertEvent("CLEAR_OBSTACLES", "all virtual obstacles cleared");
}

void MainWindow::applyExternalLidar(const QVector<LidarPoint>& points) {
    QVector<LidarPoint> merged = points;
    appendInjectedObstaclePoints(merged);
    QVector<LidarCluster> clusters = clusterPoints(merged);
    DataManager::instance().setLidarData(merged, clusters);
    externalLidarActive_ = true;
    lastExternalLidarMs_ = QDateTime::currentMSecsSinceEpoch();
}

void MainWindow::updateTelemetry(double dt) {
    const VehicleState state = DataManager::instance().vehicleState();
    speedLabel_->setText(QString("速度: %1 m/s").arg(state.speed, 0, 'f', 1));
    mileageLabel_->setText(QString("里程: %1 m").arg(state.mileage, 0, 'f', 1));
    fpsLabel_->setText(QString("FPS: %1").arg(qRound(1.0 / dt)));
    pointCountLabel_->setText(QString("点云: %1").arg(DataManager::instance().lidarPoints().size()));
    clusterCountLabel_->setText(QString("聚类: %1").arg(DataManager::instance().clusters().size()));
    packetCountLabel_->setText(QString("报文: %1").arg(controlPacketCount_));
    replayCountLabel_->setText(QString("回放帧: %1").arg(replay_.size()));
    statusLabel_->setText(braking_
        ? "算法状态: 障碍物制动 | TCP:8848"
        : "算法状态: 就绪 | TCP:8848");
}

void MainWindow::generateLidar() {
    QVector<LidarPoint> points;
    const QVector<QPointF> obstacles = {
        QPointF(16.0, 8.0),
        QPointF(24.0, -7.0),
        QPointF(33.0, 2.5),
        QPointF(12.0, -14.0)
    };

    for (const QPointF& obstacle : obstacles) {
        for (int i = 0; i < 52; ++i) {
            const double jitterX = (QRandomGenerator::global()->bounded(1000) / 1000.0 - 0.5) * 2.2;
            const double jitterY = (QRandomGenerator::global()->bounded(1000) / 1000.0 - 0.5) * 2.2;
            LidarPoint point;
            point.x = static_cast<float>(obstacle.x() + jitterX);
            point.y = static_cast<float>(obstacle.y() + jitterY);
            point.intensity = 0.7f;
            points.append(point);
        }
    }

    appendInjectedObstaclePoints(points);

    for (int i = 0; i < 80; ++i) {
        LidarPoint noise;
        noise.x = static_cast<float>(QRandomGenerator::global()->bounded(4000) / 100.0);
        noise.y = static_cast<float>(QRandomGenerator::global()->bounded(8000) / 100.0 - 40.0);
        noise.intensity = 0.2f;
        points.append(noise);
    }

    QVector<LidarCluster> clusters = clusterPoints(points);
    DataManager::instance().setLidarData(points, clusters);
}

void MainWindow::appendInjectedObstaclePoints(QVector<LidarPoint>& points) const {
    const VehicleState vehicle = DataManager::instance().vehicleState();
    const double cosYaw = std::cos(vehicle.yaw);
    const double sinYaw = std::sin(vehicle.yaw);
    for (const QPointF& worldObstacle : injectedObstacles_) {
        const double dx = worldObstacle.x() - vehicle.x;
        const double dy = worldObstacle.y() - vehicle.y;
        const double localX = cosYaw * dx + sinYaw * dy;
        const double localY = -sinYaw * dx + cosYaw * dy;

        if (localX < -5.0 || localX > 45.0 || std::abs(localY) > 45.0) {
            continue;
        }

        for (int i = 0; i < 70; ++i) {
            const double jitterX = (QRandomGenerator::global()->bounded(1000) / 1000.0 - 0.5) * 1.8;
            const double jitterY = (QRandomGenerator::global()->bounded(1000) / 1000.0 - 0.5) * 1.8;
            LidarPoint point;
            point.x = static_cast<float>(localX + jitterX);
            point.y = static_cast<float>(localY + jitterY);
            point.intensity = 1.0f;
            points.append(point);
        }
    }
}

QVector<LidarCluster> MainWindow::clusterPoints(QVector<LidarPoint>& points) {
    QVector<LidarCluster> clusters;
    QVector<bool> visited(points.size(), false);
    const float threshold2 = 2.2f * 2.2f;
    const int minClusterSize = 8;

    for (int i = 0; i < points.size(); ++i) {
        if (visited[i]) {
            continue;
        }

        QVector<int> queue;
        QVector<int> members;
        queue.append(i);
        visited[i] = true;

        for (int cursor = 0; cursor < queue.size(); ++cursor) {
            const int current = queue[cursor];
            members.append(current);
            for (int j = 0; j < points.size(); ++j) {
                if (visited[j]) {
                    continue;
                }
                const float dx = points[current].x - points[j].x;
                const float dy = points[current].y - points[j].y;
                if (dx * dx + dy * dy <= threshold2) {
                    visited[j] = true;
                    queue.append(j);
                }
            }
        }

        if (members.size() < minClusterSize) {
            continue;
        }

        QRectF bounds(QPointF(points[members.first()].x, points[members.first()].y), QSizeF(0, 0));
        QPointF sum(0, 0);
        const int clusterId = clusters.size();
        for (int index : members) {
            points[index].clusterId = clusterId;
            const QPointF point(points[index].x, points[index].y);
            bounds = bounds.united(QRectF(point, QSizeF(0.1, 0.1)));
            sum += point;
        }

        LidarCluster cluster;
        cluster.center = sum / members.size();
        cluster.bounds = bounds;
        cluster.pointCount = members.size();
        clusters.append(cluster);
    }

    return clusters;
}

QVector<QPointF> MainWindow::offsetPathToRightLane(const QVector<QPointF>& centerPath) const {
    if (centerPath.size() < 2) {
        return centerPath;
    }

    QVector<QPointF> result;
    result.reserve(centerPath.size());
    const double rightLaneOffset = 1.75;

    for (int i = 0; i < centerPath.size(); ++i) {
        QPointF tangent;
        if (i == 0) {
            tangent = centerPath[1] - centerPath[0];
        } else if (i == centerPath.size() - 1) {
            tangent = centerPath[i] - centerPath[i - 1];
        } else {
            tangent = centerPath[i + 1] - centerPath[i - 1];
        }

        const double len = std::hypot(tangent.x(), tangent.y());
        if (len < 1e-6) {
            result.append(centerPath[i]);
            continue;
        }

        const QPointF rightNormal(tangent.y() / len, -tangent.x() / len);
        result.append(centerPath[i] + rightNormal * rightLaneOffset);
    }

    return result;
}

void MainWindow::updatePathFollowing() {
    const QVector<QPointF> path = DataManager::instance().navigationPath();
    if (path.size() < 2) {
        routeFollowingEnabled_ = false;
        return;
    }

    const VehicleState state = vehicle_.state();
    const QPointF position(state.x, state.y);

    const int segmentCount = path.size() - 1;
    const int searchStart = qBound(0, routePathIndex_ - 1, segmentCount - 1);
    const int searchEnd = qMin(segmentCount - 1, searchStart + 8);
    int nearestSegment = searchStart;
    double nearestT = 0.0;
    double nearestDistance2 = std::numeric_limits<double>::infinity();

    for (int i = searchStart; i <= searchEnd; ++i) {
        const QPointF a = path[i];
        const QPointF b = path[i + 1];
        const QPointF ab = b - a;
        const double len2 = ab.x() * ab.x() + ab.y() * ab.y();
        if (len2 < 1e-6) {
            continue;
        }

        const QPointF ap = position - a;
        const double t = qBound(0.0, (ap.x() * ab.x() + ap.y() * ab.y()) / len2, 1.0);
        const QPointF projected = a + ab * t;
        const double d2 = (projected.x() - position.x()) * (projected.x() - position.x())
                        + (projected.y() - position.y()) * (projected.y() - position.y());
        if (d2 < nearestDistance2) {
            nearestDistance2 = d2;
            nearestSegment = i;
            nearestT = t;
        }
    }
    routePathIndex_ = qMax(routePathIndex_, nearestSegment + 1);

    double remainingLookahead = 6.0;
    QPointF target = path.last();
    QPointF current = path[nearestSegment] + (path[nearestSegment + 1] - path[nearestSegment]) * nearestT;
    for (int i = nearestSegment; i < path.size() - 1; ++i) {
        const QPointF segmentEnd = path[i + 1];
        const double segLen = std::hypot(segmentEnd.x() - current.x(), segmentEnd.y() - current.y());
        if (segLen >= remainingLookahead && segLen > 1e-6) {
            const double ratio = remainingLookahead / segLen;
            target = current + (segmentEnd - current) * ratio;
            break;
        }
        remainingLookahead -= segLen;
        current = segmentEnd;
    }

    const double dx = target.x() - state.x;
    const double dy = target.y() - state.y;
    const double targetYaw = std::atan2(dy, dx);
    double yawError = targetYaw - state.yaw;
    constexpr double pi = 3.14159265358979323846;
    while (yawError > pi) yawError -= 2.0 * pi;
    while (yawError < -pi) yawError += 2.0 * pi;

    if (std::abs(yawError) > 1.15) {
        vehicle_.setControl(0.0, yawError > 0.0 ? 1.25 : -1.25);
        vehicle_.rotateYaw(yawError * 0.24);
        return;
    }

    const double steering = qBound(-1.15, yawError * 1.65, 1.15);
    double speed = qMax(1.8, 4.5 * (1.0 - std::min(std::abs(yawError), 1.2) / 1.8));

    const QPointF goal = path.last();
    const double goalDx = goal.x() - position.x();
    const double goalDy = goal.y() - position.y();
    if (goalDx * goalDx + goalDy * goalDy < 9.0) {
        speed = 0.0;
        appendLog(QString("route arrived: target %1/%2")
            .arg(activeGoalIndex_ + 1)
            .arg(navigationGoals_.size()));
        logDatabase_.insertEvent(
            "ROUTE_ARRIVED",
            QString("target=%1 goals=%2").arg(activeGoalIndex_ + 1).arg(navigationGoals_.size()));

        ++activeGoalIndex_;
        if (activeGoalIndex_ < navigationGoals_.size()) {
            activateGoalRoute();
            vehicle_.setControl(0.0, state.steering);
            return;
        } else {
            routeFollowingEnabled_ = false;
            DataManager::instance().setNavigationPath({}, navigationGoals_);
            appendLog("all queued goals reached");
            logDatabase_.insertEvent("ROUTE_COMPLETE", "all queued goals reached");
        }
    }

    vehicle_.setControl(speed, steering);
}

bool MainWindow::hasBlockingObstacle() const {
    const VehicleState vehicle = DataManager::instance().vehicleState();
    const double cosYaw = std::cos(vehicle.yaw);
    const double sinYaw = std::sin(vehicle.yaw);

    for (const QPointF& obstacle : injectedObstacles_) {
        const double dx = obstacle.x() - vehicle.x;
        const double dy = obstacle.y() - vehicle.y;
        const double localX = cosYaw * dx + sinYaw * dy;
        const double localY = -sinYaw * dx + cosYaw * dy;

        if (localX > 0.0 && localX < config_.autoBrakeDistance() && std::abs(localY) < config_.autoBrakeWidth()) {
            return true;
        }
    }

    return false;
}
