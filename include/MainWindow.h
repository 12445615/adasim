#pragma once

#include "AppConfig.h"
#include "CanBusManager.h"
#include "OpenDriveMap.h"
#include "ReplayManager.h"
#include "RoutePlanner.h"
#include "TcpControlServer.h"
#include "UdpLidarReceiver.h"
#include "VehicleModel.h"
#include "LogDatabase.h"

#include <QElapsedTimer>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QSlider>
#include <QTextEdit>
#include <QTimer>

class LidarWidget;
class SimulationWidget;

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);

private slots:
    void startSimulation();
    void pauseSimulation();
    void resetSimulation();
    void tick();
    void applyControl(double speed, double steering);
    void appendLog(const QString& message);
    void replayChanged(int value);
    void injectVirtualObstacle(const QPointF& worldPoint);
    void planRouteToGoal(const QPointF& worldPoint);
    void clearVirtualObstacles();
    void applyExternalLidar(const QVector<LidarPoint>& points);
    void chooseOpenDriveMap();

private:
    void buildUi();
    void loadOpenDriveMap(const QString& selectedPath = QString());
    void updateTelemetry(double dt);
    void generateLidar();
    void appendInjectedObstaclePoints(QVector<LidarPoint>& points) const;
    QVector<LidarCluster> clusterPoints(QVector<LidarPoint>& points);
    QVector<QPointF> offsetPathToRightLane(const QVector<QPointF>& centerPath) const;
    void activateGoalRoute();
    void updatePathFollowing();
    bool hasBlockingObstacle() const;

    AppConfig config_;
    OpenDriveMap openDriveMap_;
    RoutePlanner routePlanner_;
    VehicleModel vehicle_;
    CanBusManager canBusManager_;
    ReplayManager replay_;
    TcpControlServer tcpServer_;
    UdpLidarReceiver udpLidarReceiver_;
    LogDatabase logDatabase_;

    SimulationWidget* simulationWidget_ = nullptr;
    LidarWidget* lidarWidget_ = nullptr;
    QLabel* speedLabel_ = nullptr;
    QLabel* mileageLabel_ = nullptr;
    QLabel* fpsLabel_ = nullptr;
    QLabel* pointCountLabel_ = nullptr;
    QLabel* clusterCountLabel_ = nullptr;
    QLabel* packetCountLabel_ = nullptr;
    QLabel* replayCountLabel_ = nullptr;
    QLabel* statusLabel_ = nullptr;
    QTextEdit* logEdit_ = nullptr;
    QSlider* replaySlider_ = nullptr;
    QPushButton* startButton_ = nullptr;
    QPushButton* pauseButton_ = nullptr;

    QTimer timer_;
    QElapsedTimer elapsed_;
    bool running_ = false;
    bool replaying_ = false;
    bool braking_ = false;
    bool externalLidarActive_ = false;
    bool routeFollowingEnabled_ = false;
    qint64 lastExternalLidarMs_ = 0;
    qint64 lastStateLogMs_ = 0;
    qint64 lastCanTxMs_ = 0;
    int controlPacketCount_ = 0;
    int routePathIndex_ = 1;
    int activeGoalIndex_ = 0;
    QVector<QPointF> injectedObstacles_;
    QVector<QPointF> navigationGoals_;
};
