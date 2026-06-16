#pragma once

#include "VehicleModel.h"

#include <QTimer>
#include <QWidget>

class SimulationWidget : public QWidget {
    Q_OBJECT

public:
    explicit SimulationWidget(QWidget* parent = nullptr);

signals:
    void virtualObstacleRequested(const QPointF& worldPoint);
    void navigationGoalRequested(const QPointF& worldPoint);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;

private:
    void emitPendingSingleClick();
    QPointF worldToScreen(const QPointF& world) const;
    QPointF screenToWorld(const QPointF& screen) const;
    void updateCamera(const VehicleState& vehicle);
    void drawBackground(QPainter& painter);
    void drawMap(QPainter& painter);
    void drawNavigationPath(QPainter& painter);
    void drawTrajectory(QPainter& painter);
    void drawVirtualObstacles(QPainter& painter);
    void drawVehicle(QPainter& painter);

    QPointF cameraCenter_;
    bool cameraReady_ = false;
    QTimer singleClickTimer_;
    QPointF pendingSingleClickWorld_;
};
