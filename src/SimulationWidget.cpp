#include "SimulationWidget.h"

#include "DataManager.h"

#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <cmath>
#include <limits>

SimulationWidget::SimulationWidget(QWidget* parent)
    : QWidget(parent) {
    setMinimumSize(760, 520);
    setAutoFillBackground(false);
    setContextMenuPolicy(Qt::NoContextMenu);
    singleClickTimer_.setSingleShot(true);
    singleClickTimer_.setInterval(220);
    connect(&singleClickTimer_, &QTimer::timeout, this, &SimulationWidget::emitPendingSingleClick);
}

void SimulationWidget::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    const VehicleState vehicle = DataManager::instance().vehicleState();
    updateCamera(vehicle);
    drawBackground(painter);
    drawMap(painter);
    drawNavigationPath(painter);
    drawTrajectory(painter);
    drawVirtualObstacles(painter);
    drawVehicle(painter);
}

void SimulationWidget::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        pendingSingleClickWorld_ = screenToWorld(event->pos());
        singleClickTimer_.start();
        return;
    }

    if (event->button() == Qt::RightButton) {
        const QPointF world = screenToWorld(event->pos());
        DataManager::instance().setLatestLog(
            QString("virtual obstacle requested at world(%1,%2)").arg(world.x(), 0, 'f', 1).arg(world.y(), 0, 'f', 1));
        emit virtualObstacleRequested(world);
        update();
    }
}

void SimulationWidget::mouseDoubleClickEvent(QMouseEvent* event) {
    if (event->button() != Qt::LeftButton) {
        return;
    }

    singleClickTimer_.stop();
    emit navigationGoalRequested(screenToWorld(event->pos()));
    update();
}

void SimulationWidget::emitPendingSingleClick() {
    DataManager::instance().setLatestLog(
        QString("virtual obstacle requested at world(%1,%2)")
            .arg(pendingSingleClickWorld_.x(), 0, 'f', 1)
            .arg(pendingSingleClickWorld_.y(), 0, 'f', 1));
    emit virtualObstacleRequested(pendingSingleClickWorld_);
    update();
}

QPointF SimulationWidget::worldToScreen(const QPointF& world) const {
    const double scale = 8.0;
    return QPointF(width() / 2.0 + (world.x() - cameraCenter_.x()) * scale,
                   height() / 2.0 - (world.y() - cameraCenter_.y()) * scale);
}

QPointF SimulationWidget::screenToWorld(const QPointF& screen) const {
    const double scale = 8.0;
    return QPointF(cameraCenter_.x() + (screen.x() - width() / 2.0) / scale,
                   cameraCenter_.y() - (screen.y() - height() / 2.0) / scale);
}

void SimulationWidget::updateCamera(const VehicleState& vehicle) {
    if (!cameraReady_) {
        cameraCenter_ = QPointF(vehicle.x, vehicle.y);
        cameraReady_ = true;
        return;
    }

    const double scale = 8.0;
    const double marginX = width() * 0.28;
    const double marginY = height() * 0.28;
    QPointF target = cameraCenter_;
    const QPointF vehicleScreen = worldToScreen(QPointF(vehicle.x, vehicle.y));

    if (vehicleScreen.x() > width() - marginX) {
        target.setX(vehicle.x - (width() / 2.0 - marginX) / scale);
    } else if (vehicleScreen.x() < marginX) {
        target.setX(vehicle.x + (width() / 2.0 - marginX) / scale);
    }

    if (vehicleScreen.y() > height() - marginY) {
        target.setY(vehicle.y + (height() / 2.0 - marginY) / scale);
    } else if (vehicleScreen.y() < marginY) {
        target.setY(vehicle.y - (height() / 2.0 - marginY) / scale);
    }

    cameraCenter_ += (target - cameraCenter_) * 0.08;
}

void SimulationWidget::drawBackground(QPainter& painter) {
    painter.fillRect(rect(), QColor(16, 20, 24));

    painter.setPen(QPen(QColor(38, 47, 55), 1));
    for (int x = width() / 2; x < width(); x += 40) painter.drawLine(x, 0, x, height());
    for (int x = width() / 2; x > 0; x -= 40) painter.drawLine(x, 0, x, height());
    for (int y = height() / 2; y < height(); y += 40) painter.drawLine(0, y, width(), y);
    for (int y = height() / 2; y > 0; y -= 40) painter.drawLine(0, y, width(), y);

    painter.setPen(QPen(QColor(70, 83, 94), 1));
    painter.drawLine(width() / 2, 0, width() / 2, height());
    painter.drawLine(0, height() / 2, width(), height() / 2);

    const VehicleState state = DataManager::instance().vehicleState();
    painter.setPen(QPen(QColor(130, 150, 165), 1));
    painter.drawText(12, 24, QString("camera dead-zone | x=%1 y=%2 yaw=%3")
        .arg(state.x, 0, 'f', 1)
        .arg(state.y, 0, 'f', 1)
        .arg(state.yaw, 0, 'f', 2));
}

void SimulationWidget::drawTrajectory(QPainter& painter) {
    const QVector<QPointF> trajectory = DataManager::instance().trajectory();
    if (trajectory.size() < 2) {
        return;
    }

    painter.setBrush(Qt::NoBrush);
    QPainterPath path;
    path.moveTo(worldToScreen(trajectory.first()));
    for (int i = 1; i < trajectory.size(); ++i) {
        path.lineTo(worldToScreen(trajectory.at(i)));
    }

    painter.setPen(QPen(QColor(44, 180, 120), 2));
    painter.drawPath(path);
}

void SimulationWidget::drawMap(QPainter& painter) {
    const QVector<QLineF> laneLines = DataManager::instance().mapLaneLines();
    const QVector<QLineF> centerLines = DataManager::instance().mapCenterLines();

    painter.setPen(QPen(QColor(80, 92, 104), 2));
    for (const QLineF& line : laneLines) {
        painter.drawLine(worldToScreen(line.p1()), worldToScreen(line.p2()));
    }

    QPen centerPen(QColor(190, 172, 84), 1);
    centerPen.setStyle(Qt::DashLine);
    painter.setPen(centerPen);
    for (const QLineF& line : centerLines) {
        painter.drawLine(worldToScreen(line.p1()), worldToScreen(line.p2()));
    }
}

void SimulationWidget::drawNavigationPath(QPainter& painter) {
    const QVector<QPointF> path = DataManager::instance().navigationPath();
    if (path.size() >= 2) {
        const VehicleState vehicle = DataManager::instance().vehicleState();
        const QPointF vehiclePosition(vehicle.x, vehicle.y);
        int nearestSegment = 0;
        double nearestT = 0.0;
        double nearestDistance2 = std::numeric_limits<double>::infinity();
        const QPointF headingVector(std::cos(vehicle.yaw), std::sin(vehicle.yaw));

        for (int i = 0; i < path.size() - 1; ++i) {
            const QPointF a = path.at(i);
            const QPointF b = path.at(i + 1);
            const QPointF ab = b - a;
            const double len2 = ab.x() * ab.x() + ab.y() * ab.y();
            if (len2 < 1e-6) {
                continue;
            }

            const QPointF ap = vehiclePosition - a;
            const double t = qBound(0.0, (ap.x() * ab.x() + ap.y() * ab.y()) / len2, 1.0);
            const QPointF projected = a + ab * t;
            const QPointF toProjected = projected - vehiclePosition;
            const bool segmentBehindVehicle =
                toProjected.x() * headingVector.x() + toProjected.y() * headingVector.y() < -2.0;
            if (segmentBehindVehicle && i > 0) {
                continue;
            }
            const double dx = projected.x() - vehiclePosition.x();
            const double dy = projected.y() - vehiclePosition.y();
            const double distance2 = dx * dx + dy * dy;
            if (distance2 < nearestDistance2) {
                nearestDistance2 = distance2;
                nearestSegment = i;
                nearestT = t;
            }
        }

        const QPointF visibleStart =
            path.at(nearestSegment) + (path.at(nearestSegment + 1) - path.at(nearestSegment)) * nearestT;

        painter.setBrush(Qt::NoBrush);
        QPainterPath painterPath;
        painterPath.moveTo(worldToScreen(visibleStart));
        for (int i = nearestSegment + 1; i < path.size(); ++i) {
            painterPath.lineTo(worldToScreen(path.at(i)));
        }
        painter.setPen(QPen(QColor(61, 164, 255), 4));
        painter.drawPath(painterPath);
    }

    if (DataManager::instance().hasNavigationGoal()) {
        const QVector<QPointF> goals = DataManager::instance().navigationGoals();
        painter.setPen(QPen(QColor(61, 164, 255), 2));
        painter.setBrush(QColor(61, 164, 255, 170));
        for (int i = 0; i < goals.size(); ++i) {
            const QPointF goal = worldToScreen(goals.at(i));
            painter.drawEllipse(goal, 10, 10);
            painter.drawText(goal + QPointF(12, -10), QString::number(i + 1));
        }
    }
}

void SimulationWidget::drawVirtualObstacles(QPainter& painter) {
    const QVector<QPointF> obstacles = DataManager::instance().virtualObstacles();
    painter.setPen(QPen(QColor(255, 95, 95), 2));
    painter.setBrush(QColor(255, 70, 70, 180));

    for (const QPointF& obstacle : obstacles) {
        const QPointF screen = worldToScreen(obstacle);
        painter.drawEllipse(screen, 9, 9);
        painter.drawLine(screen + QPointF(-13, 0), screen + QPointF(13, 0));
        painter.drawLine(screen + QPointF(0, -13), screen + QPointF(0, 13));
    }
}

void SimulationWidget::drawVehicle(QPainter& painter) {
    const VehicleState state = DataManager::instance().vehicleState();
    const QPointF center = worldToScreen(QPointF(state.x, state.y));

    painter.save();
    painter.translate(center);
    painter.rotate(-state.yaw * 180.0 / 3.141592653589793);

    QRectF body(-20, -10, 40, 20);
    painter.setPen(QPen(QColor(230, 244, 255), 2));
    painter.setBrush(QColor(48, 138, 245));
    painter.drawRoundedRect(body, 4, 4);

    painter.setPen(QPen(QColor(255, 220, 82), 3));
    painter.drawLine(0, 0, 32, 0);
    painter.restore();
}
