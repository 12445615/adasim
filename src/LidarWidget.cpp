#include "LidarWidget.h"

#include "DataManager.h"

#include <QPainter>

LidarWidget::LidarWidget(QWidget* parent)
    : QWidget(parent) {
    setMinimumSize(320, 320);
}

void LidarWidget::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), QColor(8, 12, 16));

    const QPointF center(width() / 2.0, height() / 2.0);
    const int radius = qMin(width(), height()) / 2 - 14;

    painter.setPen(QPen(QColor(35, 52, 64), 1));
    for (int r = radius / 4; r <= radius; r += radius / 4) {
        painter.drawEllipse(center, r, r);
    }
    painter.drawLine(center.x(), 8, center.x(), height() - 8);
    painter.drawLine(8, center.y(), width() - 8, center.y());

    const QVector<LidarPoint> points = DataManager::instance().lidarPoints();
    for (const LidarPoint& point : points) {
        const QColor color = point.clusterId >= 0
            ? QColor::fromHsv((point.clusterId * 57) % 360, 190, 240)
            : QColor(130, 155, 170);
        painter.setPen(QPen(color, 3));
        painter.drawPoint(radarToScreen(point.x, point.y));
    }

    painter.setPen(QPen(QColor(235, 240, 245), 1));
    painter.drawText(12, 22, QString("LiDAR points: %1").arg(points.size()));
}

QPointF LidarWidget::radarToScreen(float x, float y) const {
    const float range = 40.0f;
    const float scale = (qMin(width(), height()) / 2.0f - 14.0f) / range;
    return QPointF(width() / 2.0f + y * scale, height() / 2.0f - x * scale);
}
