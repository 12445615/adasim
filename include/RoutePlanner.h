#pragma once

#include <QLineF>
#include <QPointF>
#include <QVector>

class RoutePlanner {
public:
    QVector<QPointF> plan(const QVector<QLineF>& roadSegments, const QPointF& start, const QPointF& goal) const;

private:
    int nearestNode(const QVector<QPointF>& nodes, const QPointF& point) const;
    int findOrAddNode(QVector<QPointF>& nodes, const QPointF& point) const;
};
