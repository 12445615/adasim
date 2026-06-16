#include "RoutePlanner.h"

#include <QSet>
#include <QtMath>
#include <algorithm>
#include <limits>

namespace {
constexpr double kMergeDistance2 = 0.25;
constexpr double kEpsilon = 1e-6;

double distance2(const QPointF& a, const QPointF& b) {
    const double dx = a.x() - b.x();
    const double dy = a.y() - b.y();
    return dx * dx + dy * dy;
}

double distance(const QPointF& a, const QPointF& b) {
    return qSqrt(distance2(a, b));
}

double cross(const QPointF& a, const QPointF& b) {
    return a.x() * b.y() - a.y() * b.x();
}

QPointF interpolate(const QLineF& line, double t) {
    return line.p1() + (line.p2() - line.p1()) * t;
}

double parameterOnSegment(const QLineF& line, const QPointF& p) {
    const QPointF v = line.p2() - line.p1();
    const double len2 = v.x() * v.x() + v.y() * v.y();
    if (len2 < kEpsilon) {
        return 0.0;
    }
    const QPointF w = p - line.p1();
    return qBound(0.0, (w.x() * v.x() + w.y() * v.y()) / len2, 1.0);
}

QPointF projectToSegment(const QLineF& line, const QPointF& p) {
    return interpolate(line, parameterOnSegment(line, p));
}

bool segmentIntersectionParameter(const QLineF& a, const QLineF& b, double& ta, double& tb) {
    const QPointF p = a.p1();
    const QPointF r = a.p2() - a.p1();
    const QPointF q = b.p1();
    const QPointF s = b.p2() - b.p1();
    const double rxs = cross(r, s);
    if (qAbs(rxs) < kEpsilon) {
        return false;
    }

    const QPointF qp = q - p;
    ta = cross(qp, s) / rxs;
    tb = cross(qp, r) / rxs;
    return ta > kEpsilon && ta < 1.0 - kEpsilon && tb > kEpsilon && tb < 1.0 - kEpsilon;
}
}

QVector<QPointF> RoutePlanner::plan(const QVector<QLineF>& roadSegments, const QPointF& start, const QPointF& goal) const {
    if (roadSegments.isEmpty()) {
        return {start, goal};
    }

    QVector<QVector<double>> splitParams(roadSegments.size());
    for (int i = 0; i < roadSegments.size(); ++i) {
        splitParams[i].append(0.0);
        splitParams[i].append(1.0);
    }

    for (int i = 0; i < roadSegments.size(); ++i) {
        for (int j = i + 1; j < roadSegments.size(); ++j) {
            double ti = 0.0;
            double tj = 0.0;
            if (segmentIntersectionParameter(roadSegments[i], roadSegments[j], ti, tj)) {
                splitParams[i].append(ti);
                splitParams[j].append(tj);
            }
        }
    }

    int startSegment = 0;
    int goalSegment = 0;
    double bestStart = std::numeric_limits<double>::infinity();
    double bestGoal = std::numeric_limits<double>::infinity();
    QPointF startOnRoad;
    QPointF goalOnRoad;

    for (int i = 0; i < roadSegments.size(); ++i) {
        const QPointF s = projectToSegment(roadSegments[i], start);
        const QPointF g = projectToSegment(roadSegments[i], goal);
        const double sd = distance2(s, start);
        const double gd = distance2(g, goal);
        if (sd < bestStart) {
            bestStart = sd;
            startSegment = i;
            startOnRoad = s;
        }
        if (gd < bestGoal) {
            bestGoal = gd;
            goalSegment = i;
            goalOnRoad = g;
        }
    }

    splitParams[startSegment].append(parameterOnSegment(roadSegments[startSegment], startOnRoad));
    splitParams[goalSegment].append(parameterOnSegment(roadSegments[goalSegment], goalOnRoad));

    QVector<QPointF> nodes;
    QVector<QVector<QPair<int, double>>> graph;

    auto ensureGraphSize = [&graph](int size) {
        while (graph.size() < size) {
            graph.append(QVector<QPair<int, double>>());
        }
    };

    auto addEdge = [&graph, &ensureGraphSize](int a, int b, double w) {
        if (a == b || w < kEpsilon) {
            return;
        }
        ensureGraphSize(std::max(a, b) + 1);
        graph[a].append(qMakePair(b, w));
        graph[b].append(qMakePair(a, w));
    };

    for (int i = 0; i < roadSegments.size(); ++i) {
        auto& params = splitParams[i];
        std::sort(params.begin(), params.end());
        params.erase(std::unique(params.begin(), params.end(), [](double a, double b) {
            return qAbs(a - b) < 1e-5;
        }), params.end());

        int previousNode = -1;
        QPointF previousPoint;
        for (double t : params) {
            const QPointF point = interpolate(roadSegments[i], t);
            const int node = findOrAddNode(nodes, point);
            ensureGraphSize(nodes.size());
            if (previousNode >= 0) {
                addEdge(previousNode, node, distance(previousPoint, point));
            }
            previousNode = node;
            previousPoint = point;
        }
    }

    const int startNode = nearestNode(nodes, startOnRoad);
    const int goalNode = nearestNode(nodes, goalOnRoad);

    QVector<double> dist(nodes.size(), std::numeric_limits<double>::infinity());
    QVector<int> prev(nodes.size(), -1);
    QVector<bool> used(nodes.size(), false);
    dist[startNode] = 0.0;

    for (int iter = 0; iter < nodes.size(); ++iter) {
        int current = -1;
        double best = std::numeric_limits<double>::infinity();
        for (int i = 0; i < nodes.size(); ++i) {
            if (!used[i] && dist[i] < best) {
                best = dist[i];
                current = i;
            }
        }

        if (current < 0 || current == goalNode) {
            break;
        }

        used[current] = true;
        for (const auto& edge : graph[current]) {
            const int next = edge.first;
            const double candidate = dist[current] + edge.second;
            if (candidate < dist[next]) {
                dist[next] = candidate;
                prev[next] = current;
            }
        }
    }

    QVector<QPointF> path;
    path.append(startOnRoad);
    if (prev[goalNode] >= 0 || goalNode == startNode) {
        QVector<QPointF> reversed;
        for (int at = goalNode; at >= 0; at = prev[at]) {
            reversed.append(nodes[at]);
            if (at == startNode) {
                break;
            }
        }
        std::reverse(reversed.begin(), reversed.end());
        for (const QPointF& point : reversed) {
            if (path.isEmpty() || distance2(path.last(), point) > kMergeDistance2) {
                path.append(point);
            }
        }
    }
    if (path.isEmpty() || distance2(path.last(), goalOnRoad) > kMergeDistance2) {
        path.append(goalOnRoad);
    }
    return path;
}

int RoutePlanner::nearestNode(const QVector<QPointF>& nodes, const QPointF& point) const {
    int bestIndex = 0;
    double best = std::numeric_limits<double>::infinity();
    for (int i = 0; i < nodes.size(); ++i) {
        const double d2 = distance2(nodes[i], point);
        if (d2 < best) {
            best = d2;
            bestIndex = i;
        }
    }
    return bestIndex;
}

int RoutePlanner::findOrAddNode(QVector<QPointF>& nodes, const QPointF& point) const {
    for (int i = 0; i < nodes.size(); ++i) {
        if (distance2(nodes[i], point) <= kMergeDistance2) {
            return i;
        }
    }
    nodes.append(point);
    return nodes.size() - 1;
}
