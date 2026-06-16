#include "OpenDriveMap.h"

#include <QDomDocument>
#include <QFile>
#include <QtMath>

bool OpenDriveMap::load(const QString& path) {
    centerLines_.clear();
    laneLines_.clear();
    errorString_.clear();

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        errorString_ = QString("OpenDRIVE open failed: %1").arg(path);
        return false;
    }

    QDomDocument doc;
    QString parseError;
    int errorLine = 0;
    int errorColumn = 0;
    if (!doc.setContent(&file, &parseError, &errorLine, &errorColumn)) {
        errorString_ = QString("OpenDRIVE parse failed at %1:%2 %3")
            .arg(errorLine)
            .arg(errorColumn)
            .arg(parseError);
        return false;
    }

    const QDomNodeList geometries = doc.elementsByTagName("geometry");
    for (int i = 0; i < geometries.size(); ++i) {
        const QDomElement geometry = geometries.at(i).toElement();
        const double x = geometry.attribute("x").toDouble();
        const double y = geometry.attribute("y").toDouble();
        const double heading = geometry.attribute("hdg").toDouble();
        const double length = geometry.attribute("length").toDouble();

        if (!geometry.firstChildElement("line").isNull()) {
            appendLine(x, y, heading, length);
            continue;
        }

        const QDomElement arc = geometry.firstChildElement("arc");
        if (!arc.isNull()) {
            appendArc(x, y, heading, length, arc.attribute("curvature").toDouble());
        }
    }

    rebuildLaneLines();
    if (centerLines_.isEmpty()) {
        errorString_ = "OpenDRIVE contains no supported line/arc geometry";
        return false;
    }

    return true;
}

bool OpenDriveMap::isLoaded() const {
    return !centerLines_.isEmpty();
}

QString OpenDriveMap::errorString() const {
    return errorString_;
}

QVector<QLineF> OpenDriveMap::centerLines() const {
    return centerLines_;
}

QVector<QLineF> OpenDriveMap::laneLines() const {
    return laneLines_;
}

void OpenDriveMap::appendLine(double x, double y, double heading, double length) {
    const QPointF start(x, y);
    const QPointF end(x + qCos(heading) * length, y + qSin(heading) * length);
    centerLines_.append(QLineF(start, end));
}

void OpenDriveMap::appendArc(double x, double y, double heading, double length, double curvature) {
    if (qFuzzyIsNull(curvature)) {
        appendLine(x, y, heading, length);
        return;
    }

    const int segments = qMax(8, static_cast<int>(length / 3.0));
    QPointF previous(x, y);
    for (int i = 1; i <= segments; ++i) {
        const double s = length * i / segments;
        const double theta = heading + curvature * s;
        const double nx = x + (qSin(theta) - qSin(heading)) / curvature;
        const double ny = y - (qCos(theta) - qCos(heading)) / curvature;
        const QPointF current(nx, ny);
        centerLines_.append(QLineF(previous, current));
        previous = current;
    }
}

void OpenDriveMap::rebuildLaneLines() {
    laneLines_.clear();
    for (const QLineF& center : centerLines_) {
        const double dx = center.dx();
        const double dy = center.dy();
        const double len = qSqrt(dx * dx + dy * dy);
        if (len < 1e-6) {
            continue;
        }

        const QPointF normal(-dy / len * laneHalfWidth_, dx / len * laneHalfWidth_);
        laneLines_.append(QLineF(center.p1() + normal, center.p2() + normal));
        laneLines_.append(QLineF(center.p1() - normal, center.p2() - normal));
    }
}
