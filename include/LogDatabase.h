#pragma once

#include "VehicleModel.h"

#include <QSqlDatabase>
#include <QString>

class LogDatabase {
public:
    bool open(const QString& path);
    void close();
    bool isOpen() const;

    void insertVehicleState(const VehicleState& state, int pointCount, int clusterCount, bool braking);
    void insertControl(double speed, double steering);
    void insertEvent(const QString& type, const QString& detail);

private:
    bool initTables();

    QSqlDatabase db_;
};
