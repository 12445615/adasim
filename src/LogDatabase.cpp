#include "LogDatabase.h"

#include <QDateTime>
#include <QSqlError>
#include <QSqlQuery>
#include <QVariant>

namespace {
bool columnExists(QSqlDatabase& db, const QString& table, const QString& column) {
    QSqlQuery query(db);
    if (!query.exec(QString("PRAGMA table_info(%1)").arg(table))) {
        return false;
    }

    while (query.next()) {
        if (query.value(1).toString() == column) {
            return true;
        }
    }
    return false;
}
}

bool LogDatabase::open(const QString& path) {
    if (QSqlDatabase::contains("adasim_log")) {
        db_ = QSqlDatabase::database("adasim_log");
    } else {
        db_ = QSqlDatabase::addDatabase("QSQLITE", "adasim_log");
    }

    db_.setDatabaseName(path);
    if (!db_.open()) {
        return false;
    }
    return initTables();
}

void LogDatabase::close() {
    if (db_.isOpen()) {
        db_.close();
    }
}

bool LogDatabase::isOpen() const {
    return db_.isOpen();
}

bool LogDatabase::initTables() {
    QSqlQuery query(db_);
    const bool vehicleOk = query.exec(
        "CREATE TABLE IF NOT EXISTS vehicle_state ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "timestamp_ms INTEGER,"
        "x REAL,"
        "y REAL,"
        "yaw REAL,"
        "speed REAL,"
        "steering REAL,"
        "mileage REAL,"
        "point_count INTEGER,"
        "cluster_count INTEGER,"
        "braking INTEGER)");

    const bool controlOk = query.exec(
        "CREATE TABLE IF NOT EXISTS control_packet ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "timestamp_ms INTEGER,"
        "speed REAL,"
        "steering REAL,"
        "source TEXT,"
        "applied INTEGER)");

    const bool eventOk = query.exec(
        "CREATE TABLE IF NOT EXISTS event_log ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "timestamp_ms INTEGER,"
        "type TEXT,"
        "detail TEXT)");

    if (!columnExists(db_, "control_packet", "source")) {
        QSqlQuery alter(db_);
        alter.exec("ALTER TABLE control_packet ADD COLUMN source TEXT");
    }
    if (!columnExists(db_, "control_packet", "applied")) {
        QSqlQuery alter(db_);
        alter.exec("ALTER TABLE control_packet ADD COLUMN applied INTEGER DEFAULT 1");
    }

    return vehicleOk && controlOk && eventOk;
}

void LogDatabase::insertVehicleState(const VehicleState& state, int pointCount, int clusterCount, bool braking) {
    if (!db_.isOpen()) {
        return;
    }

    QSqlQuery query(db_);
    query.prepare(
        "INSERT INTO vehicle_state "
        "(timestamp_ms,x,y,yaw,speed,steering,mileage,point_count,cluster_count,braking) "
        "VALUES (?,?,?,?,?,?,?,?,?,?)");
    query.addBindValue(QDateTime::currentMSecsSinceEpoch());
    query.addBindValue(state.x);
    query.addBindValue(state.y);
    query.addBindValue(state.yaw);
    query.addBindValue(state.speed);
    query.addBindValue(state.steering);
    query.addBindValue(state.mileage);
    query.addBindValue(pointCount);
    query.addBindValue(clusterCount);
    query.addBindValue(braking ? 1 : 0);
    query.exec();
}

void LogDatabase::insertControl(double speed, double steering, const QString& source, bool applied) {
    if (!db_.isOpen()) {
        return;
    }

    QSqlQuery query(db_);
    query.prepare("INSERT INTO control_packet (timestamp_ms,speed,steering,source,applied) VALUES (?,?,?,?,?)");
    query.addBindValue(QDateTime::currentMSecsSinceEpoch());
    query.addBindValue(speed);
    query.addBindValue(steering);
    query.addBindValue(source);
    query.addBindValue(applied ? 1 : 0);
    query.exec();
}

void LogDatabase::insertEvent(const QString& type, const QString& detail) {
    if (!db_.isOpen()) {
        return;
    }

    QSqlQuery query(db_);
    query.prepare("INSERT INTO event_log (timestamp_ms,type,detail) VALUES (?,?,?)");
    query.addBindValue(QDateTime::currentMSecsSinceEpoch());
    query.addBindValue(type);
    query.addBindValue(detail);
    query.exec();
}
