#pragma once

#include "DataManager.h"

#include <QObject>
#include <QVector>

class ReplayManager : public QObject {
    Q_OBJECT

public:
    explicit ReplayManager(QObject* parent = nullptr);

    void record(const FrameSnapshot& snapshot);
    int size() const;
    int maxSize() const;
    bool restoreByPercent(int percent);
    void clear();

signals:
    void restored();

private:
    QVector<FrameSnapshot> buffer_;
    qint64 lastRecordTimestampMs_ = 0;
    int maxFrames_ = 1800;
    int recordIntervalMs_ = 100;
};
