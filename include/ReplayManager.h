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
    int maxFrames_ = 600;
};
