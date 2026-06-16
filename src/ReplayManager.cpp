#include "ReplayManager.h"

ReplayManager::ReplayManager(QObject* parent)
    : QObject(parent) {}

void ReplayManager::record(const FrameSnapshot& snapshot) {
    if (lastRecordTimestampMs_ > 0 && snapshot.timestampMs - lastRecordTimestampMs_ < recordIntervalMs_) {
        return;
    }

    lastRecordTimestampMs_ = snapshot.timestampMs;
    buffer_.append(snapshot);
    if (buffer_.size() > maxFrames_) {
        buffer_.remove(0, buffer_.size() - maxFrames_);
    }
}

int ReplayManager::size() const {
    return buffer_.size();
}

int ReplayManager::maxSize() const {
    return maxFrames_;
}

bool ReplayManager::restoreByPercent(int percent) {
    if (buffer_.isEmpty()) {
        return false;
    }

    const int clamped = qBound(0, percent, 100);
    const int index = (buffer_.size() - 1) * clamped / 100;
    DataManager::instance().restoreSnapshot(buffer_.at(index));
    emit restored();
    return true;
}

void ReplayManager::clear() {
    buffer_.clear();
    lastRecordTimestampMs_ = 0;
}
