#include "VehicleModel.h"

#include <cmath>

void VehicleModel::reset() {
    state_ = VehicleState{};
}

void VehicleModel::setPose(double x, double y, double yaw) {
    state_.x = x;
    state_.y = y;
    state_.yaw = yaw;
    state_.speed = 0.0;
    state_.steering = 0.0;
    state_.mileage = 0.0;
}

void VehicleModel::setControl(double speed, double steering) {
    state_.speed = qBound(-8.0, speed, 25.0);
    state_.steering = qBound(-1.25, steering, 1.25);
}

void VehicleModel::rotateYaw(double deltaYaw) {
    state_.yaw += qBound(-0.22, deltaYaw, 0.22);
}

void VehicleModel::update(double dt) {
    state_.x += state_.speed * std::cos(state_.yaw) * dt;
    state_.y += state_.speed * std::sin(state_.yaw) * dt;
    state_.yaw += state_.speed / wheelBase_ * std::tan(state_.steering) * dt;
    state_.mileage += std::abs(state_.speed) * dt;
}

VehicleState VehicleModel::state() const {
    return state_;
}
