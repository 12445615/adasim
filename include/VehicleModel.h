#pragma once

#include <QtGlobal>

struct VehicleState {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double speed = 5.0;
    double steering = 0.0;
    double mileage = 0.0;
};

class VehicleModel {
public:
    void reset();
    void setPose(double x, double y, double yaw);
    void setControl(double speed, double steering);
    void rotateYaw(double deltaYaw);
    void update(double dt);
    VehicleState state() const;

private:
    VehicleState state_;
    double wheelBase_ = 1.35;
};
