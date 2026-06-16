#pragma once

#include <QObject>
#include <QString>

class CanBusManager : public QObject {
    Q_OBJECT

public:
    explicit CanBusManager(QObject* parent = nullptr);
    ~CanBusManager();

    bool open(const QString& interfaceName = "vcan0");
    void close();
    bool isOpen() const;
    bool sendControl(double speed, double steering, bool braking);

signals:
    void logMessage(const QString& message);

private:
    QString interfaceName_;
    bool opened_ = false;

#ifdef Q_OS_LINUX
    int socketFd_ = -1;
#endif
};
