#include "CanBusManager.h"

#include <QtGlobal>

#ifdef Q_OS_LINUX
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

CanBusManager::CanBusManager(QObject* parent)
    : QObject(parent) {}

CanBusManager::~CanBusManager() {
    close();
}

bool CanBusManager::open(const QString& interfaceName) {
    interfaceName_ = interfaceName;

#ifdef Q_OS_LINUX
    close();
    socketFd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socketFd_ < 0) {
        emit logMessage("SocketCAN open failed: socket()");
        return false;
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    const QByteArray name = interfaceName.toLocal8Bit();
    std::strncpy(ifr.ifr_name, name.constData(), IFNAMSIZ - 1);

    if (ioctl(socketFd_, SIOCGIFINDEX, &ifr) < 0) {
        emit logMessage(QString("SocketCAN interface not ready: %1").arg(interfaceName));
        close();
        return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socketFd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        emit logMessage(QString("SocketCAN bind failed: %1").arg(interfaceName));
        close();
        return false;
    }

    opened_ = true;
    emit logMessage(QString("SocketCAN connected: %1").arg(interfaceName));
    return true;
#else
    opened_ = false;
    emit logMessage(QString("SocketCAN disabled on this platform, target=%1").arg(interfaceName));
    return false;
#endif
}

void CanBusManager::close() {
#ifdef Q_OS_LINUX
    if (socketFd_ >= 0) {
        ::close(socketFd_);
        socketFd_ = -1;
    }
#endif
    opened_ = false;
}

bool CanBusManager::isOpen() const {
    return opened_;
}

bool CanBusManager::sendControl(double speed, double steering, bool braking) {
#ifdef Q_OS_LINUX
    if (!opened_ || socketFd_ < 0) {
        return false;
    }

    const qint16 speedCmps = static_cast<qint16>(speed * 100.0);
    const qint16 steeringMrad = static_cast<qint16>(steering * 1000.0);

    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x321;
    frame.can_dlc = 8;
    frame.data[0] = static_cast<unsigned char>((speedCmps >> 8) & 0xff);
    frame.data[1] = static_cast<unsigned char>(speedCmps & 0xff);
    frame.data[2] = static_cast<unsigned char>((steeringMrad >> 8) & 0xff);
    frame.data[3] = static_cast<unsigned char>(steeringMrad & 0xff);
    frame.data[4] = braking ? 1 : 0;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;

    const int written = write(socketFd_, &frame, sizeof(frame));
    if (written != static_cast<int>(sizeof(frame))) {
        emit logMessage("SocketCAN TX failed");
        return false;
    }
    return true;
#else
    Q_UNUSED(speed)
    Q_UNUSED(steering)
    Q_UNUSED(braking)
    return false;
#endif
}
