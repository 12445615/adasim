QT += core gui network widgets sql xml

CONFIG += c++17
CONFIG -= app_bundle

TARGET = ADASim
TEMPLATE = app

INCLUDEPATH += include

HEADERS += \
    include/CanBusManager.h \
    include/AppConfig.h \
    include/DataManager.h \
    include/LidarWidget.h \
    include/LogDatabase.h \
    include/MainWindow.h \
    include/OpenDriveMap.h \
    include/RoutePlanner.h \
    include/ReplayManager.h \
    include/SimulationWidget.h \
    include/TcpControlServer.h \
    include/UdpLidarReceiver.h \
    include/VehicleModel.h

SOURCES += \
    src/CanBusManager.cpp \
    src/AppConfig.cpp \
    src/DataManager.cpp \
    src/LidarWidget.cpp \
    src/LogDatabase.cpp \
    src/MainWindow.cpp \
    src/OpenDriveMap.cpp \
    src/RoutePlanner.cpp \
    src/ReplayManager.cpp \
    src/SimulationWidget.cpp \
    src/TcpControlServer.cpp \
    src/UdpLidarReceiver.cpp \
    src/VehicleModel.cpp \
    src/main.cpp

win32 {
    copy_config.commands = if not exist $$shell_path($$OUT_PWD/config) mkdir $$shell_path($$OUT_PWD/config) && $(COPY_FILE) $$shell_path($$PWD/config/adasim.json) $$shell_path($$OUT_PWD/config/adasim.json) && if not exist $$shell_path($$OUT_PWD/maps) mkdir $$shell_path($$OUT_PWD/maps) && $(COPY_FILE) $$shell_path($$PWD/maps/*.xodr) $$shell_path($$OUT_PWD/maps)
} else {
    copy_config.commands = $(MKDIR) $$shell_path($$OUT_PWD/config) && $(COPY_FILE) $$shell_path($$PWD/config/adasim.json) $$shell_path($$OUT_PWD/config/adasim.json) && $(MKDIR) $$shell_path($$OUT_PWD/maps) && $(COPY_FILE) $$shell_path($$PWD/maps/*.xodr) $$shell_path($$OUT_PWD/maps)
}
QMAKE_EXTRA_TARGETS += copy_config
POST_TARGETDEPS += copy_config
