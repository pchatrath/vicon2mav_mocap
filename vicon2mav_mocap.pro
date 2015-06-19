#-------------------------------------------------
#
# Project created by QtCreator 2015-06-10T10:00:17
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = vicon2mav_mocap
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp

unix:!macx: LIBS += -L$$PWD/vicon -lViconDataStreamSDK_CPP

HEADERS += \
    functions.h \
    vicon/client.h \
#    mavlink/checksum.h \
#    mavlink/mavlink.h \
#    mavlink/mavlink_helpers.h \
#    mavlink/mavlink_types.h \
#    mavlink/mavlink_msg_att_pos_mocap.h \
    /home/gcs/programming/mavlink_lib/c_library-master/common/mavlink.h \
    quaternions.h

OTHER_FILES += \
    README.txt \
    ../README.txt
