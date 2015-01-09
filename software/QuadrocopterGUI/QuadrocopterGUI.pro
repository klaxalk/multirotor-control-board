#-------------------------------------------------
#
# Project created by QtCreator 2014-12-28T16:44:36
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = QuadrocopterGUI
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    quadro.cpp \
    qcustomplot.cpp \
    commands.cpp \
    const.cpp \
    openLog.cpp \
    packets.cpp \
    receive.cpp \
    send.cpp \
    serial.cpp \
    serialLink.cpp \
    XBeeComm.cpp \
    sendtrajectory.cpp

HEADERS  += mainwindow.h \
    quadro.h \
    qcustomplot.h \
    commands.h \
    const.h \
    defines.h \
    openLog.h \
    packets.h \
    receive.h \
    send.h \
    serialLink.h \
    XBeeComm.h \
    serial.h \
    sendtrajectory.h

FORMS    += mainwindow.ui \
    quadro.ui \
    sendtrajectory.ui
