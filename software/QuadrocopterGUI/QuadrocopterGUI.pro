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
    qcustomplot.cpp

HEADERS  += mainwindow.h \
    quadro.h \
    qcustomplot.h

FORMS    += mainwindow.ui \
    quadro.ui
