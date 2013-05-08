#-------------------------------------------------
#
# Project created by QtCreator 2013-04-15T21:52:24
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = wall_follow
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    robot.cpp \
    pidcorrection.cpp

HEADERS  += mainwindow.h \
    robot.h \
    pidcorrection.h

FORMS    += mainwindow.ui

INCLUDEPATH += /usr/local/Aria/include

LIBS += -L/usr/local/Aria/lib/ \
    -lAria -lArNetworking
