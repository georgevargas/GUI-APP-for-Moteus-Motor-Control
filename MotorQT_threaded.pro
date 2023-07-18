#-------------------------------------------------
#
# Project created by QtCreator 2021-10-29T14:29:12
#
#-------------------------------------------------

QT       += core gui
QT       += widgets
qtHaveModule(printsupport): QT += printsupport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = MotorQT_threaded
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

        SOURCES += \
        main.cpp \
        mainwindow.cpp \
        Motorworker.cpp \
        MoteusAPI.cpp \
        qcustomplot.cpp

HEADERS += \
        mainwindow.h \
        Motorworker.h \
        moteus_protocol.h \
        MoteusAPI.h \
        qcustomplot.h

FORMS += \
        mainwindow.ui

INCLUDEPATH += /usr/include/qwt

LIBS += \
        -L/usr/local/lib \

LIBS += \
        -L/usr/lib \
        -lqwt-qt5

RESOURCES += \
        resources.qrc

        QMAKE_CXXFLAGS_CXX2A = -std:c++20
        QMAKE_CXXFLAGS += -std=c++2a
        QMAKE_CXX = g++-13

DISTFILES +=
