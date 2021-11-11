QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH += C:/Devsoft/boost_1_76_0


SOURCES += \
    autoscale_graph.cpp \
    equation.cpp \
    integ_meth.cpp \
    main.cpp \
    mainwindow.cpp \
    qcustomplot.cpp \
    resolving.cpp

HEADERS += \
    autoscale_graph.h \
    equation.h \
    integ_meth.h \
    mainwindow.h \
    mystruct.h \
    qcustomplot.h \
    resolving.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
