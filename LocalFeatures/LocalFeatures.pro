QT += core
QT -= gui

TARGET = LocalFeatures
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \

#PCL Stuff
#INCLUDEPATH += "/home/nathan/Downloads/pcl-master/visualization/include"
#INCLUDEPATH += "/home/nathan/Downloads/pcl-master/kdtree/include"

#INCLUDEPATH += "/home/nathan/Downloads/pcl-master/io/include"
#INCLUDEPATH += "/home/nathan/Downloads/pcl-master/features/include"
#INCLUDEPATH += "/home/nathan/Downloads/pcl-master/common/include"
#INCLUDEPATH += "/home/nathan/Downloads/pcl-master/release/include"

#Eigen Stuff

INCLUDEPATH += "/usr/include/eigen3"
INCLUDEPATH += "/usr/include/vtk-6.2"

INCLUDEPATH += "/usr/include/pcl-1.8"
#INCLUDEPATH += "/usr/include/vtk-6.2"
#INCLUDEPATH += "/usr/include/boost"
#INCLUDEPATH += "/usr/include/eigen3"



#unix:!macx: LIBS += -L$$PWD/../Downloads/pcl-master/release/lib/ -lpcl_search

#INCLUDEPATH += $$PWD/../Downloads/pcl-master/release/include
#DEPENDPATH += $$PWD/../Downloads/pcl-master/release/include
unix:!macx: LIBS += -L$$PWD/../../../usr/lib/ -lboost_system

INCLUDEPATH += $$PWD/../../../usr/include
DEPENDPATH += $$PWD/../../../usr/include


unix:!macx: LIBS += -L$$PWD/../../../usr/lib/ -lpcl_visualization

INCLUDEPATH += $$PWD/../../../usr/include
DEPENDPATH += $$PWD/../../../usr/include

unix:!macx: LIBS += -L$$PWD/../../../usr/lib/ -lpcl_io

INCLUDEPATH += $$PWD/../../../usr/include
DEPENDPATH += $$PWD/../../../usr/include
unix:!macx: LIBS += -L$$PWD/../../../usr/lib/ -lpcl_kdtree

INCLUDEPATH += $$PWD/../../../usr/include
DEPENDPATH += $$PWD/../../../usr/include

unix:!macx: LIBS += -L$$PWD/../../../usr/lib/ -lpcl_common

INCLUDEPATH += $$PWD/../../../usr/include
DEPENDPATH += $$PWD/../../../usr/include



unix:!macx: LIBS += -L$$PWD/../../../usr/lib/ -lpcl_features

INCLUDEPATH += $$PWD/../../../usr/include
DEPENDPATH += $$PWD/../../../usr/include

unix:!macx: LIBS += -L$$PWD/../../../usr/lib/ -lpcl_search

INCLUDEPATH += $$PWD/../../../usr/include
DEPENDPATH += $$PWD/../../../usr/include



unix:!macx: LIBS += -L$$PWD/../../../usr/lib/ -lpcl_filters

INCLUDEPATH += $$PWD/../../../usr/include
DEPENDPATH += $$PWD/../../../usr/include


unix:!macx: LIBS += -L$$PWD/../../../usr/lib/x86_64-linux-gnu/ -lvtkRenderingLOD-6.2

INCLUDEPATH += $$PWD/../../../usr/lib/x86_64-linux-gnu
DEPENDPATH += $$PWD/../../../usr/lib/x86_64-linux-gnu

unix:!macx: LIBS += -L$$PWD/../../../usr/lib/x86_64-linux-gnu/ -lvtkRenderingCore-6.2

INCLUDEPATH += $$PWD/../../../usr/lib/x86_64-linux-gnu
DEPENDPATH += $$PWD/../../../usr/lib/x86_64-linux-gnu

unix:!macx: LIBS += -L$$PWD/../../../usr/lib/x86_64-linux-gnu/ -lvtkCommonDataModel-6.2

INCLUDEPATH += $$PWD/../../../usr/lib/x86_64-linux-gnu
DEPENDPATH += $$PWD/../../../usr/lib/x86_64-linux-gnu

unix:!macx: LIBS += -L$$PWD/../../../usr/lib/x86_64-linux-gnu/ -lvtkCommonMath-6.2

INCLUDEPATH += $$PWD/../../../usr/lib/x86_64-linux-gnu
DEPENDPATH += $$PWD/../../../usr/lib/x86_64-linux-gnu

unix:!macx: LIBS += -L$$PWD/../../../usr/lib/x86_64-linux-gnu/ -lvtkCommonCore-6.2

INCLUDEPATH += $$PWD/../../../usr/lib/x86_64-linux-gnu
DEPENDPATH += $$PWD/../../../usr/lib/x86_64-linux-gnu

unix:!macx: LIBS += -L$$PWD/../../../usr/lib/x86_64-linux-gnu/ -lboost_thread

INCLUDEPATH += $$PWD/../../../usr/lib/x86_64-linux-gnu
DEPENDPATH += $$PWD/../../../usr/lib/x86_64-linux-gnu

unix:!macx: LIBS += -L$$PWD/../../../usr/lib/x86_64-linux-gnu/ -lvtkFiltersSources-6.2

INCLUDEPATH += $$PWD/../../../usr/lib/x86_64-linux-gnu
DEPENDPATH += $$PWD/../../../usr/lib/x86_64-linux-gnu

unix:!macx: LIBS += -L$$PWD/../../../usr/lib/x86_64-linux-gnu/ -lvtkCommonExecutionModel-6.2

INCLUDEPATH += $$PWD/../../../usr/lib/x86_64-linux-gnu
DEPENDPATH += $$PWD/../../../usr/lib/x86_64-linux-gnu
