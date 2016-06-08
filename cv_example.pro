QT += core widgets


CONFIG += c++11

TARGET = cv_example
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app
INCLUDEPATH += F:/Workspace/opencv_library/opencv_binary/install/include
LIBS += -LF:\\Workspace\\opencv_library\\opencv_binary\\bin \
    libopencv_core310 \
    libopencv_highgui310 \
    libopencv_imgproc310 \
    libopencv_features2d310 \
    libopencv_calib3d310 \
libopencv_imgcodecs310\
libopencv_video310\
libopencv_videoio310\
libopencv_videostab310\
libopencv_objdetect310\

SOURCES += main.cpp \
    singlekalmanfilter.cpp \
    object.cpp \
    detection.cpp \
    reader.cpp

DISTFILES += \
    ../../Downloads/test.mp4

HEADERS += \
    singlekalmanfilter.h \
    object.h \
    detection.h \
    reader.h



