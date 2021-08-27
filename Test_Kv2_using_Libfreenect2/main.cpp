#include "opencv2/opencv.hpp"

#include <iostream>
#include <signal.h>

#include "libfreenect2/libfreenect2.hpp"
#include "libfreenect2/frame_listener_impl.h"
#include "libfreenect2/registration.h"
#include "libfreenect2/packet_pipeline.h"
#include <libfreenect2/logger.h>

using namespace cv;
using namespace std;

bool protonect_shutdown = false;

void sigint_handler(int s)
{
  protonect_shutdown = true;
}

int main(){

    std::cout << "Streaming from Kinect v2!" << std::endl;

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;  
    libfreenect2::setGlobalLogger(NULL);

    if(freenect2.enumerateDevices() == 0)
    {
      std::cout << "no device connected!" << std::endl;
      return -1;
    }
    
    std::string serial = freenect2.getDefaultDeviceSerialNumber();
    std::cout << "serial: " << serial << std::endl;

    if(pipeline)
    {
        dev = freenect2.openDevice(serial, pipeline);
    } else {
        dev = freenect2.openDevice(serial);
    }

    if(dev == 0)
    {
      std::cout << "failure opening device!" << std::endl;
      return -1;
    }
   
    signal(SIGINT,sigint_handler);
    protonect_shutdown = false;

    int types = 0;
    types |= libfreenect2::Frame::Color;
    types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
    libfreenect2::SyncMultiFrameListener listener(types);
    libfreenect2::FrameMap frames;

    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);

    dev->start();

    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4);

    size_t framecount = 0;
    size_t framemax = 30;

    cv::Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2;
   
    while(!protonect_shutdown && (framecount < framemax))
    {
        listener.waitForNewFrame(frames);
        
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
        cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
        cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);

        cv::imshow("rgb", rgbmat);
        cv::imshow("ir", irmat / 4096.0f);
        cv::imshow("depth", depthmat / 4096.0f);

        registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);
        
        cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(depthmatUndistorted);
        cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
        cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);

        cv::imshow("undistorted", depthmatUndistorted / 4096.0f);
        cv::imshow("registered", rgbd);
        cv::imshow("depth2RGB", rgbd2 / 4096.0f);

        framecount++;

        int key = cv::waitKey(1);
        protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape

        listener.release(frames);
    }

    dev->stop();
    dev->close();

    delete registration;

    std::cout << "Streaming Ends!" << std::endl;   

    return 0;
}