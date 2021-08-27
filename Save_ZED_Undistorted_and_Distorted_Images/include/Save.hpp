#ifndef __SAVE_HPP__
#define __SAVE_HPP__

#define NOMINMAX

#include <iomanip>
#include <signal.h>
#include <iostream>
#include <limits>
#include <thread>

#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>

std::string get_unique_file_name();
void create_dir();
void save_calib_params(sl::Camera& zed);
void saveImages(sl::Camera& zed, std::string filename1, std::string filename2, std::string filename3, std::string filename4);

void run(sl::Camera& zed);

#endif