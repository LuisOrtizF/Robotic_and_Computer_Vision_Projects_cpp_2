#include "Save.hpp"
#include <ctime>
#include <boost/filesystem.hpp>
#include <sl/Camera.hpp>

using namespace std;

int count_save = 0;

std::string path;
std::string pathLeft;
std::string pathRight;
std::string pathLeft_Unrec;
std::string pathRight_Unrec;

std::string get_unique_filename(){

  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer,sizeof(buffer),"%d_%m_%Y_%I_%M_%S",timeinfo);
  std::string str(buffer);

  return str;

}

void create_dir(){

    path = get_unique_filename();
    pathLeft = path+"/left/";
    pathRight = path+"/right/";
    pathLeft_Unrec = path+"/left_unrec/";
    pathRight_Unrec = path+"/right_unrec/";

    boost::filesystem::create_directory(path);
    boost::filesystem::create_directory(pathLeft);
    boost::filesystem::create_directory(pathRight);
    boost::filesystem::create_directory(pathLeft_Unrec);
    boost::filesystem::create_directory(pathRight_Unrec);
}

void run(sl::Camera& zed) {

    if(count_save == 0){
        create_dir();
        save_calib_params(zed);
    }

    saveImages(zed, pathLeft + "/" + std::to_string(count_save) + std::string(".png"), 
                    pathRight + "/" + std::to_string(count_save) + std::string(".png"),
                    pathLeft_Unrec + "/" + std::to_string(count_save) + std::string(".png"),
                    pathRight_Unrec + "/" + std::to_string(count_save) + std::string(".png"));
    count_save++;
}

void save_calib_params(sl::Camera& zed){

    // CalibrationParameters
    // Those parameters are taken from the settings file (SNXXX.conf) and are 
    // modified during the sl::Camera::open call (with or without Self-Calibration). 
    // Those parameters given after sl::Camera::open call, represent the  
    // "new camera matrix" that fits/defines each image taken after rectification
    // (through retrieveImage). fx,fy,cx,cy must be the same for Left and Right 
    // Camera once sl::Camera::open has been called. Since distortion is corrected  
    // during rectification, distortion should not be considered after
    // sl::Camera::open call.
    
    // Factory camera calibration parameters

    sl::CameraInformation cam_info = zed.getCameraInformation();
    sl::CalibrationParameters calib_param_raw = cam_info.camera_configuration.calibration_parameters_raw;

    float fx_l = calib_param_raw.left_cam.fx;
    float fy_l = calib_param_raw.left_cam.fy;
    float cx_l = calib_param_raw.left_cam.cx;
    float cy_l = calib_param_raw.left_cam.cy;
    double 	disto1_l = calib_param_raw.left_cam.disto[0];
    double 	disto2_l = calib_param_raw.left_cam.disto[1];

    float fx_r = calib_param_raw.right_cam.fx;
    float fy_r = calib_param_raw.right_cam.fy;
    float cx_r = calib_param_raw.right_cam.cx;
    float cy_r = calib_param_raw.right_cam.cy;
    double 	disto1_r = calib_param_raw.right_cam.disto[0];
    double 	disto2_r = calib_param_raw.right_cam.disto[1];

    sl::CalibrationParameters calib_param = cam_info.camera_configuration.calibration_parameters;

    float fx = calib_param.left_cam.fx;
    float fy = calib_param.left_cam.fy;
    float cx = calib_param.left_cam.cx;
    float cy = calib_param.left_cam.cy;

    std::ofstream file_calibration (path+"/calibration.txt");

    if (file_calibration.is_open ())
        file_calibration << fx_l << " " << 0.0  << " " << cx_l << " " << 0.0 << " "
                         << 0.0  << " " << fy_l << " " << cy_l << " " << 0.0 << " "
                         << 0.0  << " " << 0.0  << " " <<  1.0 << " " << 0.0 << " " 
                         << disto1_l << " " << disto2_l << "\n"
                         << fx_r << " " << 0.0  << " " << cx_r << " " << 0.0 << " "
                         << 0.0  << " " << fy_r << " " << cy_r << " " << 0.0 << " "
                         << 0.0  << " " << 0.0  << " " <<  1.0 << " " << 0.0 << " " 
                         << disto1_r << " " << disto2_r << "\n"
                         << fx  << " " << 0.0 << " " << cx  << " " << 0.0 << " "
                         << 0.0 << " " << fy  << " " << cy  << " " << 0.0 << " "
                         << 0.0 << " " << 0.0 << " " << 1.0 << " " << 0.0 << " " 
                         << 0.0 << " " << 0.0 << std::endl;

    file_calibration.close();
}

void saveImages(sl::Camera& zed, std::string filename1, std::string filename2,
                                 std::string filename3,std::string filename4){
    
    std::cout << "Saving Images... " << flush;

    sl::CameraInformation cam_info = zed.getCameraInformation();
    sl::Resolution image_size = cam_info.camera_configuration.resolution;

    cv::Mat left_image(image_size.width, image_size.height, CV_8UC4);
    cv::Mat right_image(image_size.width, image_size.height, CV_8UC4);
    cv::Mat left_image_Unrec(image_size.width, image_size.height, CV_8UC4);
    cv::Mat right_image_Unrec(image_size.width, image_size.height, CV_8UC4);

    sl::Mat buffer_sl;
    cv::Mat buffer_cv;

    zed.retrieveImage(buffer_sl, sl::VIEW::LEFT);
    buffer_cv = cv::Mat(buffer_sl.getHeight(), buffer_sl.getWidth(), CV_8UC4, buffer_sl.getPtr<sl::uchar1>(sl::MEM::CPU));
    buffer_cv.copyTo(left_image);
   
    zed.retrieveImage(buffer_sl, sl::VIEW::RIGHT);
    buffer_cv = cv::Mat(buffer_sl.getHeight(), buffer_sl.getWidth(), CV_8UC4, buffer_sl.getPtr<sl::uchar1>(sl::MEM::CPU));
    buffer_cv.copyTo(right_image);

    zed.retrieveImage(buffer_sl, sl::VIEW::LEFT_UNRECTIFIED);
    buffer_cv = cv::Mat(buffer_sl.getHeight(), buffer_sl.getWidth(), CV_8UC4, buffer_sl.getPtr<sl::uchar1>(sl::MEM::CPU));
    buffer_cv.copyTo(left_image_Unrec);
   
    zed.retrieveImage(buffer_sl, sl::VIEW::RIGHT_UNRECTIFIED);
    buffer_cv = cv::Mat(buffer_sl.getHeight(), buffer_sl.getWidth(), CV_8UC4, buffer_sl.getPtr<sl::uchar1>(sl::MEM::CPU));
    buffer_cv.copyTo(right_image_Unrec);

    cv::cvtColor(left_image, left_image, CV_RGBA2RGB);
    cv::cvtColor(right_image, right_image, CV_RGBA2RGB);
    cv::cvtColor(left_image_Unrec, left_image_Unrec, CV_RGBA2RGB);
    cv::cvtColor(right_image_Unrec, right_image_Unrec, CV_RGBA2RGB);

    cv::imwrite(filename1, left_image);
    cv::imwrite(filename2, right_image);
    cv::imwrite(filename3, right_image_Unrec);
    cv::imwrite(filename4, right_image_Unrec);

    std::cout << "Done" << endl;
}