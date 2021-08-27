#include "Aria.h"
#include <iostream>
#include <string>
#include <aruco/aruco.h>
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>
#include <boost/filesystem.hpp>
#include <ctime>

using namespace boost::filesystem;

std::string get_unique_filename()
{
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer,sizeof(buffer),"%Y%m%d%I%M%S",timeinfo);
  std::string str(buffer);

  return str;
}

cv::Mat slMat2cvMat(sl::Mat &input)
{
  int cv_type = -1;
  switch (input.getDataType())
  {
    case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
    case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
    case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
    case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
    case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
    case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
    case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
    case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
    default: break;
  }

  return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM::CPU));
}

int main(int argc, char** argv)
{

  std::cout << "\nRecord_SVO_ZED+P3AT_Keyboard\n" << std::endl;

  if (argc != 5)
  {
    printf("Syntax is: %s <VGA or HD720 or HD1080 or HD2K> <output_dir> -rp /dev/ttyUSB0\n", argv[0]);
    std::cout << "\nPress [Enter] to continue.\n" << std::endl;
    std::cin.ignore();
    return 1;
  }

  //*************************************************************************************************

  Aria::init(); // Initialize some global data
  ArArgumentParser parser(&argc, argv); // This object parses program options from the command line
  parser.loadDefaultArguments(); // Load some default values for command line arguments from /etc/Aria.args
  ArRobot robot; // Central object
  ArRobotConnector robotConnector(&parser, &robot); // Object that connects to the robot

  // Connect to the robot
  if (!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "-->Could not connect to the robot");
    if(parser.checkHelpAndWarnUnparsed())
    {
      Aria::logOptions();
      Aria::exit(1);
    }
  }

  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions();
    Aria::exit(1);
  }

  // Used to perform actions when keyboard keys are pressed
  ArActionKeydrive keydriveAct;
  robot.setTransVelMax(200.0);
  robot.enableMotors();
  robot.addAction(&keydriveAct, 100);
  robot.runAsync(true);

  //*************************************************************************************************

  // Create a ZED camera object
  sl::Camera zed;

  // Set configuration parameters for the ZED
  sl::InitParameters zedParams;
  if (std::string(argv[1]) == "VGA") 
    zedParams.camera_resolution = sl::RESOLUTION::VGA;
  else if (std::string(argv[1]) == "HD720") 
    zedParams.camera_resolution = sl::RESOLUTION::HD720;
  else if (std::string(argv[1]) == "HD1080") 
    zedParams.camera_resolution = sl::RESOLUTION::HD1080;
  else if (std::string(argv[1]) == "HD2K") 
    zedParams.camera_resolution = sl::RESOLUTION::HD2K;
  else 
    return 1;

  zedParams.depth_mode = sl::DEPTH_MODE::QUALITY;
  zedParams.camera_disable_self_calib = true;
  zedParams.coordinate_units = sl::UNIT::METER;
  
  // Open the ZED
  sl::ERROR_CODE zed_open_state = zed.open(zedParams);

  if (zed_open_state != sl::ERROR_CODE::SUCCESS)
  {
    std::cout << "\n\tCamera not open: " << zed_open_state << ". Exit program.\n" << std::endl;
    zed.close();
    return 1;
  }

  std::cout << "\nPlease wait 30 seconds for ZED camera estabilization.\n" << std::endl;
  sl::sleep_ms(30000);

  std::string firstPath = std::string(argv[2]) + "data_" + std::string(argv[1]) + "_" + get_unique_filename() + "/";
  create_directory(firstPath);
  std::string secondPath = firstPath + "mm/";
  create_directory(secondPath);
  std::string thirdPath = firstPath + "vo/";
  create_directory(thirdPath);

  // CalibrationParameters calibration_parameters
  // Intrinsic and Extrinsic stereo parameters for rectified/undistorded images (default).

  // CalibrationParameters calibration_parameters_raw
  // Intrinsic and Extrinsic stereo parameters for original images (unrectified/distorded).

  sl::CalibrationParameters camInt = zed.getCameraInformation().calibration_parameters;
  float fx_l = camInt.left_cam.fx;
  float fy_l = camInt.left_cam.fy;
  float cx_l = camInt.left_cam.cx;
  float cy_l = camInt.left_cam.cy;
  double d0_l = camInt.left_cam.disto[0];
  double d1_l = camInt.left_cam.disto[1];
  double d2_l = camInt.left_cam.disto[2];
  double d3_l = camInt.left_cam.disto[3];
  double d4_l = camInt.left_cam.disto[4];

  aruco::CameraParameters CamParam;
  CamParam.CameraMatrix = (cv::Mat_<float>(3,3) << fx_l, 0, cx_l, 0, fy_l, cy_l, 0, 0, 1);
  CamParam.Distorsion = (cv::Mat_<double>(1,5) << d0_l, d1_l, d2_l, d3_l, d4_l);
  auto resolution = zed.getCameraInformation().camera_configuration.resolution;
  CamParam.CamSize = cv::Size(resolution.width, resolution.height);
  CamParam.saveToFile(secondPath + "cam_param.yml");
  
  cv::namedWindow("Recording", CV_WINDOW_NORMAL);
  cv::moveWindow("Recording", 0, 0);
  cv::resizeWindow("Recording", 672, 376);

  sl::Mat left_image (resolution.width, resolution.height, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
  cv::Mat left_image_cv_rgba = slMat2cvMat(left_image);

  //*************************************************************************************************
  sl::String pathSVO = (secondPath + "mm.svo").c_str();

  auto returned_state = zed.enableRecording(sl::RecordingParameters(pathSVO, sl::SVO_COMPRESSION_MODE::H264));

  if (returned_state != sl::ERROR_CODE::SUCCESS){
    std::cout << "Recording initialization error. " << returned_state << std::endl;
    if (returned_state == sl::ERROR_CODE::SVO_RECORDING_ERROR)
      std::cout << " Note : This error mostly comes from a wrong path or missing writing permissions." << std::endl;
    if (returned_state == sl::ERROR_CODE::SVO_UNSUPPORTED_COMPRESSION)
      std::cout << " Note : This error mostly comes from a non-compatible graphic card. If you are using HEVC compression (H265), please note that most of the graphic card below pascal architecture will not support it. Prefer to use AVCHD compression which is supported on most of NVIDIA graphic cards" << std::endl;

    zed.close();
    return 1;
  }

  char key = ' ';
  int frames_recorded = 0;

  std::cout << "\n*********DRIVE with keyboard or press 's' to RECORD or press 'q' to EXIT*********\n" << std::endl;

  while(key != 's')
    key = cv::waitKey(1);

  while (key != 'q')
  {
    if (zed.grab() == sl::ERROR_CODE::SUCCESS)
    {
      // Each new frame is added to the SVO file
      sl::RecordingStatus state = zed.getRecordingStatus();
      if (state.status)
        frames_recorded++;

      zed.retrieveImage(left_image, sl::VIEW::LEFT, sl::MEM::CPU, resolution);
      cv::imshow("Recording", left_image_cv_rgba);
      key = cv::waitKey(1);
    }
  }

  cv::destroyAllWindows();
  std::cout << "\nFrames Recorded: " << frames_recorded << std::endl;
  std::cout << "\nShutting and exiting\n" << std::endl;

  zed.disableRecording();
  zed.close();
  robot.stopRunning();
  robot.waitForRunExit();

  assert(boost::filesystem::is_regular_file(secondPath + "mm.svo") && "Assume file is present");
  boost::filesystem::copy_file(secondPath + "mm.svo", thirdPath + "vo.svo");

  Aria::exit(0);
  return 0;
}