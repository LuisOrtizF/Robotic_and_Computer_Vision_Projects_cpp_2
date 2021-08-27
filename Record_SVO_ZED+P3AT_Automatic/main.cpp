#include "Aria.h"
#include <iostream>
#include <string>
#include <sl/Camera.hpp>
#include <boost/filesystem.hpp>

using namespace boost::filesystem;

sl::InitParameters zedParams;
ArRobot robot;
sl::String pathSVO;

void record(int iter, int distance)
{
  printf("\n== autoop Mode ==\n");
  printf("\tThe robot will now only activate to record videos automatically.\n");

  sl::Camera zed;

  // Open the ZED
  sl::ERROR_CODE zed_open_state = zed.open(zedParams);

  if (zed_open_state != sl::ERROR_CODE::SUCCESS)
  {
    std::cout << "\n\n\tCamera Open " << zed_open_state << " Exit program.\n" << std::endl;
    zed.close();
    exit(1);
  }

  auto returned_state = zed.enableRecording(sl::RecordingParameters(pathSVO, sl::SVO_COMPRESSION_MODE::H264));

  if (returned_state != sl::ERROR_CODE::SUCCESS)
  {
    std::cout << "\tRecording initialization error. " << returned_state << std::endl;

    if (returned_state == sl::ERROR_CODE::SVO_RECORDING_ERROR)
        std::cout << "\tNote : This error mostly comes from a wrong path or missing writing permissions." << std::endl;
    if (returned_state == sl::ERROR_CODE::SVO_UNSUPPORTED_COMPRESSION)
        std::cout << "\tNote : This error mostly comes from a non-compatible graphic card. If you are using HEVC compression (H265), please note that most of the graphic card below pascal architecture will not support it. Prefer to use AVCHD compression which is supported on most of NVIDIA graphic cards" << std::endl;

    zed.close();
    exit(1);
  }

  int frames_recorded = 0;

  ArPose pose;
  std::cout << "\tForward-->\t";

  pose = ArPose(0.0,0.0,0.0);

  if (iter==1)
    sl::sleep_ms(10000);

  while (pose.getX() <= distance)
  {
    if (zed.grab() == sl::ERROR_CODE::SUCCESS)
    {
      // Each new frame is added to the SVO file
      sl::RecordingStatus state = zed.getRecordingStatus();
      if (state.status)
        frames_recorded++;     
    }

    robot.lock();
    robot.setVel(iter*100.0);
    pose = robot.getPose();
    robot.unlock();
  }

  ArLog::log(ArLog::Normal, "(%.0f, %.0f)", pose.getX(), pose.getY());
  
  std::cout << "\tFrames Recorded: " << frames_recorded << std::endl;
  zed.disableRecording();
  zed.close();

  robot.lock();
  robot.stop();
  robot.unlock();
  ArUtil::sleep(4000);

  std::cout << "\tBackward-->\t";

  while(pose.getX() >= 0.0 )
  {
    robot.lock();
    robot.setVel(-400);
    pose = robot.getPose();
    robot.unlock();
    ArUtil::sleep(100);
  }

  ArLog::log(ArLog::Normal, "(%.0f, %.0f)", pose.getX(), pose.getY());

  robot.lock();
  robot.stop();
  robot.unlock();
  ArUtil::sleep(4000);
}

int main(int argc, char** argv)
{
  std::cout << "\nRecord_SVO_ZED+P3AT_Automatic\n" << std::endl;

  if (argc != 6){
    printf("Syntax is: %s <VGA or HD720 or HD1080 or HD2K> <output_dir> <distance_in_mm> -rp /dev/ttyUSB0\n\n", argv[0]);
    std::cout << "\n\n\tPress [Enter] to continue.\n" << std::endl;
    std::cin.ignore();
    return 1;
  }

  Aria::init();
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();

  ArRobotConnector robotConnector(&parser, &robot);

  if(!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "Could not connect to the robot.");
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

  ArLog::log(ArLog::Normal, "Connected to robot.");

  ArKeyHandler *keyHandler = Aria::getKeyHandler();
  keyHandler = new ArKeyHandler;
  Aria::setKeyHandler(keyHandler);
  robot.attachKeyHandler(keyHandler);

  ArActionKeydrive *keydriveAct = new ArActionKeydrive("keydrive", 200, 10);
  robot.addAction(keydriveAct, 100);

  robot.enableMotors();
  robot.runAsync(true);

  //****************************************************************ZED********************************************************************
  
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
    
  std::string firstPath;
  if(is_directory(std::string(argv[2])))
  {
    firstPath = std::string(argv[2]) + "error/";
    create_directory(firstPath);
  }
  else
  {
    std::cout << "\n\n\t" << std::string(argv[2]) << " not exist.\n" << std::endl;
    return 1;
  }

  printf("\n== keyoperation Mode ==\n");
  printf("\tUse the arrow keys to drive, and the spacebar to stop.\n\tPress 'a' to switch to autoop mode.\n\tPress escape to exit.\n");

  int iteration = 1;
  int distance = atoi(argv[3]);

  while(1)
  {
    robot.lock();
    keydriveAct->activate();
    robot.unlock();
    ArUtil::sleep(100);

    if (keyHandler->getKey() == 97) //a
    {
      robot.lock();
      keydriveAct->deactivate();
      robot.unlock();
      ArUtil::sleep(100);

      while(iteration < 7)
      {
        pathSVO = (firstPath + "e" + std::to_string(iteration) + ".svo").c_str();
        record(iteration, distance);
        iteration++;
      }

      iteration = 1;
      robot.lock();
      robot.clearDirectMotion();
      robot.unlock();
      ArUtil::sleep(100);
      printf("\n== keyoperation Mode ==\n");
      printf("\tUse the arrow keys to drive, and the spacebar to stop.\n\tPress 'a' to switch to autoop mode.\n\tPress escape to exit.\n");
    }

    if (keyHandler->getKey() == 27) // 'esc'
      break;
  }

  robot.stopRunning();
  robot.waitForRunExit();
  Aria::exit(0); 
  return 0;
}
