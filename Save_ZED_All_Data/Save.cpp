#include "Save.hpp"
#include <ctime>
#include <boost/filesystem.hpp>
#include <sl/Camera.hpp>

int count_save = 0;
sl::Pose zed_pose;

std::ofstream file_poses;
std::ofstream file_times;
std::ofstream file_confidence;

std::string path;
std::string pathLeft;
std::string pathRight;
std::string pathDepth;
std::string pathPointCloud;

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
    pathDepth = path+"/depth/";
    pathPointCloud = path+"/cloud/";

    boost::filesystem::create_directory(path);
    boost::filesystem::create_directory(pathLeft);
    boost::filesystem::create_directory(pathRight);
    boost::filesystem::create_directory(pathDepth);
    boost::filesystem::create_directory(pathPointCloud);

}

void run(sl::Camera& zed, char &key) {

    if(count_save == 0){
        create_dir();
        file_poses.open(path+"/poses.txt");
        file_confidence.open(path+"/confidence.txt");
        file_times.open(path+"/times.txt");
        save_calib_params(zed);
    }

    saveImages(zed, pathLeft + "/" + std::to_string(count_save) + std::string(".png"), 
                    pathRight + std::to_string(count_save) + std::string(".png"));
    
    savePose(zed);
    
    saveDepth(zed, pathDepth + "/" + std::to_string(count_save));
    savePointCloud(zed, pathPointCloud + "/" + std::to_string(count_save));

    count_save++;

    if(key == 'q'){
        file_poses.close();
        file_confidence.close();
        file_times.close();
    }
}

void save_calib_params(sl::Camera& zed){

    sl::CameraInformation cam_info = zed.getCameraInformation();
    
    sl::CalibrationParameters calib_param = cam_info.camera_configuration.calibration_parameters;
    float fx_l = calib_param.left_cam.fx;
    float fy_l = calib_param.left_cam.fy;
    float cx_l = calib_param.left_cam.cx;
    float cy_l = calib_param.left_cam.cy;

    std::ofstream file_calibration (path+"/calibration.txt");

    if (file_calibration.is_open ())
        file_calibration << fx_l << " " << 0.0 << " " << cx_l << " " << 0.0 << " "
                         << 0.0 << " " << fy_l << " " << cy_l << " " << 0.0 << " "
                         << 0.0 << " " <<  0.0 << " " <<  1.0 << " " << 0.0 << std::endl;

    file_calibration.close();
}

void saveImages(sl::Camera& zed, std::string filename1, std::string filename2) {
    
    std::cout << "Saving Images... " << std::flush;

    sl::CameraInformation cam_info = zed.getCameraInformation();
    sl::Resolution image_size = cam_info.camera_configuration.resolution;

    cv::Mat left_image(image_size.width, image_size.height, CV_8UC4);
    cv::Mat right_image(image_size.width, image_size.height, CV_8UC4);

    sl::Mat buffer_sl;
    cv::Mat buffer_cv;

    zed.retrieveImage(buffer_sl, sl::VIEW::LEFT);
    buffer_cv = cv::Mat(buffer_sl.getHeight(), buffer_sl.getWidth(), CV_8UC4, buffer_sl.getPtr<sl::uchar1>(sl::MEM::CPU));
    buffer_cv.copyTo(left_image);
   
    zed.retrieveImage(buffer_sl, sl::VIEW::RIGHT);
    buffer_cv = cv::Mat(buffer_sl.getHeight(), buffer_sl.getWidth(), CV_8UC4, buffer_sl.getPtr<sl::uchar1>(sl::MEM::CPU));
    buffer_cv.copyTo(right_image);

    cv::cvtColor(left_image, left_image, CV_RGBA2RGB);
    cv::cvtColor(right_image, right_image, CV_RGBA2RGB);

    cv::imwrite(filename1, left_image);
    cv::imwrite(filename2, right_image);

    std::cout << "Done" << std::endl;
}

void savePose(sl::Camera& zed) {

    // Get the pose of the left eye of the camera with reference to the world frame
    sl::POSITIONAL_TRACKING_STATE tracking_state = zed.getPosition(zed_pose, sl::REFERENCE_FRAME::WORLD); 

    if (tracking_state == sl::POSITIONAL_TRACKING_STATE::OK) {
        
        std::cout << "Saving POSES... " << std::flush;

        // Save the pose data in a txt file
        if (file_poses.is_open () && file_confidence.is_open () && file_times.is_open ()){

            sl::float3 t = zed_pose.getTranslation();
            sl::Matrix3f r = zed_pose.getRotationMatrix();	

            file_poses << r.r00 << " " << r.r01 << " " << r.r02 << " " << t.x << " "
                       << r.r10 << " " << r.r11 << " " << r.r12 << " " << t.y << " "
                       << r.r20 << " " << r.r21 << " " << r.r22 << " " << t.z << std::endl;
        
            // printf("\n\t    | %6.3f %6.3f %6.3f | \n", r.r00, r.r01, r.r02);
            // printf("\tR = | %6.3f %6.3f %6.3f | \n", r.r10, r.r11, r.r12);
            // printf("\t    | %6.3f %6.3f %6.3f | \n", r.r20, r.r21, r.r22);
            // std::cout << std::endl;
            // printf("\tt = < %0.3f, %0.3f, %0.3f >\n", t.x, t.y, t.z);

            //Confidence/Quality of the pose estimation for the target frame. 
            file_confidence << zed_pose.pose_confidence << std::endl;

                        // getCameraTimestamp ()
            // Returns the timestamp at the time the frame has been 
            // extracted from USB stream. (should be called after a grab()).
            
            // getCurrentTimestamp ()
            // Returns the current timestamp at the time the function is called. 
            // Can be compared to the camera getCameraTimestamp for synchronization.
            // Use this function to compare the current timestamp and the camera 
            // timestamp, since they have the same reference (Computer start time).

            // zed_pose.timestamp -> Timestamp of the pose. This 
            // timestamp should be compared with the camera timestamp 
            // for synchronization.

            // All timestamps are in nanoseconds.

            file_times << zed.getTimestamp(sl::TIME_REFERENCE::IMAGE).getMilliseconds() << std::endl;
        }
        else std::cout << "Unable to open files" << std::endl;
        
        std::cout << "Done\n" << std::endl;
    } 
    else std::cout << "Positional tracking is not enabled" << std::endl; 
}

void saveDepth(sl::Camera& zed, std::string filename) {

    std::cout << "Saving Depth Map... " << std::flush;

    sl::Mat depth;
    zed.retrieveMeasure(depth, sl::MEASURE::DEPTH);

    convertUnit(depth, zed.getInitParameters().coordinate_units, sl::UNIT::MILLIMETER);
    auto state = depth.write((filename + ".png").c_str());

    if (state == sl::ERROR_CODE::SUCCESS)
        std::cout << "Depth Map has been save under " << filename << ".png" << std::endl;
    else
		std::cout << "Failed to save depth map... Please check that you have permissions to write at this location (" << filename << "). Re-run the sample with administrator rights under windows" << std::endl;

}

void savePointCloud(sl::Camera& zed, std::string filename) {
    std::cout << "Saving Point Cloud... " << std::flush;

    sl::Mat point_cloud;
    zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA);

    auto state = point_cloud.write((filename + ".pcd").c_str());

    if (state == sl::ERROR_CODE::SUCCESS)
        std::cout << "Point Cloud has been saved under " << filename << ".pcd" << std::endl;
    else
        std::cout << "Failed to save point cloud... Please check that you have permissions to write at this location ("<< filename<<"). Re-run the sample with administrator rights under windows" << std::endl;
}