#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <iostream>
#include <boost/filesystem.hpp>

using namespace boost::filesystem;

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

int main(int argc, char **argv) 
{
    if (argc != 2 )
    {
        printf("\n\n\tSyntax is: %s <video.svo>", argv[0]);
        std::cout << "\n\n\tPress [Enter] to continue.\n" << std::endl;
        std::cin.ignore();
        return 1;
    }

    // ZED
    sl::Camera zed;
    sl::InitParameters zedParams;
    zedParams.input.setFromSVOFile(argv[1]);
    zedParams.depth_mode = sl::DEPTH_MODE::QUALITY;
    zedParams.camera_disable_self_calib = true;
    zedParams.coordinate_units = sl::UNIT::METER;

    // Open the ZED
    sl::ERROR_CODE zed_open_state = zed.open(zedParams);
    if (zed_open_state != sl::ERROR_CODE::SUCCESS) 
    {
        std::cout << "\n\n\tCamera Open " << zed_open_state << " Exit program.\n" << std::endl;
        zed.close();
        return 1;
    }

    // Enable positional tracking with default parameters
    sl::ERROR_CODE zed_pose_state = zed.enablePositionalTracking();

    if (zed_pose_state != sl::ERROR_CODE::SUCCESS) 
    {
        std::cout << "Enabling positionnal tracking failed: " << zed_pose_state << std::endl;
        zed.close();
        return 1;
    }

    sl::Pose zed_pose;
    sl::POSITIONAL_TRACKING_STATE tracking_state;

    auto resolution = zed.getCameraInformation().camera_configuration.resolution;
    cv::Size size(resolution.width, resolution.height);

    // OpenCV
    sl::Mat image_sl_l (size.width, size.height, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
    cv::Mat image_cv_rgba_l = slMat2cvMat(image_sl_l);
	cv::Mat image_cv_rgb_l(size.width, size.height, CV_8UC3);

    sl::Mat image_sl_r (size.width, size.height, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
    cv::Mat image_cv_rgba_r = slMat2cvMat(image_sl_r);
    cv::Mat image_cv_rgb_r(size.width, size.height, CV_8UC3);;

    int width = 800;
    int height = 400;

    cv::namedWindow("Left", CV_WINDOW_NORMAL);
	cv::moveWindow("Left", 0, 0);
	cv::resizeWindow("Left", width, height);

    cv::namedWindow("Right", CV_WINDOW_NORMAL);
	cv::moveWindow("Right", width+65, 0);
	cv::resizeWindow("Right", width, height);

    // Create 3D window
    cv::viz::Viz3d visualizer("vo_zed_viewer");
	visualizer.setBackgroundColor(cv::viz::Color::white());
    visualizer.setWindowPosition(cv::Point(0, height+45));
    visualizer.setWindowSize(cv::Size(width, height));
    visualizer.showWidget("origin", cv::viz::WCoordinateSystem());

    char key = ' ';
    int svo_frame_rate = zed.getInitParameters().camera_fps;
    int nb_frames = zed.getSVONumberOfFrames();
    int coun_zed_poses = 0;
    std::cout << "\n\n\t[Info] SVO contains " << std::to_string(nb_frames) << " frames.\n" << std::endl;

    // cv::Mat RM33(3, 3, CV_64FC1, cv::Scalar(0.0));
    // RM33.at<double>(0,0) = -1;
    // RM33.at<double>(1,1) = -1;
    // RM33.at<double>(2,2) = 1;
    // cv::Mat T31(3, 1, CV_64FC1, cv::Scalar(0.0));
    // cv::Affine3d T_zed_cv = cv::Affine3d(RM33,T31);

    cv::Affine3d T_zed_cv;
    std::vector<uint64_t> zed_timestamp;

    //Substrac the file path form argv[1] path
    std::string fullname (argv[1]);
    std::string rawname = fullname.substr(0, fullname.find_last_of("/")); 

    std::string leftPath = rawname + "/left/";
    create_directory(leftPath);
    std::string rightPath =  rawname + "/right/";
    create_directory(rightPath);

    std::ofstream poses_kitti_world;
    poses_kitti_world.open (rawname + "/poses_world.txt");

    std::ofstream poses_kitti_camera;
    poses_kitti_camera.open (rawname + "/poses_camera.txt");

    zed.setSVOPosition(0);

    while (key != 'q') 
    {
        sl::ERROR_CODE zed_error = zed.grab();

		if (zed_error == sl::ERROR_CODE::SUCCESS) 
        {
            zed.retrieveImage(image_sl_l, sl::VIEW::LEFT, sl::MEM::CPU, sl::Resolution(size.width, size.height));
            zed.retrieveImage(image_sl_r, sl::VIEW::RIGHT, sl::MEM::CPU, sl::Resolution(size.width, size.height));

            uint64_t timestamp = zed.getTimestamp(sl::TIME_REFERENCE::IMAGE).getSeconds(); // Get image timestamp        
            
            int svo_position = zed.getSVOPosition();
            
            tracking_state = zed.getPosition(zed_pose, sl::REFERENCE_FRAME::CAMERA);

            if(tracking_state == sl::POSITIONAL_TRACKING_STATE::OK)
            {
                sl::Matrix3f R33_zed = zed_pose.getRotationMatrix();
                
                cv::Matx33d R33_cv;

                for(int i = 0; i < 3; i++)
                    for(int j = 0; j < 3; j++)
                        R33_cv(i, j) = R33_zed.r[3*i + j];

                double tx = zed_pose.getTranslation().tx;
                double ty = zed_pose.getTranslation().ty;
                double tz = zed_pose.getTranslation().tz;

                poses_kitti_camera  << R33_cv(0, 0) << " " << R33_cv(0, 1) << " " << R33_cv(0, 2) << " " << tx << " "
                                    << R33_cv(1, 0) << " " << R33_cv(1, 1) << " " << R33_cv(1, 2) << " " << ty << " "
                                    << R33_cv(2, 0) << " " << R33_cv(2, 1) << " " << R33_cv(2, 2) << " " << tz << "\n";

                cv::Vec3d t_cv (tx, ty, tz);
                cv::Affine3d T_cv (R33_cv, t_cv);

                T_zed_cv = T_cv.concatenate(T_zed_cv);
                zed_timestamp.push_back(timestamp);

                printf("tvec_ZED: Tx: %.3f, Ty: %.3f, Tz: %.3f\n", T_zed_cv.translation().val[0], T_zed_cv.translation().val[1], T_zed_cv.translation().val[2]);

                poses_kitti_world << T_zed_cv.rotation().val[0] << " " << T_zed_cv.rotation().val[1] << " " << T_zed_cv.rotation().val[2] << " " << T_zed_cv.translation().val[0] << " "
                            << T_zed_cv.rotation().val[3] << " " << T_zed_cv.rotation().val[4] << " " << T_zed_cv.rotation().val[5] << " " << T_zed_cv.translation().val[1] << " "
                            << T_zed_cv.rotation().val[6] << " " << T_zed_cv.rotation().val[7] << " " << T_zed_cv.rotation().val[8] << " " << T_zed_cv.translation().val[2] << "\n";

                cv::viz::WCameraPosition axis_zed = cv::viz::WCameraPosition ( 0.1 );
                visualizer.showWidget(std::to_string(coun_zed_poses), axis_zed, T_zed_cv);

                visualizer.spinOnce(1, true);

                cv::cvtColor(image_cv_rgba_l, image_cv_rgb_l, CV_RGBA2RGB);
                cv::cvtColor(image_cv_rgba_r, image_cv_rgb_r, CV_RGBA2RGB);

                cv::imshow("Left", image_cv_rgb_l);
                cv::imshow("Right", image_cv_rgb_r);

                char base_name_l[256];
                char base_name_r[256];
                sprintf(base_name_l, "%06d.png", coun_zed_poses);
                sprintf(base_name_r, "%06d.png", coun_zed_poses);

                cv::imwrite(leftPath+base_name_l, image_cv_rgb_l);
                cv::imwrite(rightPath+base_name_r, image_cv_rgb_r);

                key = cv::waitKey(1);

                coun_zed_poses++;
            }
        }

         else if (zed_error == sl::ERROR_CODE::END_OF_SVOFILE_REACHED)
        {
            std::cout << "\n\n\tSVO end has been reached.\n" << std::endl;
            break;
        }
        else {
            std::cout << "\n\n\tGrab ZED : " << zed_error << ".\n" << std::endl;
            break;
        }
    }

    cv::destroyWindow("Left");
    cv::destroyWindow("Right");
    zed.disablePositionalTracking();
    zed.close();
    poses_kitti_world.close();
    poses_kitti_camera.close();

    while(!visualizer.wasStopped())
        visualizer.spinOnce(1, true);
        
    return 0;
}