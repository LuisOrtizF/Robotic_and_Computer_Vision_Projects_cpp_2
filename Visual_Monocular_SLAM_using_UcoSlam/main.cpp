#include <iostream>
#include <ucoslam/ucoslam.h>
#include <ucoslam/mapviewer.h>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <fstream>

int main(int argc,char **argv)
{
    try 
    {
        if(argc!=5) throw std::runtime_error("Usage : ./ucoslam_monocular <video> <camera_parameters> <vocabulary> <output_map>");

        cv::VideoCapture VideoIn;//video capturer
        ucoslam::ImageParams cameraParams;//camera parameters
        ucoslam::Params UcoSlamParams;//processing parameters
        ucoslam::UcoSlam SLAM;//The main class
        ucoslam::MapViewer MapViwer;//Viewer to see the 3D map and the input images

        //creates an empty map
        std::shared_ptr<ucoslam::Map> map = std::make_shared<ucoslam::Map>();
        //open video
        VideoIn.open(argv[1]);

        if(!VideoIn.isOpened()) throw std::runtime_error("Could not open video:"+string(argv[1]));

        //read camera params
        cameraParams.readFromXMLFile(argv[2]);
        //set the slam params for Kitti using orb descriptor
        UcoSlamParams.runSequential=true;//run in sequential mode to avoid skipping frames
        UcoSlamParams.detectMarkers=false;//no markers in this example.

        //Start UcoSlam
        SLAM.setParams(map,UcoSlamParams,argv[3]);//the last parameter is the path to the vocabulary file of extension .fbow
        
        std::fstream poses, frames, points3D, points2D;

        poses.open("../Data/poses_Kitti.txt", std::fstream::out);
        // frames.open("../Data/frames.txt", std::fstream::out);
        // points3D.open("../Data/points3D.txt", std::fstream::out);
        // points2D.open("../Data/points2D.txt", std::fstream::out);
        
        //if (poses.fail() || frames.fail() || points3D.fail() || points2D.fail() )
        if (poses.fail() )
            std::cout<<"Can't open file" << std::endl;

        cv::Mat inputImage;
        char keyPressed=0;

        // INITAL TRANFORMATION
        cv::Matx33d R33_cv;
        R33_cv(0,0) = 1.000000e+00; 
        R33_cv(0,1) = 1.197625e-11; 
        R33_cv(0,2) = 1.704638e-10; 
        R33_cv(1,0) = 1.197625e-11; 
        R33_cv(1,1) = 1.000000e+00; 
        R33_cv(1,2) = 3.562503e-10; 
        R33_cv(2,0) = 1.704638e-10; 
        R33_cv(2,1) = 3.562503e-10; 
        R33_cv(2,2) = 1.000000e+00; 

        cv::Vec3d t (5.551115e-17, 0.000000e+00, 2.220446e-16);
        
        cv::Affine3d T_uco(R33_cv, t);

        while( VideoIn.grab() && keyPressed != 27)
        {//keyPressed ==27 is esc
            
            VideoIn.retrieve(inputImage);
            int frameNumber = VideoIn.get(CV_CAP_PROP_POS_FRAMES);

            cv::Mat posef2g = SLAM.process(inputImage, cameraParams, frameNumber);

            // if(!posef2g.empty())
            // {
            // cv::Matx33d R33_uco;
            // for(int i = 0; i < 3; i++)
            //     for(int j = 0; j < 3; j++)
            //         R33_uco(i,j) = posef2g.at<float>(i, j);
            // double tx_uco = posef2g.at<float>(0, 3);
            // double ty_uco = posef2g.at<float>(1, 3);
            // double tz_uco = posef2g.at<float>(2, 3);
            // cv::Vec3d t_uco (tx_uco, ty_uco, tz_uco);
            // cv::Affine3d T_uco_aux (R33_uco, t_uco);
            // T_uco_aux = T_uco_aux.inv().concatenate(T_uco);
            // }

            //draw a mininimal interface in an opencv window
            keyPressed = MapViwer.show(map, inputImage, posef2g);
        }

        //now, save the map
        map->saveToFile(argv[4]);

        for(const auto &Kframe : map->keyframes)
        {
            //frames << Kframe.idx << "\n";

            cv::Point3f ttt = Kframe.getCameraCenter();
            cv::Point3f rrr = Kframe.getCameraDirection();

            cv::Vec3d rr (rrr.x, rrr.y, rrr.z);
            cv::Vec3d tt (ttt.x ,ttt.y, ttt.z);

            // ucoslam::Se3Transform p = Kframe.pose_f2g;

            // cv::Mat Rvec = p.getRvec();
            // cv::Point3f t_vec = p.getT();

            // cv::Vec3d rr (Rvec.at<float>(0,0), Rvec.at<float>(0,1), Rvec.at<float>(0,2));
            // cv::Vec3d tt (t_vec.x ,t_vec.y, t_vec.z);

            cv::Affine3d T_uco_aux(rr,tt);

            //T_uco_aux = T_uco_aux.inv();

            int aux = 0;
            
            for(int i = 0; i < 3; i++)
                for(int j = 0; j < 4; j++){
                    poses << T_uco_aux.matrix.val[4*i+j];
                    if( aux < 11)               
                        poses << " ";
                    aux++; 
                }

            poses << "\n";

            // //iterate through the ids.
            // for(size_t i = 0; i < Kframe.ids.size(); i++)
            // {
            //     if(  Kframe.ids[i] != std::numeric_limits<uint32_t>::max() )
            //     {
            //         uint32_t mapId = Kframe.ids[i];
            //         points3D << map->map_points[mapId].getCoordinates() << " ";

            //         cv::Point2f pixel = Kframe.und_kpts[mapId].pt;
            //         points2D << pixel << " ";                   
            //     }
            // }
            // points3D << "\n";
            // points2D << "\n";
        }

        poses.close();
        // frames.close();
        // points3D.close();
        // points2D.close();

    } 
    catch (std::exception &ex) 
    {
        std::cout<<ex.what()<<std::endl;
    }
}