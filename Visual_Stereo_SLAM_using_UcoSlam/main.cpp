#include <string>
#include <vector>
#include <iostream>
#include <iomanip>

#include <ucoslam/ucoslam.h>
#include <ucoslam/map.h>
#include <ucoslam/mapviewer.h>
#include <ucoslam/stereorectify.h>
#include <ucoslam/basictypes/debug.h>
#include <ucoslam/basictypes/cvversioning.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Dense>

std::map<int32_t, double> getKittiTimestamps (std::string file)
{
    std::ifstream dir(file.c_str());
    std::string row;
    std::map<int32_t, double> timestamps;
    if (dir.is_open())
    {
        int32_t i = 0;
        while (getline(dir, row))
        {
            timestamps.insert(std::make_pair(i,std::stod(row)));
            i++;
        }
        dir.close(); 
    }
    else
    {
        printf("\n\tUnable to open kitti times.txt file!\n\n");
        exit(EXIT_FAILURE);
    }
    return timestamps;
}

Eigen::Quaterniond cvAffine2egQuat (cv::Affine3d cv_mat)
{
    cv::Matx33d R33_cv = cv_mat.rotation();
    Eigen::Matrix3d R33_eg;
    for(int32_t i = 0; i < 3; i++)
        for(int32_t j = 0; j < 3; j++) 
            R33_eg(i,j) = R33_cv(i,j);
    Eigen::Quaterniond q (R33_eg);
    return q;
}

void saveTUMPoses (std::string file, std::map<int32_t, cv::Affine3d> T_total, std::map<int32_t, double> Timestamps)
{
    std::fstream outFile(file.c_str(), std::fstream::out);
    if (outFile.is_open())
    {
        for (std::map<int32_t, cv::Affine3d>::iterator k = T_total.begin(); k != T_total.end(); k++)
        {
            Eigen::Quaterniond q = cvAffine2egQuat(k->second);
            Eigen::Vector3d t(k->second.translation().val[0],
                                k->second.translation().val[1],
                                k->second.translation().val[2]);
            double time = Timestamps[k->first];
            outFile << std::fixed << std::setprecision(10) << time << " " 
                    << t[0] << " " << t[1] << " " << t[2]<< " " 
                    << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n"; 
        }
        outFile.close();
    }
    else
    {
        printf( "\n\tUnable to open file to save poses!\n\n" );
        exit(EXIT_FAILURE);
    }
}

int main(int argc, char* argv[])
{
    if(argc != 8)
    {
        std::cout<<"\nUsage:" << argv[0] << "<video_left> <video_right> <stereo_calibration_file> <kitti_times_file> <output_poses> <voc_path> <ucoslam_params>\n" << std::endl;
        return -1;
    }

    ucoslam::StereoRectify Rectifier;
    Rectifier.readFromXMLFile(argv[3]);

    cv::Mat inImage[2];
    cv::VideoCapture video[2];
    for(int i=0;i<2;i++)
    {
        video[i].open(argv[i+1]);
        if(!video[i].isOpened())
            throw std::runtime_error(std::string("Cannot open video file at:")+argv[i+1]);
    }

    ucoslam::Params sparams;
    sparams.readFromYMLFile(argv[7]);

    std::shared_ptr<ucoslam::Map> smap = std::make_shared<ucoslam::Map>();

    ucoslam::UcoSlam system;
    ucoslam::MapViewer mv;
    system.setParams(smap,sparams,argv[6]);
    
    char key=0;

    int countUcoPoses = 0;
    std::map<int32_t, cv::Affine3d> ucoPoses;

    ucoPoses.insert(std::make_pair(0,cv::Affine3d::Identity()));

    std::map<int32_t, double> Timestamps = getKittiTimestamps(argv[4]);

    while( video[0].grab() && video[1].grab() && key!=27)
    {
        int frameNumber=video[0].get(CV_CAP_PROP_POS_FRAMES);
        video[0].retrieve(inImage[0]);
        video[1].retrieve(inImage[1]);
        // Rectifier.rectify(inImage[0],inImage[1]);
        cv::Mat Tr=system.processStereo(inImage[0],inImage[1],Rectifier.getImageParams(),frameNumber );
        key=mv.show(smap,inImage[0],Tr,"");

        if(!Tr.empty())
        {
            cv::Matx33d R33_uco;
            for(int i = 0; i < 3; i++)
                for(int j = 0; j < 3; j++)
                    R33_uco(i,j) = Tr.at<float>(i, j);
            double tx_uco = Tr.at<float>(0, 3);
            double ty_uco = Tr.at<float>(1, 3);
            double tz_uco = Tr.at<float>(2, 3);
            cv::Vec3d t_uco (tx_uco, ty_uco, tz_uco);
            cv::Affine3d T_uco_aux (R33_uco, t_uco);

            T_uco_aux = T_uco_aux.inv().concatenate(ucoPoses[0]);

            ucoPoses.insert(std::make_pair(frameNumber,T_uco_aux)); 

            std::cout   << "Frame: " << frameNumber <<  ", trans: " << T_uco_aux.translation() << std::endl;
        }
    }

    saveTUMPoses(argv[5], ucoPoses, Timestamps);

    return 0;
}