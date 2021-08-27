#include <iostream>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>


//int main(int argc, char **argv) {

//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ> ());
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ> ());

//    cloud_in->width = 4;
//    cloud_in->height = 1;
//    cloud_in->is_dense = false;
//    cloud_in->resize(cloud_in->width * cloud_in->height);

//    cloud_out->width = 4;
//    cloud_out->height = 1;
//    cloud_out->is_dense = false;
//    cloud_out->resize(cloud_out->width * cloud_out->height);

//    cloud_in->points[0].x = 0;
//    cloud_in->points[0].y = 0;
//    cloud_in->points[0].z = 0;

//    cloud_in->points[1].x = 2;
//    cloud_in->points[1].y = 0;
//    cloud_in->points[1].z = 0;

//    cloud_in->points[2].x = 0;
//    cloud_in->points[2].y = 2;
//    cloud_in->points[2].z = 0;

//    cloud_in->points[3].x = 0;
//    cloud_in->points[3].y = 0;
//    cloud_in->points[3].z = 2;

//    // make a translation
//    Eigen::Vector3f trans;
//    trans << 4.0,4.0,4.0;

//    std::vector<int> indices(cloud_in->points.size());

//    for (int i=0;i<cloud_in->points.size();i++)
//    {
//        indices[i] = i+1;
//        cloud_out->points[i].x = cloud_in->points[i].x + trans(0);
//        cloud_out->points[i].y = cloud_in->points[i].y + trans(1);
//        cloud_out->points[i].z = cloud_in->points[i].z + trans(2);
//        std::cout << cloud_out->points[i].x << "—" << cloud_out->points[i].y << "—" << cloud_out->points[i].z << std::endl;
//    }

//    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> TESVD;
//    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4 transformation2;
//    TESVD.estimateRigidTransformation (*cloud_in,indices,*cloud_out,transformation2);

//    std::cout << "The Estimated Rotation and translation matrices (using getTransformation function) are : \n" << std::endl;
//    printf ("\n");
//    printf ("    | %6.3f %6.3f %6.3f | \n", transformation2 (0,0), transformation2 (0,1), transformation2 (0,2));
//    printf ("R = | %6.3f %6.3f %6.3f | \n", transformation2 (1,0), transformation2 (1,1), transformation2 (1,2));
//    printf ("    | %6.3f %6.3f %6.3f | \n", transformation2 (2,0), transformation2 (2,1), transformation2 (2,2));
//    printf ("\n");
//    printf ("t = < %0.3f, %0.3f, %0.3f >\n", transformation2 (0,3), transformation2 (1,3), transformation2 (2,3));


//    return 0;
//}

// Multiple Point Clouds Visualizer

boost::shared_ptr<pcl::visualization::PCLVisualizer> MultiViewPorts(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_ic,
                                                                    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_cc,
                                                                    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_res)
{
    // Open 3D viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Visualizer"));

    // Create three separated viewports
    int v1 (0), v2 (1), v3 (2);

    // View a Ideal Cube Point Cloud
    viewer->createViewPort (0.0, 0.5, 0.5, 1, v1);
    viewer->setBackgroundColor(0.1, 0.1, 0.1, v1);
    viewer->addText("Ideal Point Cloud", 10, 10, "v1 text", v1);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(cloud_ic);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_ic, rgb1,"cloud1", v1);

    // View a Captured Cube Point Cloud
    viewer->createViewPort (0.5, 0.5, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0.05, 0.05, 0.05, v2);
    viewer->addText("Captured Point Cloud", 10, 10, "v2 text", v2);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(cloud_cc);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_cc, rgb2,"cloud2", v2);

    //View a result of the ICP Algorithm
    viewer->createViewPort (0.0, 0.0, 1.0, 0.5, v3);
    viewer->setBackgroundColor(0.0, 0.0, 0.0, v3);
    viewer->addText("ICP Algorithm Result", 10, 10, "v3 text", v3);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb3(cloud_res);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_res, rgb3,"cloud3",   v3);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_ic, rgb1,"cloud1v3",  v3);

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5,"cloud1");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5,"cloud2");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5,"cloud3");
    viewer->addCoordinateSystem(1.0);

    viewer->initCameraParameters();
    viewer->setSize(700,700);
    viewer->setPosition(800,0);

    return (viewer);
}

int main(int argc, char **argv) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ideal_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("ideal_cloud.pcd", *ideal_cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file 'ideal_cloud.pcd'.\n");
      return (-1);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr actual_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("actual_cloud.pcd", *actual_cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file 'actual_cloud.pcd'.\n");
      return (-1);
    }

    pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB,pcl::PointXYZRGB> TESVD;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB,pcl::PointXYZRGB>::Matrix4 transformation2;
    TESVD.estimateRigidTransformation (*actual_cloud,*ideal_cloud,transformation2);

    cout << "\n|R|t| Matrix" << endl;
    printf ("\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", transformation2 (0,0), transformation2 (0,1), transformation2 (0,2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", transformation2 (1,0), transformation2 (1,1), transformation2 (1,2));
    printf ("    | %6.3f %6.3f %6.3f | \n", transformation2 (2,0), transformation2 (2,1), transformation2 (2,2));
    printf ("\n");
    printf ("t = | %6.3f, %6.3f, %6.3f | \n", transformation2 (0,3), transformation2 (1,3), transformation2 (2,3));
    printf ("\n");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final(new pcl::PointCloud<pcl::PointXYZRGB> ());

    pcl::transformPointCloud(*actual_cloud, *Final, transformation2);

    double dist,dist_x,dist_y, dist_z,valx,valy, valz, aux2, erms;
    double values= 0;

    for (size_t i = 0; i < Final->points.size (); i++)
    {
        valx=Final->points[i].x-ideal_cloud->points[i].x;
        valy=Final->points[i].y-ideal_cloud->points[i].y;
        valz=Final->points[i].z-ideal_cloud->points[i].z;

        dist_x = pow(valx,2);
        dist_y = pow(valy,2);
        dist_z = pow(valz,2);

        dist = dist_x+dist_y+dist_z;

        values+=dist;

    }

    aux2=values/Final->points.size ();
    erms=pow(aux2,0.5);

    cout <<"RMS Error= "<<erms<<"\n"<<endl;

    //Visualizate point clouds

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer; //Create visualizer

    viewer = MultiViewPorts(ideal_cloud, actual_cloud, Final);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));

    }

    return 0;
}
