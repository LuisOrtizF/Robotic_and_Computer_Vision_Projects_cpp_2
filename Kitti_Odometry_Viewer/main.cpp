#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <Eigen/Dense>
#include <pcl/visualization/pcl_visualizer.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer> viz_traj;
pcl::PointXYZ p_0;
Eigen::Affine3f T_k;
std::ofstream file_quaternion;
int aux = 0;

void txt2affine3f(std::vector<double> T_in, Eigen::Affine3f &T_out)
{
    T_out = Eigen::Affine3f::Identity();

    int aux=0;

    for (int row = 0; row < T_out.rows(); row++)
    {
        for (int col = 0; col < T_out.cols(); col++)
            T_out(row, col) = T_in[aux + col];

        aux+=4;
    }
}

void saveQuaternion(Eigen::Affine3f T_in)
{
    if (file_quaternion.is_open ())
    {
        // convert 3x4 affine transformation to quaternion and write to file
        Eigen::Quaternionf q (T_in.rotation ());
        Eigen::Vector3f t (T_in.translation ());
        // write translation , quaternion in a row
        file_quaternion << t[0] << "," << t[1] << "," << t[2]
                    << "," << q.w () << "," << q.x ()
                    << "," << q.y ()<< ","  << q.z () << std::endl;
    }
    else std::cout << "Unable to open file\n";
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> init_visualizator ()
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
    
    //Transformation in the time (t-1)
    T_k = Eigen::Affine3f::Identity();
    
    p_0.x = 0.0;
    p_0.y = 0.0;
    p_0.z = 0.0;

    viewer->addCoordinateSystem(1.0, "initial");
    viewer->setBackgroundColor (0.25, 0.25, 0.25);

    viewer->setPosition(720,0);	
    viewer->setShowFPS (false);	
    viewer->setSize(640,480);

    viewer->setCameraPosition(0.0,-500,300,0.0,0.0,0.0,0.0,0.0,0.0);

    //Add axes labels
    pcl::PointXYZI point;
    point.getArray3fMap () << 1.1, 0, 0;
    viewer->addText3D ("X", point, 0.1, 1, 0, 0, "x_");
    point.getArray3fMap () << 0, 1.1, 0;
    viewer->addText3D ("Y", point, 0.1, 0, 1, 0, "y_");
    point.getArray3fMap () << 0, 0, 1.1;
    viewer->addText3D ("Z", point, 0.1, 0, 0, 1, "z_");

    return (viewer);
}

void visualize_trajectory (Eigen::Affine3f T_x)
{
    //Generate a random (bright) color
    double r = (rand() % 100);
    double g = (rand() % 100);
    double b = (rand() % 100);
    double max_channel = std::max (r, std::max (g, b));
    r /= max_channel;
    g /= max_channel;
    b /= max_channel;
    
    // Generate a unique string for each CoordinateSystem
    std::stringstream ss ("frame_");
    ss << aux;

    viz_traj->addCoordinateSystem(0.15, T_x, ss.str());
    
    // Generate a unique string for each frame and line
    std::stringstream ss2 ("line_");
    ss2 << aux;

    Eigen::Vector3f pos1 = T_x.translation();
    pcl::PointXYZ p_1;

    p_1.x = pos1(0);
    p_1.y = pos1(1); //0.0
    p_1.z = pos1(2);

    viz_traj->addLine (p_0, p_1, r, g, b, ss2.str());

    p_0 = p_1;

    viz_traj->spinOnce (1);
    aux+=1;
}

int main (int argc, char** argv) 
{

    printf("\n%s\n", argv[0]);

    // we need the path name to poses as input argument
    if (argc != 2) {
        printf("\nSyntax is: %s <kitti_poses_file (.txt)>\n\n", argv[0]);
        return 1;
    }

    std::ifstream file_poses;

    file_poses.open(argv[1]);

    std::string row;
    
    if(file_poses.is_open())
    { 
        viz_traj = init_visualizator();
        file_quaternion.open ("poses_quaternion.txt");
        
        while (getline(file_poses, row))
        {
            if (row[0] != '\n' )
            {
                std::istringstream istr(row);
                double number;
                std::vector<double> t_in;
                Eigen::Affine3f t_out;

                while (istr >> number) 
                    t_in.push_back(number);
                
                txt2affine3f(t_in, t_out);
                //std::cout << t_out.matrix() << std::endl;
                saveQuaternion(t_out);
                visualize_trajectory (t_out);
            }        
        }
        file_poses.close();
    }
    else std::cout << "Unable to open file\n"; 
    
    file_quaternion.close();

    if (!viz_traj->wasStopped())
        viz_traj->spin();

    return 0;
}