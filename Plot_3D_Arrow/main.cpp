#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

int main(int argc, char** argv)
{
  //MATRIZ INICAL
  Eigen::Affine3f t_0 = Eigen::Affine3f::Identity();
  // Define a translation.
  t_0.translation() << 0.0, 0.0, 0.0;
  // 0 radians arround Y axis
  t_0.rotate (Eigen::AngleAxisf (0.0, Eigen::Vector3f::UnitY()));
  //Get the pair of points
  pcl::PointXYZ p_0;
  p_0.x = 0.0;
  p_0.y = 0.0;
  p_0.z = 0.0;

  pcl::visualization::PCLVisualizer viz ("3D Trajectory");
  viz.addCoordinateSystem(1.0, t_0, "f_inicial");
  viz.setBackgroundColor (0.5, 0.5, 0.5);
  viz.setShowFPS (false);	
  viz.setSize(640,480);
  viz.setPosition(0,0);

  //Add axes labels
  pcl::PointXYZRGBA point;
  point.getArray3fMap () << 1.25, 0, 0;
  viz.addText3D ("X", point, 0.1, 1, 0, 0, "x_");
  point.getArray3fMap () << 0, 1.25, 0;
  viz.addText3D ("Y", point, 0.1, 0, 1, 0, "y_");
  point.getArray3fMap () << 0, 0, 1.25;
  viz.addText3D ("Z", point, 0.1, 0, 0, 1, "z_");

  viz.setCameraPosition(0,0,10.0,0.0,0.0,0.0,0.0,0.0,0.0);

  // MATRIZ 2
  Eigen::Matrix4f t_x = Eigen::Matrix4f::Identity();
  float theta = M_PI;
  t_x (0,0) = cos (theta);
  t_x (0,1) = -sin(theta);
  t_x (1,0) = sin (theta);
  t_x (1,1) = cos (theta);
  t_x (0,3) = 1.0;
  t_x (1,3) = 1.0;
  t_x (2,3) = 1.0;

  Eigen::Vector3f t_x_1 = t_x.block<3, 1>(0, 3);
  pcl::PointXYZ p_x;
  p_x.x = t_x_1(0);
  p_x.y = t_x_1(1);
  p_x.z = t_x_1(2);

  Eigen::Affine3f t_x_x;
  t_x_x.matrix() = t_x;

  std::stringstream ss ("frame_");
  std::stringstream ss2 ("arrow_");

  for( size_t i = 0 ; i<1; ++i)
  {
    //Generate a random (bright) color
    double r = (rand() % 100);
    double g = (rand() % 100);
    double b = (rand() % 100);
    double max_channel = std::max (r, std::max (g, b));
    r /= max_channel;
    g /= max_channel;
    b /= max_channel;

    // Generate a unique string for each frame and arrow
    ss << i;
    viz.addCoordinateSystem(0.1, t_x_x, ss.str());

    // Generate a unique string for each frame and arrow
    ss2 << i;
    viz.addArrow (p_0, p_x, r, g, b, r, g, b, ss2.str () );
    //p_0 = p_x;
  }
  
  // Give control over to the visualizer
  viz.spin ();
}