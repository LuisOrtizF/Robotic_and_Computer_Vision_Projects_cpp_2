#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <fstream>

#define FLT_EPSILON __FLT_EPSILON__

boost::shared_ptr<pcl::visualization::PCLVisualizer> viz_cloud;

boost::shared_ptr<pcl::visualization::PCLVisualizer> init_viewer ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("Clouds Viewer"));

  // Initial frame
  viewer->addCoordinateSystem(1.0);
  viewer->setBackgroundColor (0.5, 0.5, 0.5);
  viewer->setShowFPS (false);	
  viewer->setSize(800,600);
  viewer->setPosition(900,0);
  //pos_x, pos_y, pos_z, view_x, view_y, view_z, up_x, up_y, up_z	
  //viewer->setCameraPosition(0.0,-400.0,0.1,0.0,0.0,0.0,0.0,0.0,0.0);

  //Add axes labels
  pcl::PointXYZRGB point;
  point.getArray3fMap () << 1.1, 0, 0;
  viewer->addText3D ("X", point, 0.1, 1, 0, 0, "x_");
  point.getArray3fMap () << 0, 1.1, 0;
  viewer->addText3D ("Y", point, 0.1, 0, 1, 0, "y_");
  point.getArray3fMap () << 0, 0, 1.1;
  viewer->addText3D ("Z", point, 0.1, 0, 0, 1, "z_");

  viewer->initCameraParameters();

  return (viewer);
}


int main(int argc, char **argv) 
{
  pcl::console::print_info ("\nConvert Kitti Stereo Images to PCL Point Clouds\n\n");

  if (argc != 3)
	{
    pcl::console::print_info ("\nUsing: %s <sequence_path> <num_frames> <save_flag (optional)>\n\n", argv[0]);
    pcl::console::print_info ("\nYou can write the clouds in '.pcd' format using the flag '-w'\n");
  	return 1;
	}

  int MAX_FRAME = std::atoi(argv[2]);

  std::string left_path, rigth_path;
  char file_number[21];

  cv::Mat left_rgb, rigth_gray, left_gray, disparity, true_disparity, viz_disparity;

  int image_channels = left_gray.channels();  
  int minDisparity = 0;
  int numDisparities = 64-minDisparity;// # divisible for 16
  int blockSize = 11;// 3 to 11
  int P1 = 8*image_channels*blockSize*blockSize;
  int P2 = 32*image_channels*blockSize*blockSize;
  int disp12MaxDiff = 1;  
  int preFilterCap = 63;
  int uniquenessRatio = 15;// 5 to 15
  int speckleWindowSize = 200;// 50 to 200
  int speckleRange = 2;// 1 or 2
  //mode = MODE_SGBM_3WAY or MODE_SGBM or MODE_HH or MODE_HH4 

  cv::namedWindow("Left", cv::WINDOW_NORMAL);
  cv::moveWindow("Left", 0, 0);
  cv::resizeWindow("Left", 800, 400);
  
  cv::namedWindow("Disparity", cv::WINDOW_NORMAL);
  cv::moveWindow("Disparity", 0, 470);
  cv::resizeWindow("Disparity", 800, 400);

  // Matrix Q (perspective transformation matrix)
  // From KITTI VO-DATASET SEQUENCE 01 LEFT AND RIGTH 3X4 PROJECTION MATRIX (color cameras)
  cv::Mat Q = (cv::Mat_<float>(4,4) <<1, 0, 0,       -607.1928, 
                                      0, 1, 0,       -185.2157,
                                      0, 0, 0,       718.856,
                                      0, 0, -1/0.54, 0);

  cv::Mat cloud_xyz;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgbxyz (new pcl::PointCloud<pcl::PointXYZRGB>);
  const float max_z = 50;
  uchar r, g, b;

  std::string id_cloud ("cloud_");

  std::string sequence_path = argv[1];
  std::string times_path = sequence_path+"times.txt";

  // Load time.txt
	std::fstream times(times_path.c_str());
  std::string line;
  if(!times.good()){
		std::cerr << "Could not read file: 'times.txt'" << std::endl;
		return (-1);
	}
  
  //Initialize the viewer
  viz_cloud = init_viewer();
  
  std::string info_text;
  std::string out_filename;

  for (int i = 0 ; i < MAX_FRAME; i++)
  {
    sprintf(file_number, "%06d", i);
    left_path = sequence_path+"image_0/" + std::string(file_number) + ".png";
    rigth_path = sequence_path+"image_1/" + std::string(file_number) + ".png";

    left_rgb = cv::imread(left_path, cv::IMREAD_COLOR ); // 3 channel BGR color image
    rigth_gray = cv::imread(rigth_path, cv::IMREAD_GRAYSCALE );

    if(left_rgb.empty()){
			std::cerr << "Could not read file: " << left_path << std::endl;
			return (-1);
    }
    
    if(rigth_gray.empty()){
			std::cerr << "Could not read file: " << rigth_path << std::endl;
			return (-1);
    }

    cv::cvtColor(left_rgb, left_gray, cv::COLOR_BGR2GRAY);

    cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(
        minDisparity, numDisparities, blockSize, 
        P1, P2, disp12MaxDiff, preFilterCap,  
        uniquenessRatio, speckleWindowSize, 
        speckleRange, cv::StereoSGBM::MODE_SGBM_3WAY
    );

    //intput: left and rigth = CV_8U - 8-bit unsigned integers ( 0..255 )
    //output: disparity = CV_16US  - 16-bit unsigned integers ( 0..65535 )
    stereo->compute( left_gray, rigth_gray, disparity);

    //Convert to real disparity
    //CV_32F - 32-bit floating-point numbers
    disparity.convertTo(true_disparity, CV_32F, 1.0/16.0, 0.0);

    //Only for visualization
    cv::imshow( "Left", left_rgb );
    cv::normalize( disparity, viz_disparity, 0, 256, cv::NORM_MINMAX, CV_8U );
    cv::imshow( "Disparity", viz_disparity );

    //std::cout << left_gray.type() << "," <<rigth_gray.type() << "," << true_disparity.type() << std::endl;    

    cv::waitKey(1);

    // Both images must be same size
    if (left_gray.size() != true_disparity.size())
    {
      std::cerr << "ERROR: left_image and disparity_image have different sizes." << std::endl;
      return (-1);
    }

    cv::reprojectImageTo3D(true_disparity, cloud_xyz, Q, true);

    for(int i = 0; i < cloud_xyz.rows; i++)
    {
      for(int j = 0; j < cloud_xyz.cols; j++)
      {
        cv::Vec3f point = cloud_xyz.at<cv::Vec3f>(i, j);
        cv::Vec3b color = left_rgb.at<cv::Vec3b>(i, j);

        if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;

        pcl::PointXYZRGB point3D;

        point3D.x = point[0];
        point3D.y = point[1];
        point3D.z = point[2];
        b = color[0];
        g = color[1];
        r = color[2];
        
        uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                        static_cast<uint32_t>(g) << 8 | 
                        static_cast<uint32_t>(b));

        point3D.rgb = *reinterpret_cast<float*>(&rgb);

        cloud_rgbxyz->points.push_back (point3D);
      }
    }

    cloud_rgbxyz->width = (int) cloud_rgbxyz->points.size ();
    cloud_rgbxyz->height = 1; 

    //Unique ID for each PCL Cloud
    id_cloud = std::string(file_number);
    //Add Cloud to viewer 
    viz_cloud->addPointCloud(cloud_rgbxyz, id_cloud);
      
    //Add times into Viewer
    if (times.is_open())
    {
      getline (times,line);
      line = "Time: "+line+" s";
      viz_cloud->addText (line, 10, 10, 20, 1.0, 1.0, 1.0, id_cloud);
    }

    //Write PCL Clouds in .pcd format
    if(pcl::console::find_argument (argc, argv, "-w") != -1)
		{
      out_filename = "sequence_clouds/" + std::string(file_number) + ".pcd";
      info_text = "Save: " + out_filename;
      viz_cloud->addText (info_text, 350, 10, 20, 1.0, 1.0, 1.0, out_filename);

      pcl::PCDWriter writer;
      writer.write<pcl::PointXYZRGB> (out_filename, *cloud_rgbxyz, false);
    }

    if (!viz_cloud->wasStopped()){
      viz_cloud->spinOnce (103.662545455);
    }
    else{
      times.close();
      viz_cloud->close();
      return (-1);
    }
		viz_cloud->removePointCloud (id_cloud);
    viz_cloud->removeShape (id_cloud);
    viz_cloud->removeShape (out_filename);

    cloud_rgbxyz->clear();
    id_cloud.clear();
    out_filename.clear();
		info_text.clear();
  }

	times.close();
  viz_cloud->close();
  
  return (0);
}