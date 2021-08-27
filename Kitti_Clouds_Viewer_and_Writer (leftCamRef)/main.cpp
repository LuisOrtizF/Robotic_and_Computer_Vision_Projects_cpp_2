#include <iostream>
#include <fstream>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer> viz_cloud;

void print_help (const char* progName)
{
	pcl::console::print_info ("- You can write clouds in (.pcd) format using the flag: %s -w\n", progName);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> init_viewer ()
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("Clouds Viewer"));
    
    // Initial frame
    viewer->addCoordinateSystem(1.0);
    viewer->setBackgroundColor (0.25, 0.25, 0.25);
    viewer->setShowFPS (false);	
	viewer->setSize(800,600);
	//  pos_x, pos_y, pos_z, view_x, view_y, view_z, up_x, up_y, up_z	
	viewer->setCameraPosition(0.0,-400.0,0.1,0.0,0.0,0.0,0.0,0.0,0.0);

    //Add axes labels
    pcl::PointXYZRGB point;
    point.getArray3fMap () << 1.1, 0, 0;
    viewer->addText3D ("X", point, 0.1, 1, 0, 0, "x_");
    point.getArray3fMap () << 0, 1.1, 0;
    viewer->addText3D ("Y", point, 0.1, 0, 1, 0, "y_");
    point.getArray3fMap () << 0, 0, 1.1;
    viewer->addText3D ("Z", point, 0.1, 0, 0, 1, "z_");

    return (viewer);
}

int main(int argc, char **argv)
{
	pcl::console::print_info ("\nClouds Viewer and Clouds  Writer (.pcd) for Kitti Dataset\n\n");
	
	if (argc != 4)
	{
	    printf("\nUsage: %s <times_file_path (.txt)> <clouds_sequence_path (.bin)> <max_frame> <write_flag>\n\n", argv[0]);
	    return 1;
	}

	//Input Cloud filename
	std::string in_filename;
	char numstr[21];

	//PCL Cloud
	pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr points_leftcam (new pcl::PointCloud<pcl::PointXYZI>);
	
	std::string id_cloud ("id_cloud_");
	//Initialize the viewer
	viz_cloud = init_viewer();

	std::string dir_times = argv[1];
	std::string times_file_path = dir_times+"times.txt";

	// Load time.txt
	std::fstream times(times_file_path.c_str());
	std::string line;

	//Output PCD filename
	std::string out_filename;
	std::string line2;

	if(!times.good()){
		std::cerr << "Could not read file: 'times.txt'" << std::endl;
		print_help(argv[0]);
		return (-1);
	}

	std::string clouds_sequence_path = argv[2];
	int MAX_FRAME = std::atoi(argv[3]);
	
	for (int j = 0 ; j < (MAX_FRAME); j++)
	{
		// Filename to read
		sprintf(numstr, "%06d", j);
		in_filename = clouds_sequence_path + std::string(numstr) + ".bin";

		// Load Cloud
		std::fstream input(in_filename.c_str(), std::ios::in | std::ios::binary);
		
		if(!input.good()){
			std::cerr << "Could not read file: " << in_filename << std::endl;
			print_help(argv[0]);
			return (-1);
		}

		input.seekg(0, std::ios::beg);

		// Convert .bin to PCL PointXYZI Cloud
		int i;
		for (i=0; input.good() && !input.eof(); i++) {
			pcl::PointXYZI point;
			input.read((char *) &point.x, 3*sizeof(float));
			input.read((char *) &point.intensity, sizeof(float));
			points->push_back(point);
		}

		Eigen::Affine3f Tr = Eigen::Affine3f::Identity();

		Tr.translation() << -0.01198459927713, -0.05403984729748, -0.2921968648686;
		Tr.linear() << 0.0004276802385584, -0.9999672484946  , -0.008084491683471, 
					   -0.007210626507497, 0.008081198471645, -0.9999413164504  , 
					   0.9999738645903   , 0.0004859485810390, -0.007206933692422;
		pcl::transformPointCloud (*points, *points_leftcam, Tr);

		// Close .bin file
		input.close();
		// Clear .bin path
		in_filename.clear();

		//Unique ID for each PCL Cloud
		id_cloud = std::string(numstr);
		//Get intensity field 
		pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> rgb(points_leftcam, "intensity");
		//Add Cloud to viewer 
		viz_cloud->addPointCloud<pcl::PointXYZI> (points_leftcam, rgb, id_cloud);
			
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
			// Filename to write
			out_filename = "sequence_clouds/" + id_cloud + ".pcd";
			line2 = "Save: " + out_filename;
			viz_cloud->addText (line2, 350, 10, 20, 1.0, 1.0, 1.0, out_filename);
			pcl::PCDWriter writer;
			// Save DoN features
			writer.write<pcl::PointXYZI> (out_filename, *points_leftcam, false);
		}

		//(1100[frames]/114,0288[seg]) = 9,64668575 [fps]
	    //(114,0288[seg]/1100[frames])*1000[ms]/1[seg] = 103,662545455 [ms per frame] 
		viz_cloud->spinOnce (103.662545455);
		viz_cloud->removePointCloud (id_cloud);
		viz_cloud->removeShape (id_cloud);
		viz_cloud->removeShape (out_filename);
		
		points->clear();
		points_leftcam->clear();
		id_cloud.clear();
		out_filename.clear();
		line2.clear();
	}

	times.close();
	viz_cloud->close();

	return (-1);
}