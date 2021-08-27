#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv) {

    std::cout << "\nZED Capture Data Process Times Analisys"<< std::endl;

    if(argc != 3)
    {
        printf("\nSyntax is: %s <VGA or HD720 or HD1080 or HD2K> <15 or 30 or 60 or 100> \n\n", argv[0]);
        std::cout << "Remember: VGA mode have 15, 30, 60 and 100 fps\n" 
                  << "          HD720 mode have 15, 30, 60 fps\n"
                  << "          HD1080 mode have 15, 30 fps\n"
                  << "          HD2K mode have only 15 fps\n" << std::endl;
        return 1;
    }

    // Create a ZED camera object
    sl::Camera zed;

    // Set configuration parameters
    sl::InitParameters init_params;

    if (std::string(argv[1]) == "VGA") init_params.camera_resolution = sl::RESOLUTION::VGA;
    else if (std::string(argv[1]) == "HD720") init_params.camera_resolution = sl::RESOLUTION::HD720;
    else if (std::string(argv[1]) == "HD1080") init_params.camera_resolution = sl::RESOLUTION::HD1080;
    else if (std::string(argv[1]) == "HD2K") init_params.camera_resolution = sl::RESOLUTION::HD2K;
    else return 1;

    init_params.camera_fps = atoi(argv[2]);
    init_params.depth_mode = sl::DEPTH_MODE::QUALITY;
    init_params.coordinate_units = sl::UNIT::METER;

    // Open the camera
    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS){
        std::cout << "\n\n\tCamera Open " << err << " Exit program.\n" << std::endl;
        zed.close();
        return 1;
    }

    // Camera Resolution
    sl::Resolution image_size = zed.getCameraInformation().camera_configuration.resolution;

    int j = 0;

    sl::Mat left_image_zed, right_image_zed, depth_map_zed, point_cloud_zed;
    double t_i, t_d, t_p, aux_i, aux_d, aux_p;

    int depth_aux_width = atoi(argv[2]);
    int depth_aux_height = image_size.width*image_size.height;

    cv::Mat depth_aux (depth_aux_height, depth_aux_width, CV_32F);

    int aux0 = 0;

    cv::Mat depth_mean (image_size.height, image_size.width, CV_32F);
    cv::Mat depth_var (image_size.height, image_size.width, CV_32F);

    while (j < atoi(argv[2])) {

        if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
 
            // Retrieve the left image, depth image in half-resolution
            auto t_i_i = std::chrono::system_clock::now();
                zed.retrieveImage(left_image_zed, sl::VIEW::LEFT);
                zed.retrieveImage(right_image_zed, sl::VIEW::RIGHT);
            auto t_i_f = std::chrono::system_clock::now();
            t_i = std::chrono::duration_cast<std::chrono::microseconds>(t_i_f - t_i_i).count();

            auto t_d_i = std::chrono::system_clock::now();
                zed.retrieveMeasure(depth_map_zed, sl::MEASURE::DEPTH);
            auto t_d_f = std::chrono::system_clock::now();
            t_d = std::chrono::duration_cast<std::chrono::microseconds>(t_d_f - t_d_i).count();

            auto t_p_i = std::chrono::system_clock::now();
                zed.retrieveMeasure(point_cloud_zed, sl::MEASURE::XYZRGBA);
            auto t_p_f = std::chrono::system_clock::now();
            t_p = std::chrono::duration_cast<std::chrono::microseconds>(t_p_f - t_p_i).count();
            
            aux_i += t_i;
            aux_d += t_d;
            aux_p += t_p;

            for(int y = 0; y < image_size.height; y++)
            {
                for(int x = 0; x < image_size.width; x++)
                {
                    float depth_value = 0;
                    depth_map_zed.getValue(x, y, &depth_value);
                    //if(depth_value != depth_value)
                    //    depth_value = 0;
                    depth_aux.at<float>(aux0+x,j) = depth_value;
                }
                aux0+=image_size.width;    
            }

            j++;
            aux0 = 0;

        } else sl::sleep_ms(1);
    }

    zed.close();

    // TIME ANALISYS 

    std::cout << "MedianImageCapture (ms) " << "MedianDepthProcess (ms) " << "MedianCloudProcess (ms)" << std::endl;
    std::cout << (aux_i/atoi(argv[2]))/1000 << "\t\t" << (aux_d/atoi(argv[2]))/1000 << "\t\t" << (aux_p/atoi(argv[2]))/1000 << std::endl;

    // DEPTH MEAN and VARIANDE ANALISYS

    float aux1 = 0;
    std::vector<float> aux2(depth_aux_height);
    
    std::ofstream depth_mean_txt;
    depth_mean_txt.open("depth_mean_"+std::string(argv[1])+"_"+std::string(argv[2])+".txt");

    float aux3 = 0;
    std::vector<float> aux4(depth_aux_height);

    std::ofstream depth_var_txt;
    depth_var_txt.open("depth_var_"+std::string(argv[1])+"_"+std::string(argv[2])+".txt");

    for (int r = 0; r < depth_aux_height; r++)
    {
        //MEAN
        for (int c = 0; c < depth_aux_width; c++)
            aux1 += depth_aux.at<float>(r,c);
        
        aux2[r] = aux1/depth_aux_width;
        
        //VARIANCE
        for(int c = 0; c < depth_aux_width; c++)
            aux3 += (depth_aux.at<float>(r,c)-aux2[r])*(depth_aux.at<float>(r,c)-aux2[r]);
        
        aux4[r] = aux3/depth_aux_width;
        
        aux1 = 0;
        aux3 = 0;

        if (depth_mean_txt.is_open () && depth_var_txt.is_open ()) {
            depth_mean_txt << aux2[r] << "\n";
            depth_var_txt << aux4[r] << "\n";
        }
        else 
            std::cout << "Unable to open file" << std::endl;
    }

    depth_mean_txt.close();
    depth_var_txt.close();

    memcpy(depth_mean.data, aux2.data(), aux2.size()*sizeof(float));  
    cv::imwrite("depth_mean_"+std::string(argv[1])+"_"+std::string(argv[2])+".png", depth_mean);

    memcpy(depth_var.data, aux4.data(), aux4.size()*sizeof(float));  
    cv::imwrite("depth_var_"+std::string(argv[1])+"_"+std::string(argv[2])+".png", depth_var);

    // int aux5 = 0;
    // for (int r = 0; r < image_size.height; r++)
    // {
    //     for (int c = 0; c < image_size.width; c++)
    //     {
    //         depth_mean.at<float>(r,c) = aux2[aux5 + c];
    //         depth_var.at<float>(r,c) = aux4[aux5 + c];
    //     }
    //     aux5+=image_size.width;
    // }

    cv::namedWindow("Depth_Mean", cv::WINDOW_NORMAL);
    cv::moveWindow("Depth_Mean", 0, 0);
    cv::resizeWindow("Depth_Mean", image_size.width, image_size.height);

    cv::namedWindow("Depth_var", cv::WINDOW_NORMAL);
    cv::moveWindow("Depth_var", 720, 0);
    cv::resizeWindow("Depth_var", image_size.width, image_size.height);

    char key = ' ';
    while (key != 'q') {
        cv::imshow("Depth_Mean", depth_mean);
        cv::imshow("Depth_var", depth_var);
        key = cv::waitKey(10);    
    }

    return 0;
}