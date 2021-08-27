#include <sl/Camera.hpp>
#include <Save.hpp>

#include <boost/filesystem.hpp>

using namespace sl;

// Conversion function between sl::Mat and cv::Mat
cv::Mat slMat2cvMat(sl::Mat& input) {
    int cv_type = -1;
    switch (input.getDataType()) {
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
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM::CPU));
}

int main(int argc, char **argv) {

    std::cout << "\nZED_Save_Calib"<< std::endl;

    if(argc != 2)
    {
        printf("Syntax is: %s <VGA or HD720 or HD1080 or HD2K>\n\n", argv[0]);
        return 1;
    }


    // Create a ZED camera object
    Camera zed;

    // Set configuration parameters
    InitParameters init_params;
    if (std::string(argv[1]) == "VGA") 
        init_params.camera_resolution = sl::RESOLUTION::VGA;
    else if (std::string(argv[1]) == "HD720") 
        init_params.camera_resolution = sl::RESOLUTION::HD720;
    else if (std::string(argv[1]) == "HD1080") 
        init_params.camera_resolution = sl::RESOLUTION::HD1080;
    else if (std::string(argv[1]) == "HD2K") 
        init_params.camera_resolution = sl::RESOLUTION::HD2K;
    else 
        return 1;

    init_params.camera_fps = 60;

    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS){
        std::cout << "\n\n\tCamera Open " << err << " Exit program.\n" << std::endl;
        zed.close();
        return 1;
    }

    // Set runtime parameters after opening the camera
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE::STANDARD;

    // Prepare new image size to retrieve half-resolution images
    Resolution image_size = zed.getCameraInformation().camera_configuration.resolution;
    int new_width = image_size.width / 2;
    int new_height = image_size.height / 2;

    // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
    // Only the headers and pointer to the sl::Mat are copied, not the data itself
    sl::Mat image_left_zed(new_width, new_height, sl::MAT_TYPE::U8_C4);
    sl::Mat image_right_zed(new_width, new_height, sl::MAT_TYPE::U8_C4);
    sl::Mat image_left_zed_Und(new_width, new_height, sl::MAT_TYPE::U8_C4);
    sl::Mat image_right_zed_Und(new_width, new_height, sl::MAT_TYPE::U8_C4);

    cv::Mat image_left_ocv = slMat2cvMat(image_left_zed);
    cv::Mat image_right_ocv = slMat2cvMat(image_right_zed);
    cv::Mat image_left_ocv_Und = slMat2cvMat(image_left_zed_Und);
    cv::Mat image_right_ocv_Und = slMat2cvMat(image_right_zed_Und);
 
    cv::namedWindow("Image_Right", cv::WINDOW_NORMAL);
    cv::moveWindow("Image_Right", 0, 0);
    cv::resizeWindow("Image_Right", new_width, new_height);

    cv::namedWindow("Image_Left", cv::WINDOW_NORMAL);
    cv::moveWindow("Image_Left", new_width+65, 0 );
    cv::resizeWindow("Image_Left", new_width, new_height);

    cv::namedWindow("Image_Right_Und", cv::WINDOW_NORMAL);
    cv::moveWindow("Image_Right_Und", 2*new_width+65, 0 );
    cv::resizeWindow("Image_Right_Und", new_width, new_height); 
    
    cv::namedWindow("Image_Left_Und", cv::WINDOW_NORMAL);
    cv::moveWindow("Image_Left_Und", 3*new_width+65, 0 );
    cv::resizeWindow("Image_Left_Und", new_width, new_height); 

    // Loop until 'q' is pressed
    char key = ' ';
    while (key != 'q') {

        if (zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS) {
 
            // Retrieve the left image, depth image in half-resolution
            zed.retrieveImage(image_left_zed, VIEW::LEFT, MEM::CPU, image_size);
            zed.retrieveImage(image_right_zed, VIEW::RIGHT, MEM::CPU, image_size);

            zed.retrieveImage(image_left_zed_Und, VIEW::LEFT_UNRECTIFIED, MEM::CPU, image_size);
            zed.retrieveImage(image_right_zed_Und, VIEW::RIGHT_UNRECTIFIED, MEM::CPU, image_size);
            
            // Display image and depth using cv:Mat which share sl:Mat data
            cv::imshow("Image_Left", image_left_ocv);
            cv::imshow("Image_Right", image_right_ocv);
            cv::imshow("Image_Left_Und", image_left_ocv_Und);
            cv::imshow("Image_Right_Und", image_right_ocv_Und);

            // Handle key event
            key = cv::waitKey(10);
            run(zed);
        }
    }

    zed.close();

    return 0;
}
