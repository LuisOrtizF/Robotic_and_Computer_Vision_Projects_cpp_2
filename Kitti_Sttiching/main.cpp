#include <opencv2/opencv.hpp>
#include "opencv2/stitching.hpp"
#include <dirent.h>

using namespace std;
using namespace cv;

int main(int argc, char* argv[])
{
    printf("\n%s\n\n", argv[0]);

	if (argc != 2)
	{
	    printf("\nUsage: %s <sequence_path>\n\n", argv[0]);
	    return 1;
	}

    namedWindow("Stitch", CV_WINDOW_NORMAL);
	moveWindow("Stitch", 0, 0);
	resizeWindow("Stitch", 800, 600);

    // File count
    int file_count = 0;
    string dirAll = string(argv[1])+"image_0";

    DIR * dirp;
    struct dirent * entry;

    dirp = opendir(dirAll.c_str()); 
    while ((entry = readdir(dirp)) != NULL)
        if (entry->d_type == DT_REG) 
            file_count++;
    closedir(dirp);

    bool try_use_gpu = false;
    Stitcher::Mode mode = Stitcher::PANORAMA;

    Ptr<Stitcher> stitcher = Stitcher::create(mode, try_use_gpu);
    Ptr<WarperCreator> warper_creator;
    warper_creator = makePtr<cv::PlaneWarper>();
    stitcher->setWarper(warper_creator);
    stitcher->setPanoConfidenceThresh(0.5);

    char base_name[256]; 
    string file_name;
    Mat image;
    vector<Mat> imgs;

    for (int i = 0; i < 1; i++) 
    {
        for (int j = 0; j < 2; j++)
        {
            sprintf(base_name, "image_%d/%06d.png", j, i);
            file_name  =  string(argv[1]) + base_name;
            image = imread( file_name );
            
            if (image.empty())
            {
                cout << "Can't read image '" << file_name << "'\n";
                return -1;
            }

            imgs.push_back(image);
            memset(base_name, 0, sizeof base_name);
            file_name.clear();
            image.release();
        }

        Mat pano;
        Stitcher::Status status = stitcher->stitch(imgs, pano);

        imgs.clear();

        if (status != Stitcher::OK)
        {
            cout << "Can't stitch images, error code = " << int(status) << endl;
            // return -1;
        }
        else{
            imshow("Stitch", pano);
            waitKey(1);
            cout << "stitching completed successfully" << endl;
        }
        
        imwrite("panorama.png", pano);
    }

    destroyAllWindows();
    return 0;
}