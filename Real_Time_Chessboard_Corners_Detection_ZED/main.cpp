//ZED include
#include <zed/Mat.hpp>
#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>

//Opencv includes
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <iostream>

using namespace cv;
using namespace std;
using namespace sl::zed;

int main( )
{

    // Initialize ZED in HD and depth in quality mode
    Camera* camara_zed = new Camera(HD720,60); //Camera constructor: ZED resolution, fps for this resolution

    // Parameters for ZED initialization.
    InitParams parameters;
    parameters.mode = MODE::QUALITY;
    parameters.unit = UNIT::MILLIMETER;

    ERRCODE error = camara_zed->init(parameters);

    // Quit if an error occurred
    if (error != SUCCESS)
    {
          cout << "Unable to init the ZED: " << errcode2str(error) << endl;
          delete camara_zed;
          return 1;
    }

    // Create OpenCV windows
    namedWindow("Corners", WINDOW_AUTOSIZE);

    while (1)
    {
       // Grab frame in STANDARD (Structure conservative, no occlusion filling) sensing mode
       if (!camara_zed->grab(SENSING_MODE::STANDARD))
       {

           // Retrieve left color image
           cv::Mat src = slMat2cvMat(camara_zed->retrieveImage(LEFT));

           cv::Mat src_gray;
           cvtColor(src, src_gray, CV_BGR2GRAY);

           Size xy_chess_size(4,3);//number of internal corners of the plane (x-y) 3D chessboard  (6 en x(cols-width),4 en y(rows-height))
           //Size yz_chess_size(7,6);//number of internal corners of the plane (y-z) 3D chessboard
           //Size xz_chess_size(7,5);//number of internal corners of the plane (x-z) 3D chessboard

           vector<Point2f> corners1;
           //vector<Point2f> corners2;
           //vector<Point2f> corners3;
           //vector<vector<Point2f> > corners;

           //CALIB_CB_FAST_CHECK saves a lot of time on images
           //that do not contain any chessboard corners
           bool patternfound1 = findChessboardCorners(src_gray, xy_chess_size, corners1,
                                                      CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
           //bool patternfound2 = findChessboardCorners(src_gray, yz_chess_size, corners2,
           //                                           CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
           //bool patternfound3 = findChessboardCorners(src_gray, xz_chess_size, corners3,
           //                                           CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);


           //if(patternfound1 && patternfound2 && patternfound3 && corners1.size()==24 && corners2.size()==42 && corners3.size()==35)
           if(patternfound1)
           {

               cornerSubPix(src_gray, corners1, Size(3,3), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 0.01));
               //cornerSubPix(src_gray, corners2, Size(3,3), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 0.01));
               //cornerSubPix(src_gray, corners3, Size(3,3), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 0.01));

               drawChessboardCorners(src, xy_chess_size, cv::Mat(corners1), patternfound1);
               //drawChessboardCorners(src, yz_chess_size, cv::Mat(corners2), patternfound2);
               //drawChessboardCorners(src, xz_chess_size, cv::Mat(corners3), patternfound3);

           }

           imshow("Corners", src); //show the frame in window

       }

       if(waitKey(30) >= 0) break;

    }

   delete camara_zed;
}
