
#include <iostream>

//Opencv includes
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
    VideoCapture cap(0);  // open the video camera no. 0

    if (!cap.isOpened())  // if not success, exit program
    {
        cout << "Cannot open the video cam" << endl;
        return -1;
    }

   cap.set(CV_CAP_PROP_FRAME_WIDTH,1280); //get the width of frames of the video
   cap.set(CV_CAP_PROP_FRAME_HEIGHT,720); //get the height of frames of the video
   cap.set(CV_CAP_PROP_FPS,30);

   namedWindow("MyVideo",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"

   while (1)
    {
        Mat frame;

        bool bSuccess = cap.read(frame); // read a new frame from video

         if (!bSuccess) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }    

        Size patternsize(3,3); //interior number of corners

        Mat gray;

        cvtColor(frame, gray, COLOR_BGR2GRAY);
        vector<Point2f> corners; //this will be filled by the detected corners

        //CALIB_CB_FAST_CHECK saves a lot of time on images
        //that do not contain any chessboard corners
        bool patternfound = findChessboardCorners(gray, patternsize, corners,
                CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
                + CALIB_CB_FAST_CHECK);

        if(patternfound)
          cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

        //cornerSubPix( viewGray, pointbuf, Size(11,11), Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));

        drawChessboardCorners(frame, patternsize, Mat(corners), patternfound);

        imshow("MyVideo", frame); //show the frame in "MyVideo" window


//        int key = 0xff & waitKey(cap.isOpened() ? 50 : 500);

//        if( (key & 255) == 27 )
//           break;

        if(waitKey(30) >= 0) break;

    }

   return 0;
}
