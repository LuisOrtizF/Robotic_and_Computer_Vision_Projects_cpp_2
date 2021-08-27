#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <time.h>

using namespace cv;
using namespace std;

int main(){
    
    cout << "Opening Kinect_V1." << endl;

    VideoCapture sensor1;
    sensor1.open(CV_CAP_OPENNI);

    if( !sensor1.isOpened() ){
        cout << "Can Not Open Device." << endl;
        return -1;
    }

    sensor1.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ );   
    sensor1.set( CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION, 1 );
    
    cout << "fps: " << sensor1.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FPS) << endl;
   
    Mat depth1, rgb1;
    
    // Number of frames to capture
    int num_frames = 30;
    // Start and end times
    time_t start, end;
    // Start time
    time(&start);

    for(int i = 0; i < num_frames; i++){
        if( !sensor1.grab() ){
            cout << "Device can not grab images." << endl;
            return -1;
        }else{
            sensor1.retrieve( depth1, CV_CAP_OPENNI_DEPTH_MAP);
            imshow("depth", depth1);
            //imwrite("depth.png",depth1);
            sensor1.retrieve( rgb1, CV_CAP_OPENNI_BGR_IMAGE);
            imshow("rgb" ,rgb1);
        }
        if( waitKey(8) >= 0 )
                break;
   }
    // End Time
    time(&end);

    // Time elapsed
    double seconds = difftime (end, start);
    cout << "Time taken: " << seconds << " seconds" << endl;
     
    // Calculate frames per second
    double fps  = num_frames / seconds;
    cout << "Estimated fps: " << fps << endl;
     
    // Release video
    sensor1.release();
    return 0;
}