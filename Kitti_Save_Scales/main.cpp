
#include <opencv2/opencv.hpp>
#include "Eigen/Dense"

using namespace cv;
using namespace std;
using namespace Eigen;

void getPosesGT(string filename, vector<Isometry3d> & gtPoses)
{
  ifstream poses;
  string file;
  poses.open(filename);
  
  if (poses.is_open())
  {
    Vector3d t;
    Matrix3d R;
    Isometry3d T;
      
    while (getline(poses, file))
    {
      istringstream file_stream (file);
      double item_stream;
      vector<double> file_vector; 

      while(file_stream>>item_stream)
        file_vector.push_back(item_stream);
      
      t << file_vector[3], file_vector[7], file_vector[11];
      R(0,0) = file_vector[0];
      R(0,1) = file_vector[1];
      R(0,2) = file_vector[2];
      R(1,0) = file_vector[4];
      R(1,1) = file_vector[5];          
      R(1,2) = file_vector[6];
      R(2,0) = file_vector[8];
      R(2,1) = file_vector[9];          
      R(2,2) = file_vector[10]; 
      T.linear() = R;
      T.translation() = t;       
      gtPoses.push_back(T);
    }
    
    poses.close(); 
      
  }
  else
    cout << "Unable to open file 1!" << endl;
}

void getScalesGT(vector<Isometry3d> gtPoses, vector<double> & gtScales)
{
  double scales;

  for(int i = 0; i < gtPoses.size()-1; i++)
  {
    scales  = (gtPoses[i].inverse()*gtPoses[i+1]).translation().norm();
    gtScales.push_back(scales);
  }
}

void saveScalesGT (string file_name, vector<double> gtScales)
{
    fstream out_file;
    out_file.open(file_name, fstream::out);

    if (out_file.is_open())
    {
      for (int i = 0; i < gtScales.size(); i++)
        out_file << gtScales[i] << "\n"; 

      out_file.close();
    }
    else
      cout << "Unable to open file 2" << endl;
}

int main( int argc, char** argv )	
{  
  
  printf("\n%s\n\n", argv[0]);

  if (argc != 3)
  {
      printf("\nUsage: %s <kitti_poses_file (.txt)> <out_file_name (.txt)>\n\n", argv[0]);
      return 1;
  }

  vector<Isometry3d> gtPoses;
  getPosesGT(argv[1], gtPoses);
  vector<double> gtScales;
  getScalesGT(gtPoses, gtScales);
  saveScalesGT(argv[2], gtScales);

}