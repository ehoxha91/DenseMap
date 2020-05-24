
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <ostream>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>

//Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/core/eigen.hpp>
#include <opencv4/opencv2/xfeatures2d/nonfree.hpp>

// Point Cloud Library
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/filters/passthrough.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/common/transforms.h>
#include <pcl-1.8/pcl/visualization/cloud_viewer.h>
#include <pcl-1.8/pcl/filters/voxel_grid.h>
#include <pcl-1.8/pcl/filters/passthrough.h>


using namespace std;
using namespace Eigen;
using namespace cv;

#define RESET "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */
#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */
#define BOLDRED "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */

// camera internal reference
const double camera_factor = 1000; //1000 - represent 1m
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;

//Definition of PointCloud and PointT types
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef vector<Isometry3d, aligned_allocator<Isometry3d>> poseType;

typedef struct CAMERA_INTRINSIC_PARAMETERS
{
	double cx, cy, fx, fy, scale;
}Camera;

typedef struct FRAME
{
	Mat rgb;
	Mat depth;
	int id;
}Frame;

Frame readFrame(int, string);
PointCloud::Ptr image2PointCloud(cv::Mat& rgb, cv::Mat& depth, Camera& Camera);
void readFile(string filename, poseType &poses ,vector<int> &_frameId);

int main( int argc, char** argv )
{
	if(argc != 5)
	{
		cerr << RED"Command should have this form: ./denseMapping dataset_folder trajectory_file max_frames denseMapp.pcd" << endl;
		return -1;
	}

	Camera cam;
	// cam.cx = 294.827;
	// cam.cy = 241.541;
	// cam.fx = 638.276;
	// cam.fy = 637.043;
	// cam.scale  = 1000.0;
	cam.cx=318.6;
    cam.cy=255.3;
    cam.fx=517.3;
    cam.fy=516.5;
    cam.scale=5000.0;

	string datasetPath = string(argv[1]);		// Folder for rgb and depth images.
	string _maxframes = string(argv[3]);
	int maxframes;
	istringstream(_maxframes) >> maxframes;
	string outputFilenName = string(argv[4]);
	
	vector<int> frameIDs;
	poseType poses;
	cout <<"Path: " << datasetPath<<endl;
	cout <<"Trajectory: " << string(argv[2])<<endl;
	readFile(string(argv[2]), poses, frameIDs);	// Get all the poses
	
    PointCloud::Ptr pcloud (new PointCloud());
    PointCloud::Ptr tmp (new PointCloud());

	for (int i = 0; (i < frameIDs.size()) && (i<maxframes); i++)
	{
		
		Frame currFrame = readFrame(frameIDs[i], datasetPath);
		PointCloud::Ptr newCloud = image2PointCloud( currFrame.rgb, currFrame.depth, cam ); //Generate point cloud
        pcl::transformPointCloud(*newCloud, *tmp, poses[i].matrix());
		cout <<"T = \n"<<poses[i].matrix() << endl;
        *pcloud += *tmp;
        tmp->clear();
		cout <<YELLOW"Current frame: " << frameIDs[i]<<endl;
	}
	pcl::visualization::CloudViewer viewer( "viewer" );
    viewer.showCloud( pcloud );
    while( !viewer.wasStopped() )
    {

    }
	pcl::io::savePCDFile(outputFilenName, *pcloud);
	cout << GREEN"Point Cloud Saved as: " << outputFilenName<<endl;
	return 0;
}

// Read frame
Frame readFrame(int index, string datasetFolder)
{
    Frame frame;
    string rgbd_filename =  datasetFolder+ "/rgb/r" + to_string(index)+".png";
    string depth_filename = datasetFolder+ "/depth/d" + to_string(index)+".png";

    frame.rgb = imread(rgbd_filename);
    frame.depth = imread(depth_filename,-1);
    frame.id = index;
    return frame;
}

//Convert RGB image/map to point cloud
PointCloud::Ptr image2PointCloud( Mat& rgb, Mat& depth, Camera& camera )
{
	PointCloud::Ptr cloud (new PointCloud);
	for (int m = 0; m < depth.rows; m++)
	{
		for (int n = 0; n < depth.cols; n++)
		{
			//get the value of depth from depth image:
			ushort d = depth.ptr<ushort> (m)[n];
			//IF d == 0, we don't have enough info for point cloud, skip it.
			if(d == 0) continue;

			//ELSE:
			PointT p;

			//Use depth to generate 3D coordinates
			p.z = double (d)/camera.scale;
			p.x = (n - camera.cx) * p.z / camera.fx;
			p.y = (m - camera.cy) * p.z / camera.fy;
			
			//Read the colors:
			p.b = rgb.ptr<uchar>(m)[n*3];
			p.g = rgb.ptr<uchar>(m)[n*3+1];
			p.r = rgb.ptr<uchar>(m)[n*3+2];

			//push the point to the list:
			cloud->points.push_back(p);
		}
	}
	
	cloud->height = 1;
	cloud->width = cloud->points.size();
	cloud->is_dense = false;

	return cloud;
}

void readFile(string filename, poseType &poses ,vector<int> &_frameIds)
{
    ifstream fin(filename);

    if(!fin)
    {
        cout << "Can't load trajectory file\n";
        exit(-1);
    }

    while (!fin.eof()) 
    {
        double frameId, tx, ty, tz, qx, qy, qz, qw;
        fin >> frameId >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Isometry3d Twr(Quaterniond(qw, qx, qy, qz));
        Twr.pretranslate(Vector3d(tx, ty, tz));
        poses.push_back(Twr);
		_frameIds.push_back(frameId);

    }
    cout << "Read total " << _frameIds.size() << " pose entries." << endl;
}