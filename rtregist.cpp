// Original code by Geoffrey Biggs, taken from the PCL tutorial in
// http://pointclouds.org/documentation/tutorials/pcl_visualizer.php
// Simple Kinect viewer that also allows to write the current scene to a .pcd
// when pressing SPACE.
#include <iostream>
#include <string>
#include <fstream>
#include <vector>

#include <pcl/registration/icp_nl.h>
 
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
// Generic includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
// Visualizer includes
#include <pcl/common/time.h>
// OpenNI includes
#include <pcl/io/openni_camera/openni_driver.h>
//ICP includes
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>

using namespace std;
using namespace pcl;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::ConstPtr CloudConstPtr;
typedef Cloud::Ptr CloudPtr;

//PointCloud<PointXYZRGB>::Ptr cloudptr(new PointCloud<PointXYZRGB>);   // A cloud that will store colour info.
//PointCloud<PointXYZ>::Ptr fallbackCloud(new PointCloud<PointXYZ>);    // A fallback cloud with just depth data.
boost::shared_ptr<visualization::CloudViewer> viewer;                 // Point cloud viewer object.
Grabber* kinectGrabber;                                               // OpenNI grabber that takes data from Kinect.
unsigned int filesSaved = 0;                                          // For the numbering of the clouds saved to disk.
bool saveCloud(false), quitProg(false);                               // Program control.

//added by XD
std::vector<string> savedFiles;

//noColour(false)
// This function is called every time the Kinect has new data.
void grabberCallback(const PointCloud<PointXYZ>::ConstPtr& cloud)
{
   // static size_t i=0;
    if (! viewer->wasStopped())
        viewer->showCloud(cloud);
 
   //save the current frame into a VFH pcd file
    if (saveCloud)
    {      
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
          pcl::PassThrough<pcl::PointXYZ> pass;
          pass.setInputCloud(cloud);
          pass.setFilterFieldName("x");
          pass.setFilterLimits(0, 1000.0);
          pass.setFilterFieldName("y");
          pass.setFilterLimits(0, 1000.0);
          pass.setFilterFieldName("z");
          pass.setFilterLimits(0, 1000.0);
          pass.filter(*cloud_filtered);
      //pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
      //convert normals keypoints into VFH features.
      //convtVFH(cloud, vfhs);
        stringstream stream;
        stream << "inputCloud" << filesSaved << ".pcd";
        string filename = stream.str();
        //std::vector<string> name_indices;
	
	//changed by XD
        savedFiles.push_back( filename );
        
	//string filename = stream.str();
        if (io::savePCDFile(filename, *cloud_filtered, true) == 0)
        {
            filesSaved++;
            //i++;
            cout << "Saved " << filename << "." << endl;

        }
        else PCL_ERROR("Saving Problem%s.\n", filename.c_str());
        saveCloud = false;
        
    }
}
// For detecting when SPACE is pressed.
void keyboardEventOccurred(const visualization::KeyboardEvent& event,
    void* nothing)
{
    if (event.getKeySym() == "space" && event.keyDown())
        saveCloud = true;
    if (event.getKeySym() == "Escape" && event.keyDown())
    {
        quitProg = true;        
    }    
    else cout<< event.getKeySym()<<endl;
}
 
// Creates, initializes and returns a new viewer.
boost::shared_ptr<visualization::CloudViewer> createViewer()
{
    boost::shared_ptr<visualization::CloudViewer> v
        (new visualization::CloudViewer("3D Viewer"));
    v->registerKeyboardCallback(keyboardEventOccurred);
    return(v);
}
 
int main(int argc, char** argv)
{
 double dist = 0.05;
 double rans = 0.05;
 int iter = 50;
 bool nonLinear = false;
    //start fetching and displaying frames from Kinect.   
    kinectGrabber = new OpenNIGrabber();
    if (kinectGrabber == 0)
	return false;
    boost::function<void (const PointCloud<PointXYZ>::ConstPtr&)> f =
	boost::bind(&grabberCallback, _1);
    kinectGrabber->registerCallback(f);

    viewer = createViewer();
 
    kinectGrabber->start();
 
    // Main loop.
    while (! viewer->wasStopped())
    {
        boost::this_thread::sleep(boost::posix_time::seconds(1));
        if (quitProg)
        {
            cout << "esc..."<<endl;
//changed by XD
//test the file vector
/*cout << endl << "In the savedFiles vector:";
	for(vector<string>::iterator it = savedFiles.begin(); it != savedFiles.end(); it++)
	{
		cout << ' ' << *it;
	} 
	cout << endl;
*/
            kinectGrabber->stop();
            free(kinectGrabber);
        pcl::PointCloud<pcl::PointXYZ>::Ptr model (new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile (savedFiles[0], *model) == -1)
        {
         std::cout << "Could not read file" << std::endl;
          return -1;
         }

 Eigen::Matrix4f t (Eigen::Matrix4f::Identity ());
for (size_t i = 1; i < savedFiles.size (); i++)
  {
    CloudPtr data (new Cloud);
    if (pcl::io::loadPCDFile (savedFiles[i], *data) == -1)
    {
      std::cout << "Could not read file" << std::endl;
      return -1;
    }
   // std::cout << savedFiles[i] << " width: " << data->width << " height: " << data->height << std::endl;

    pcl::IterativeClosestPoint<PointType, PointType> *icp;

    if (nonLinear)
    {
      std::cout << "Using IterativeClosestPointNonLinear" << std::endl;
      icp = new pcl::IterativeClosestPointNonLinear<PointType, PointType>();
    }
    else
    {
      std::cout << "Using IterativeClosestPoint" << std::endl;
      icp = new pcl::IterativeClosestPoint<PointType, PointType>();
    }

    icp->setMaximumIterations (iter);
    icp->setMaxCorrespondenceDistance (dist);
    icp->setRANSACOutlierRejectionThreshold (rans);

    icp->setInputTarget (model);

    //icp->setInputCloud (data);
       icp->setInputSource (data);
    CloudPtr tmp (new Cloud);
    icp->align (*tmp);

    t = icp->getFinalTransformation () * t;

    pcl::transformPointCloud (*data, *tmp, t);

    std::cout << icp->getFinalTransformation () << std::endl;

    *model = *data;

   // std::string result_filename (argv[pcd_indices[i]]);
  //  result_filename = result_filename.substr (result_filename.rfind ("/") + 1);
    pcl::io::savePCDFileBinary (savedFiles[i], *tmp);
    std::cout << "saving result to the last figure " << std::endl;
  }
            break;
                
        }
    }
 
//     if (! justVisualize)
//         kinectGrabber->stop();

}
