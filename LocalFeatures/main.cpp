#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/passthrough.h>

#include <fstream>
#include <sstream>


#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <iterator>
 #include <sys/types.h>
#include <dirent.h>
//#include <cmath.h>

bool next_iteration=false;

struct bird_object
{
    std::string name;
    std::string fname;
    int xmax;
    int ymax;
    int xmin;
    int ymin;
    int BOnum;
};
void read_directory(const std::string& name, std::vector<std::string>& v)

{
    DIR* dirp = opendir(name.c_str());
    struct dirent * dp;
    std::string bad1 = "..";
    std::string bad2 = ".";
    while ((dp = readdir(dirp)) != NULL) {
        if (bad1.compare(dp->d_name) != 0 && bad2.compare(dp->d_name) != 0)
        {
            v.push_back(dp->d_name);
        }

    }
    closedir(dirp);
}
bool fileExists(const char *fileName)
{
    ifstream infile(fileName);
    return infile.good();
}
void write_desc_filePFH(std::string filenameout,pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors)
{
    //Initialize ofstream
    std::ofstream descfile;
    descfile.open(filenameout.c_str(), std::ofstream::out | std::ofstream::app);

    //Write out descriptors to file
    for (size_t i = 0; i < (*descriptors).size(); ++i)
    {
        descfile<<(*descriptors)[i].histogram[0];
        for (size_t w = 1; w < 125; ++w)
        {
            descfile<<",";
            descfile<<(*descriptors)[i].histogram[w];
        }
        descfile<<endl;
    }


    //Close File
    descfile.close();
}
void write_desc_fileFPFH(std::string filenameout,pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors)
{
    //Initialize ofstream
    std::ofstream descfile;

    //Check for file existence
    if(fileExists(filenameout.c_str()))
    {
        descfile.open(filenameout.c_str(), std::ofstream::out | std::ofstream::app);
    }
    else
    {
        descfile.open(filenameout.c_str(), std::ofstream::out | std::ofstream::app);
    }
    //Write out descriptors to file
    for (size_t i = 0; i < (*descriptors).size(); ++i)
    {
        descfile<<(*descriptors)[i].histogram[0];
        for (size_t w = 1; w < 33; ++w)
        {
            descfile<<",";
            descfile<<(*descriptors)[i].histogram[w];
        }
        descfile<<endl;
    }

    std::cout << filenameout << std::endl;
    //Close File
    descfile.close();
}
boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2;
  //viewer2->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer2->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer2->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 1, 0.05, "normals");
  viewer2->addCoordinateSystem (1.0);
  viewer2->initCameraParameters ();
  return (viewer2);
}
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,void* nothing)
{
  if (event.getKeySym () == "space" && event.keyDown ())
    next_iteration = true;
}
void load_bird_objects(std::string path, std::vector<bird_object>* bird_objects, std::string filename)
{
    std::ifstream label_file;
    std::string compareBO;
    std::string currentBO;
    std::string output;
    int BOcount=0;
    label_file.open(path.c_str());
    if(label_file.is_open())
    {
        int i = 0;
        while(!label_file.eof() && i < 30)
        {
            if(!label_file.eof() && i < 30)
            {
                bird_object new_obj;
                label_file >> output;
                new_obj.name = filename+output;

                label_file >> output;
                new_obj.xmin = std::atoi(output.c_str());

                label_file >> output;
                new_obj.ymin = std::atoi(output.c_str());

                label_file >> output;
                new_obj.xmax = std::atoi(output.c_str());

                label_file >> output;
                new_obj.ymax = std::atoi(output.c_str());

                new_obj.fname = filename;

            //Check to see if the name of this object needs an iterator at the end:
                currentBO=new_obj.name;
                for(size_t birdObjectid = 0; birdObjectid < bird_objects->size(); ++birdObjectid)
                {
                    compareBO=(bird_objects->at(birdObjectid).name);
                    

                    if(currentBO.compare(compareBO)==0)
                    {


                        BOcount++;
                    }
                    else
                    {
                    //new_obj.BOnum=0;
                    }
                }
                new_obj.BOnum=BOcount;
                BOcount=0;
                bird_objects->push_back(new_obj);

                i++;
            }
        }
        bird_objects->pop_back();

        label_file.close();
    } else
    {
        std::cout << "didn't open" << std::endl;
    }

    std::cout.flush();
}

void calculate_PFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud , pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors,pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    std::cout << "Calculate PFH" << std::endl;
    //pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    // Estimate the normals.
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud);
    normalEstimation.setRadiusSearch(0.005);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    //normalEstimation.setSearchMethod(kdtree);
    //normalEstimation.compute(*normals);

    // PFH estimation object.
    pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud(cloud);
    pfh.setInputNormals(normals);
    pfh.setSearchMethod(kdtree);
    // Search radius, to look for neighbors. Note: the value given here has to be
    // larger than the radius used to estimate the normals.
    pfh.setRadiusSearch(0.005);

    pfh.compute(*descriptors);

}
void calculate_FPFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud , pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors,int normalRad,int searchRad)
{
  {
      std::cout << "Calculate PFH" << std::endl;
      pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

      // Estimate the normals.
      pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
      normalEstimation.setInputCloud(cloud);
      normalEstimation.setRadiusSearch(0.001);
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
      normalEstimation.setSearchMethod(kdtree);
      normalEstimation.compute(*normals);
      // Viewer
      //normalVis(cloud,normals);

      // PFH estimation object.
      pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
      fpfh.setInputCloud(cloud);
      fpfh.setInputNormals(normals);
      fpfh.setSearchMethod(kdtree);
      // Search radius, to look for neighbors. Note: the value given here has to be
      // larger than the radius used to estimate the normals.
      fpfh.setRadiusSearch(0.01);

      fpfh.compute(*descriptors);

  }
}


int main (int argc, char** argv)
{
    //std::string label_path = "/home/nathan/LocalFeatures/LocalFeatureData/rgb_label_txt";
    //std::string cloud_path = "/home/nathan/LocalFeatures/LocalFeatureData/SmallPointcloud/";
    //Get list of files
    std::vector<std::string> files;

    read_directory("/home/nathan/LocalFeatures/LocalFeatureData/rgb_label_txt/", files);
    //Open base file to compare to others

    //std::cout << "/home/nathan/LocalFeatures/LocalFeatureData/rgb_label_txt/" + files[0] << std::endl;
    std::cout <<"File read complete"<< std::endl;
    std::vector<bird_object>* bird_objects = new std::vector<bird_object>();
    std::string fullfilename="";
    std::string fnamecopy="";
    std::string currentBO="";
    std::string compareBO="";


    //Variable init
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    //std::string descType="FPFH";
    std::string descType="PFH";
    
    //Nearest NEighbor settings
    int K = 1000;

    //Descriptor Settings (meters)
    int NormR=0.001;
    int DescR=0.01;
    
    int xcenter=0;
    int ycenter=0;
    int bird_obj_dupe=0;
    size_t birdObjectid=0;//For tracking the current bird object out of the total list.
    int birdObjecttotal=0;//For tracking the total amt of bird objects
    size_t BOold=0;//Preloop object length
    size_t BOnew=0;//Postloop lengthstd
    size_t BOdiff=0;//BOnew-BOold
    std::stringstream ss;



    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtreeptr(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointXYZRGB searchPoint;
    //filter
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filteredNN (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors(new pcl::PointCloud<pcl::PFHSignature125>());
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fdescriptors(new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr fullnormals(new pcl::PointCloud<pcl::Normal>);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerVis(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewerVis->registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);
    //pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    std::string holder;

    //pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    //Add in for loop for processing the pointclouds here.
    for(size_t fileidx = 0; fileidx < files.size (); ++fileidx)
    {
        //Making copy of filename
        fnamecopy=files[fileidx];
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/nathan/LocalFeatures/LocalFeatureData/pointcloudascii/" + files[fileidx].replace(files[fileidx].begin()+3, files[fileidx].end(), ".pcd"), *cloud) == -1) //* load the file
        {
          PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
          return (-1);
      }
        //viewer.showCloud (cloud);

      
        //pass.setInputCloud (cloud);
        //pass.setFilterFieldName ("x");
        //pass.setFilterLimits (-0.2, 0.2);
        //pass.setFilterLimitsNegative (true);
        //pass.filter (*cloud_filtered);
        //cloud = cloud_filtered;
        //viewer.showCloud (cloud);

        //Find current B.O Lenght
      BOold=bird_objects->size();

        //Load all of the bird objects (length=#files*#Labels/file)
      load_bird_objects("/home/nathan/LocalFeatures/LocalFeatureData/rgb_label_txt/" + fnamecopy, bird_objects,fnamecopy);

        //New B.O length
      BOnew=bird_objects->size();

        //Difference

      BOdiff=BOnew-BOold;


      std::cout << cloud->size() << std::endl;
      for(size_t birdObjectid = 0; birdObjectid < BOdiff; ++birdObjectid)
      {
        	//Center point of label window for each bird object.
       xcenter = bird_objects->at(BOold+birdObjectid).xmin + (bird_objects->at(BOold+birdObjectid).xmax - bird_objects->at(BOold+birdObjectid).xmin)/2;
       ycenter = bird_objects->at(BOold+birdObjectid).ymin + (bird_objects->at(BOold+birdObjectid).ymax - bird_objects->at(BOold+birdObjectid).ymin)/2;
       searchPoint = cloud->at(xcenter, ycenter);
            //Check for nan on any search point coords
       if(!std::isnan(searchPoint.x)|| !std::isnan(searchPoint.x) ||!std::isnan(searchPoint.x))
       {
            //viewer.showCloud (cloud);
        kdtree.setInputCloud (cloud);

        std::cout << "K nearest neighbor search at (" << searchPoint.x
        << " " << searchPoint.y
        << " " << searchPoint.z
        << ") with K=" << K << std::endl;

        std::cout<<"Loading file: "<<"/home/nathan/LocalFeatures/LocalFeatureData/rgb_label_txt/" + fnamecopy<<endl;

            // K nearest neighbor search
        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
            {
//                    std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x<< " "
//                    << cloud->points[ pointIdxNKNSearch[i] ].y
//                    << " " << cloud->points[ pointIdxNKNSearch[i] ].z
//                    << " (distance: " << sqrt(pointNKNSquaredDistance[i]) << ")"
//                    << std::endl;

                cloud_filteredNN->push_back(cloud->points[ pointIdxNKNSearch[i] ]);
            }
        }
            //viewer.showCloud (cloud_filteredNN);
            // Estimate the normals.

        normalEstimation.setInputCloud(cloud_filteredNN);
        normalEstimation.setRadiusSearch(0.003);

        normalEstimation.setSearchMethod(kdtreeptr);
        normalEstimation.compute(*normals);
        //Calulate full normals
        normalEstimation.setInputCloud(cloud);
        normalEstimation.setRadiusSearch(0.003);

        normalEstimation.setSearchMethod(kdtreeptr);
        normalEstimation.compute(*fullnormals);
        //PCL Visualizer Commands

        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_filteredNN);
        viewerVis->addPointCloud<pcl::PointXYZRGB> (cloud_filteredNN, rgb, "sample cloud");
        viewerVis->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "full cloud");
        viewerVis->resetCameraViewpoint("sample cloud");
        //viewerVis->resetCamera ();
        viewerVis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud");
        viewerVis->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud_filteredNN, normals, 1, 0.001, "normals");
        //viewerVis->addSphere(searchPoint,0.005,"sphere1");
        //viewerVis->addCoordinateSystem (1.0);
        viewerVis->initCameraParameters();
        viewerVis->spinOnce(2000);
        if(next_iteration)
          {
            next_iteration=false;
            while (!viewerVis->wasStopped ())
             {
                viewerVis->spinOnce();
                if(next_iteration)
                  {
                    break;
                  }

             }
            next_iteration=false;
          }


        //viewerVis->spinOnce(10000);

        // Clear the view
        viewerVis->removeAllShapes();
        viewerVis->removeAllPointClouds();


        //pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors(new pcl::PointCloud<pcl::PFHSignature125>());
        //calculate_FPFH(cloud_filteredNN, fdescriptors,NormR,DescR);
        calculate_PFH(cloud_filteredNN, descriptors,normals);
            //Generate descriptor Filename


        ss<<bird_objects->at(BOold+birdObjectid).BOnum;
        fullfilename=(bird_objects->at(BOold+birdObjectid).name);
        fullfilename=fullfilename+fullfilename.substr(0,3)+descType+ss.str()+".csv";
        fullfilename.replace(fullfilename.begin(),fullfilename.begin()+7,"");
        //write_desc_fileFPFH(fullfilename,fdescriptors);
        write_desc_filePFH(fullfilename,descriptors);
    }
    //cin>>holder;
            //Clear vectors and points
    cloud_filteredNN->clear();
    pointIdxNKNSearch.clear();
    pointNKNSquaredDistance.clear();
    fdescriptors->clear();
    bird_obj_dupe=0;
    ss.str(std::string());
            //cin>>holder;

            //birdObjectid++;
}
        //fileidx++;

        //cin>>holder;
}
//cin>>hold\er;






    //base_descriptors(new pcl::PointCloud<pcl::PFHSignature125>());
std::cout << "PFH descriptor calc done" << std::endl;
//    for (int i = 0; i < pointIdxNKNSearch.size (); ++i)
//    {
//        base_descriptors->push_back(descriptors->points[ pointIdxNKNSearch[i] ]);
//    }
//    std::cout << base_descriptors->size() << std::endl;


std::cout.flush();
while (!viewerVis->wasStopped ())
{
        viewerVis->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
}
   // viewer.showCloud (cloud_filteredNN);
//    while (!viewer.wasStopped ())
//    {
//    }
//    return 0;


}
