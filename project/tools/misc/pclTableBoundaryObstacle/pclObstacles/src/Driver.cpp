#include "Driver.h"

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>

#include <math.h>

//void Driver::startDriver() 
pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr Driver::startDriver(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{
    //std::cout << "starting driver" << std::endl;
    //TODO handle the case where there are multiple planes like table and ground
    //axis normal to which plane has to be found
    Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0);
    //fit plane
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (SACMODEL_PARALLEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    //TODO try out these parameters
    seg.setDistanceThreshold (0.02);//2
    seg.setEpsAngle( 30.0f * (M_PI/180.0f) );
    //TODO choose the right axis
    seg.setAxis(axis);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
      //TODO remove the cloud that does not fit
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl ;
    }
    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr returnCloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOnPlane (new pcl::PointCloud<pcl::PointXYZRGBA>);
    //get points on plane
    pcl::ExtractIndices<PointXYZRGBA> extract; 
    // Extract the inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    // Get the points associated with the planar surface
    extract.filter(*plane);
    // Create the filtering object
    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter (*cloudOnPlane);
    
    //remove nan
    std::vector<int> indices;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr noNANCloudOnPlane (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::removeNaNFromPointCloud(*cloudOnPlane, *noNANCloudOnPlane, indices);
    cloudOnPlane->clear(); //TODO uncomment this if this cloud is not used later in code (in bounding box)
    
    //TODO call this only for plane boundary not all the time
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud (plane);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    ec.setClusterTolerance(0.03); // 2cm
    ec.setMinClusterSize(100);
    //ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(plane);
    ec.extract(cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      {
        cloud_cluster->points.push_back(plane->points[*pit]); //*
      }
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
      //TODO use the largest plane for now, clean this
      *plane= *cloud_cluster;
      break;
    }
    
    //std::cout << "Plane size: " << plane->points.size ()  << std::endl;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scanCloud2(new pcl::PointCloud<pcl::PointXYZRGBA>);
    scanCloud2 = pointCloudTo2DScan2(noNANCloudOnPlane);
    //scanCloud2 = pointCloudTo2DScanPlane(plane);
    /*
    std::vector<float> scan = pointCloudTo2DScan(noNANCloudOnPlane);
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scanCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    int green = ((int)0) << 16 | ((int)255) << 8 | ((int)255);
    for(int i=0; i< scan.size();i++)
    //for(int i=90; i< 270;i++)
    {
      pcl::PointXYZRGBA p;
      p.rgb= green;
      p.x = (i + 180 -1)/100.0;//scan.at(i)*sin(i);// 
      p.y = 0.0f;
      p.z = scan.at(i);//*cos(i);//0.0f;
      scanCloud->push_back(p);
      if(i%10==0)
      std::cout << "scan:" << i << ": " << scan.at(i) << std::endl;
    } 
    */
    

    //*scanCloud = *scanCloud + *noNANCloudOnPlane;
    //returnCloud = noNANCloudOnPlane;
    //returnCloud = plane;
    //returnCloud = scanCloud;
    returnCloud = scanCloud2;
    return returnCloud;
}


pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Driver::pointCloudTo2DScan2(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOnPlane)
{
  std::vector<float> scan(360 ,1000.0f);//large initial value , field of view is 60 so taking 120 values
  std::vector<pcl::PointXYZRGBA > scanPoints(360);
  int green = ((int)0) << 16 | ((int)255) << 8 | ((int)255);
  for(pcl::PointCloud<pcl::PointXYZRGBA>::const_iterator it = cloudOnPlane->begin(); it!= cloudOnPlane->end(); it++)
  {
      float disFromOrig = sqrt( pow(((float) it->x),2.0) + pow(((float) it->y),2.0) + pow(((float) it->z),2.0) );
      //std::cout << "x" << it->x << "y" << it->y << "z" << it->z << std::endl ;
      //TODO find out the correct axis in forward direction
      //for now consider its y, so 
      //http://mathworld.wolfram.com/SphericalCoordinates.html theta = tan-1(y/x)

      float angleDegreesTheta  = atan2( ((float) it->y), ((float) it->x) ) * (180/M_PI);
      //float angleDegreesPhi =  acos( ( (float) it->z)/ disFromOrig ) * (180/M_PI); 
      //float angleDegreesTheta  = atan( ((float) it->y)/ ((float) it->x) ) * (180/M_PI);
      float angleDegreesPhi =  acos( ( (float) it->z)/ disFromOrig ) * (180/M_PI); 
      
      //if( ((int)angleDegreesTheta )%5 == 0)
      //std::cout <<  "angle: " << (int) angleDegreesTheta + 180 -1 << " dis: " << disFromOrig << std::endl ;
      //if( ((int) angleDegreesTheta + 180 -1) > 0 && ( (int) angleDegreesTheta + 180 -1)< scan.size() )
      //{
	if(scan.at((int) angleDegreesTheta + 180 -1) > disFromOrig)
	{
	  scan.at((int) angleDegreesTheta + 180 -1) = disFromOrig;
	  pcl::PointXYZRGBA p;
	  p.rgb= it->rgb;
	  p.x = it->x;
	  p.y = it->y;
	  p.z = it->z;
	  scanPoints.at((int) angleDegreesTheta + 180 -1) = p;
	}
      //}
  }
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scanCloud2(new pcl::PointCloud<pcl::PointXYZRGBA>);
  for(int i= 0; i<scanPoints.size() ;i++)
  {
    //std::cout <<  "push in to cloud: " << scanPoints.at(i) << std::endl ;
    scanPoints.at(i).rgb = green;
    scanCloud2->push_back(scanPoints.at(i));
  }
  *scanCloud2 = *cloudOnPlane + *scanCloud2;
  return scanCloud2;
}


std::vector<float> Driver::pointCloudTo2DScan(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOnPlane)
{
  std::vector<float> scan(360 ,1000.0f);//large initial value , field of view is 60 so taking 120 values
  for(pcl::PointCloud<pcl::PointXYZRGBA>::const_iterator it = cloudOnPlane->begin(); it!= cloudOnPlane->end(); it++)
  {
    
      float disFromOrig = sqrt( pow(((float) it->x),2.0) + pow(((float) it->y),2.0) + pow(((float) it->z),2.0) );
      //std::cout << "x" << it->x << "y" << it->y << "z" << it->z << std::endl ;
      //TODO find out the correct axis in forward direction
      //for now consider its y, so 
      //http://mathworld.wolfram.com/SphericalCoordinates.html theta = tan-1(y/x)

      float angleDegreesTheta  = atan2( ((float) it->y), ((float) it->x) ) * (180/M_PI);
      //float angleDegreesPhi =  acos( ( (float) it->z)/ disFromOrig ) * (180/M_PI); 
      //float angleDegreesTheta  = atan( ((float) it->y)/ ((float) it->x) ) * (180/M_PI);
      float angleDegreesPhi =  acos( ( (float) it->z)/ disFromOrig ) * (180/M_PI); 
      
      //if( ((int)angleDegreesTheta )%5 == 0)
      //std::cout <<  "angle: " << (int) angleDegreesTheta + 180 -1 << " dis: " << disFromOrig << std::endl ;
      //std::cout <<  "angle: " << (int) angleDegreesTheta + 180 -1 << " dis: " << disFromOrig << std::endl ;
      if( ((int) angleDegreesTheta + 180 -1) > 0 && ( (int) angleDegreesTheta + 180 -1)< scan.size() )
      {
	if(scan.at((int) angleDegreesTheta + 180 -1) > disFromOrig)
	{
	  scan.at((int) angleDegreesTheta + 180 -1) = disFromOrig;
	}
      }
  }
  return scan;
}