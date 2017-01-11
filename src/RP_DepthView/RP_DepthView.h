#ifndef RP_DEPTHVIEW_H
#define RP_DEPTHVIEW_H

#include <iostream>
#include <string>
#include <thread>
#include <mutex>

using namespace std;

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
//#include "opencv2/nonfree/features2d.hpp"
//#include "opencv2/nonfree/nonfree.hpp"
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


namespace enc = sensor_msgs::image_encodings;




class RP_DepthView
{
	protected :
	
	std::mutex mutexRES;
	bool continuer;
	
	//std::vector<cv::Mat> frameProcessed;
	std::vector<std::vector<cv::Mat> > frames;
	std::vector<sensor_msgs::PointCloud2> clouds;
	
	std::thread* t;
	
	public :
	
	ros::NodeHandle nh;
	
	
	image_transport::ImageTransport* it;
	//image_transport::Publisher img_pub;
	image_transport::Publisher obs_pub;
	ros::Publisher cloud_pub;
	ros::Publisher cloudFiltered_pub;
	
	ros::Subscriber cloud_sub;
	/*image_transport::Subscriber img_sub1;
	image_transport::Subscriber img_sub2;
	image_transport::Subscriber img_sub3;
	image_transport::Subscriber img_sub4;*/
	
	int robot_number;
	int method;
	int scaler;
	float voxelGridScaler;
	float tresholdFilter;
	
	RP_DepthView(const int& robot_number_ = 0, const float& voxelGridScaler_ = 0.25f, const float& tresholdFilter_ = 0.3f, const int& method_ = 0 ) : continuer(true), robot_number(robot_number_), method(method_), scaler(1), voxelGridScaler(voxelGridScaler_), tresholdFilter( tresholdFilter_)
	{			
		for(int i=4;i--;)	
		{
			frames.push_back( std::vector<cv::Mat>() );
			//frameProcessed.push_back( cv::Mat() );
		}
		
		
		it = new image_transport::ImageTransport(nh);
		
		//std::string path( "/RP_robot_model_"+std::to_string(robot_number)+"/");
		std::string path( "/camera/");
		std::string pathCloud( path+"depth/points");
		/*
		std::string path1( path+"depth/image_raw");		
		std::string path2( path+"camera2/image_raw");
		std::string path3( path+"camera3/image_raw");
		std::string path4( path+"camera4/image_raw");
		*/
		
		cloud_sub = nh.subscribe( pathCloud.c_str(), 1, &RP_DepthView::callbackCloud,this);
		/*
		img_sub1 = it->subscribe( path1.c_str(), 1, &RP_DepthView::callback1,this);
		img_sub2 = it->subscribe( path2.c_str(), 1, &RP_DepthView::callback2,this);
		img_sub3 = it->subscribe( path3.c_str(), 1, &RP_DepthView::callback3,this);
		img_sub4 = it->subscribe( path4.c_str(), 1, &RP_DepthView::callback4,this);
		*/
		
		/*
		img_pub = it->advertise( std::string(path+"/DEPTHVIEW").c_str(), 1);
		*/
		obs_pub = it->advertise( std::string(path+"/DEPTHOBSTACLES").c_str(), 1);
		
		cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(std::string(path+"/POINTCLOUD").c_str(), 1);
		cloudFiltered_pub = nh.advertise<sensor_msgs::PointCloud2>(std::string(path+"/POINTCLOUD_FILTERED").c_str(), 1);
		
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		
		
		t = new std::thread(&RP_DepthView::loop, this);
		
		ROS_INFO( std::string("RP_DepthView::"+std::to_string(robot_number)+"::Initialization : OK.").c_str() );
	}
	
	~RP_DepthView()
	{
		this->setContinuer(false);
		
		if(t->joinable())
		{
			t->join();
		}
		
		delete t;
		delete it;
		
		ROS_INFO("RP_DepthView::Exiting.");
	}
	
	void callbackCloud(const sensor_msgs::PointCloud2Ptr& original_cloud)
	{
		sensor_msgs::PointCloud2 cloud_in;
		
		//------------------------------
		//------------------------------
		
		// Containers :
		pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
		pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
		pcl::PCLPointCloud2 cloud_filtered;

		// Convert to PCL data type
		pcl_conversions::toPCL(*original_cloud, *cloud);

		// Perform the actual filtering
		pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
		sor.setInputCloud(cloudPtr);
		sor.setLeafSize(this->voxelGridScaler, this->voxelGridScaler, this->voxelGridScaler);
		sor.filter(cloud_filtered);

		// Convert to ROS data type
		pcl_conversions::fromPCL(cloud_filtered, cloud_in);
		
		mutexRES.lock();
		clouds.insert(clouds.end(), cloud_in );	
		mutexRES.unlock();
		
	}
	
	/*
	void callback1(const sensor_msgs::ImageConstPtr& original_image)
	{
		cv_bridge::CvImagePtr cv_ptr;
	
		try
		{
			//cv_ptr = cv_bridge::toCvCopy(original_image,enc::32FC1);
			if(enc::isColor(original_image->encoding))
				cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
			else
				cv_ptr = cv_bridge::toCvCopy(original_image, enc::MONO16);
				//cv_ptr = cv_bridge::toCvCopy(original_image, enc::MONO8);
				//cv_ptr = cv_bridge::toCvShare(original_image, enc::MONO8);
		}
		catch( cv_bridge::Exception e)
		{
			ROS_ERROR("RP_DepthView::::cv_bridge exception : %s", e.what());
			return;
		}
		//------------------------------------------------
		//		Stack of Frames
		//-----------------------------------------------
		cv::Mat frameDownSample;
		int h = cv_ptr->image.rows/scaler;
		//h*=2;
		//int h = 480;
		int w = cv_ptr->image.cols/scaler;
		//w*=2;
		//int w = 640;
		cv::resize(cv_ptr->image,frameDownSample,cv::Size(w,h));
		
		//------------------------------
		//------------------------------
		//	enhancement :
		//------------------------------
		//------------------------------
		//float sigma_s = 10.àf;
		//float sigma_r = 0.15f;
		//cv::detailEnhance(frameDownSample, frameDownSample, sigma_s, sigma_r);
		
		/// Do the operation new_image(i,j) = alpha*image(i,j) + beta
		float alpha = 1.0f;
		float beta = 0.0f;
		frameDownSample.convertTo( frameDownSample, -1, alpha, beta);
    
		//------------------------------
		//------------------------------
		
		
		mutexRES.lock();
		frames[0].insert(frames[0].begin(), frameDownSample);	
		mutexRES.unlock();
		
	}
	
	void callback2(const sensor_msgs::ImageConstPtr& original_image)
	{
		cv_bridge::CvImagePtr cv_ptr;
	
		try
		{
			cv_ptr = cv_bridge::toCvCopy(original_image,enc::BGR8);
		}
		catch( cv_bridge::Exception e)
		{
			ROS_ERROR("RP_DepthView::::cv_bridge exception : %s", e.what());
			return ;
		}
		//------------------------------------------------
		//		Stack of Frames
		//-----------------------------------------------
		cv::Mat frameDownSample;
		int h = cv_ptr->image.rows/scaler;
		//h*=2;
		//int h = 480;
		int w = cv_ptr->image.cols/scaler;
		//w*=2;
		//int w = 640;
		cv::resize(cv_ptr->image,frameDownSample,cv::Size(w,h));
		
		//------------------------------
		//------------------------------
		//	enhancement :
		//------------------------------
		//------------------------------
		//float sigma_s = 10.àf;
		//float sigma_r = 0.15f;
		//cv::detailEnhance(frameDownSample, frameDownSample, sigma_s, sigma_r);
		
		/// Do the operation new_image(i,j) = alpha*image(i,j) + beta
		float alpha = 1.0f;
		float beta = 0.0f;
		frameDownSample.convertTo( frameDownSample, -1, alpha, beta);
    
		//------------------------------
		//------------------------------
		
		
		mutexRES.lock();
		frames[1].insert(frames[1].begin(), frameDownSample);	
		mutexRES.unlock();
		
	}
	
	void callback3(const sensor_msgs::ImageConstPtr& original_image)
	{
		cv_bridge::CvImagePtr cv_ptr;
	
		try
		{
			cv_ptr = cv_bridge::toCvCopy(original_image,enc::BGR8);
		}
		catch( cv_bridge::Exception e)
		{
			ROS_ERROR("RP_DepthView::::cv_bridge exception : %s", e.what());
			return ;
		}
		//------------------------------------------------
		//		Stack of Frames
		//-----------------------------------------------
		cv::Mat frameDownSample;
		int h = cv_ptr->image.rows/scaler;
		//h*=2;
		//int h = 480;
		int w = cv_ptr->image.cols/scaler;
		//w*=2;
		//int w = 640;
		cv::resize(cv_ptr->image,frameDownSample,cv::Size(w,h));
		
		//------------------------------
		//------------------------------
		//	enhancement :
		//------------------------------
		//------------------------------
		//float sigma_s = 10.àf;
		//float sigma_r = 0.15f;
		//cv::detailEnhance(frameDownSample, frameDownSample, sigma_s, sigma_r);
		
		/// Do the operation new_image(i,j) = alpha*image(i,j) + beta
		float alpha = 1.0f;
		float beta = 0.0f;
		frameDownSample.convertTo( frameDownSample, -1, alpha, beta);
    
		//------------------------------
		//------------------------------
		
		
		mutexRES.lock();
		frames[2].insert(frames[2].begin(), frameDownSample);	
		mutexRES.unlock();
		
	}
		
		
		
	void callback4(const sensor_msgs::ImageConstPtr& original_image)
	{
		cv_bridge::CvImagePtr cv_ptr;
	
		try
		{
			cv_ptr = cv_bridge::toCvCopy(original_image,enc::BGR8);
		}
		catch( cv_bridge::Exception e)
		{
			ROS_ERROR("RP_DepthView::::cv_bridge exception : %s", e.what());
			return ;
		}
		//------------------------------------------------
		//		Stack of Frames
		//-----------------------------------------------
		cv::Mat frameDownSample;
		int h = cv_ptr->image.rows/scaler;
		//h*=2;
		//int h = 480;
		int w = cv_ptr->image.cols/scaler;
		//w*=2;
		//int w = 640;
		cv::resize(cv_ptr->image,frameDownSample,cv::Size(w,h));
		
		//------------------------------
		//------------------------------
		//	enhancement :
		//------------------------------
		//------------------------------
		//float sigma_s = 10.àf;
		//float sigma_r = 0.15f;
		//cv::detailEnhance(frameDownSample, frameDownSample, sigma_s, sigma_r);
		
		/// Do the operation new_image(i,j) = alpha*image(i,j) + beta
		float alpha = 1.0f;
		float beta = 0.0f;
		frameDownSample.convertTo( frameDownSample, -1, alpha, beta);
    
		//------------------------------
		//------------------------------
		
		
		mutexRES.lock();
		frames[3].insert(frames[3].begin(), frameDownSample);	
		mutexRES.unlock();
		
	}
	*/
	
	
	
	
	void loop()
	{
		clock_t timer = clock();
		int count_info = 0;
		
		bool goOnCloud = false;
		bool publishCloud = false;
		
		//cv::namedWindow("DEPTHVIEW",CV_WINDOW_AUTOSIZE);
		cv::Mat frameProcessed;
		sensor_msgs::PointCloud2 cloudProcessed;
		sensor_msgs::PointCloud2 cloudFilteredProcessed;
		pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
		
		
		mutexRES.lock();
		while(continuer)
		{
			mutexRES.unlock();
			
			/*
			cv::Mat frameToProcess[4];
			bool goOn = true;
			
			for(int i=0;i<4;i++)
			{
				if(frames[i].size())
				{
					goOn = true;
				}
				else
				{
					goOn = false;
					break;
				}
			}
			*/
			
			if(clouds.size())
			{
				goOnCloud = true;
			}
			else
			{
				goOnCloud = false;
				publishCloud = false;
			}
			
			/*
			if(goOn)
			{
				
				mutexRES.lock();
				for(int i=0;i<4;i++)
				{
					frames[i][frames[i].size()-1].copyTo(frameToProcess[i]);
				}
				mutexRES.unlock();
				
				//----------------------------------------------------
				//----------------------------------------------------
				//CLIPPING :
				//----------------------------------------------------
				//----------------------------------------------------
				
				cv::hconcat(frameToProcess[3],frameToProcess[2],frameProcessed);
				cv::hconcat(frameProcessed,frameToProcess[1],frameProcessed);
				cv::hconcat(frameProcessed,frameToProcess[0],frameProcessed);
				
				//----------------------------------------------------
				//----------------------------------------------------
				//----------------------------------------------------
				
				mutexRES.lock();
				for(int i=4;i--;)	frames[i].clear();
				mutexRES.unlock();
				
#ifdef debug_v0		
				count_info++;
		
				if(count_info>10)
				{
					ROS_INFO("RP_DepthView::FPS : %f.", CLOCKS_PER_SEC/((float)(clock()-timer)) );
					count_info = 0;
				}
				timer = clock();
#endif 

				//----------------------------
				//		Publisher
				//----------------------------
		
				//--------------------------------------------
				//
				//	Convert CvImage to ROS image message and publish it to camera/image_processed topic
				//
				//--------------------------------------------
					
				sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frameProcessed).toImageMsg();
				img_pub.publish(msg);
				cv::imshow("DEPTHVIEW", frameProcessed);
			
	
			}
			*/
			
			
			if(goOnCloud)
			{
				mutexRES.lock();
				cloudProcessed = clouds[clouds.size()-1];
				mutexRES.unlock();
				
				/*
				Processing of the PointCloud :
				*/
				
				//conversion from sensor_msgs::PointCloud2 to pcl::PointCloud<T>
				//pcl::PointCloud<pcl::PointXYZRGB> cloud;
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudptr( new pcl::PointCloud<pcl::PointXYZRGB>() );

				// Convert to PCL data type
				pcl::fromROSMsg(cloudProcessed, *cloudptr);
				
				/*
				1) segmentation into the floor and the rest...
				*/
				/*
				pcl::ModelCoefficients coefficients;
				coefficients.values = std::vector<float>(4,0.0f);
				coefficients.values[0] = 0.0f;
				coefficients.values[1] = 1.0f;
				coefficients.values[2] = 0.0f;
				coefficients.values[3] = 10.0f;
				//pcl::PointIndices inliers;
				pcl::PointIndices::Ptr inliersptr( new pcl::PointIndices() );
				// Create the segmentation object
				pcl::SACSegmentation<pcl::PointXYZRGB> seg;
				//seg.setOptimizeCoefficients(true);
				seg.setOptimizeCoefficients(false);
				seg.setModelType(pcl::SACMODEL_PLANE);
				seg.setMethodType(pcl::SAC_RANSAC);
				seg.setDistanceThreshold(0.1);
				//seg.setDistanceThreshold(0.1);

				seg.setInputCloud(cloudptr);
				seg.segment(*inliersptr, coefficients);
				
				
				if (inliersptr->indices.size () == 0)
				{
					PCL_ERROR ("Could not estimate a planar model for the given dataset.");
					//return (-1);
				}
				
				
				std::cerr << "Model coefficients: ";
				for(int i=0;i<coefficients.values.size();i++)
					std::cerr << coefficients.values[i] << " " ;
				std::cerr << std::endl;
				
				
				
				//let us extract the rest of the data : 
				// Create the filtering object
			  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
				// Extract the outliers
				extract.setInputCloud(cloudptr);
				extract.setIndices(inliersptr);
				extract.setNegative(true);
				extract.filter(cloud_filtered);
				*/
				pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond( new pcl::ConditionAnd<pcl::PointXYZRGB>() );
				range_cond->addComparison( pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr( new pcl::FieldComparison<pcl::PointXYZRGB>( "y",pcl::ComparisonOps::LT, this->tresholdFilter) ) );
				pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
				condrem.setCondition( range_cond);
				condrem.setInputCloud( cloudptr );
				condrem.setKeepOrganized( true);
				condrem.filter( cloud_filtered);
				
				//regularization to remove the NaNs...
				/*
				for(int i=0;i<cloud_filtered.points.size();i++)
				{
					bool deleted = false;
					
					if( std::isnan(cloud_filtered.points[i].x) || std::isnan(cloud_filtered.points[i].y) || std::isnan(cloud_filtered.points[i].z) )
					{
						deleted = true;
						cloud_filtered.points.erase(cloud_filtered.points.begin()+i);
					}
					
					if(deleted)
					{
						i--;
					}
				}*/
				std::vector<int> nanindices;
				pcl::removeNaNFromPointCloud( cloud_filtered,cloud_filtered, nanindices);
				
				
				/*
				2) let us publish those pointclouds...
				*/
				publishCloud = true;
				//pcl_conversions::fromPCL(cloud_filtered, cloudProcessed);
				
				
				pcl::toROSMsg(cloud_filtered, cloudFilteredProcessed);
				/*
				std::cerr << "AFTER FILTERING : " << cloudFilteredProcessed.height*cloudFilteredProcessed.width << " points." << std::endl;
				*/
				cloud_pub.publish(cloudProcessed);
				cloudFiltered_pub.publish(cloudFilteredProcessed);
			}
			else
			{
				publishCloud = false;
			}
			
			if(char(cv::waitKey(1))=='q')			
			{
				this->setContinuer(false);
			}
			
			
			if(publishCloud)
			{
				/*
				3) let us extract the positions of the remaining points and publish those as an OpenCV matrix :
				*/
				/*
				Shape of the matrix :
				- first column : [nbrPoints ; 0 ]
				- other columns : [ position X ; position Y ]
				*/
				//int nbrPoints = cloudFilteredProcessed.height*cloudFilteredProcessed.width;
				int nbrPoints = cloud_filtered.points.size();
				//cv::Mat obstaclesPositions = cv::Mat::zeros(2,nbrPoints+1,CV_32F);
				cv::Mat obstaclesPositions = cv::Mat::zeros(3,nbrPoints+1,CV_32F);
				obstaclesPositions.at<float>(0,0) = nbrPoints;
				
				for(int i=1;i<nbrPoints+1;i++)
				{
					obstaclesPositions.at<float>(0,i) = cloud_filtered.points[i-1].x;
					obstaclesPositions.at<float>(1,i) = cloud_filtered.points[i-1].y;
					obstaclesPositions.at<float>(2,i) = cloud_filtered.points[i-1].z;
				}
				
				//std::cout << " OBS : " << obstaclesPositions << std::endl;
				
				//let us publish :
				sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), enc::TYPE_32FC1, obstaclesPositions).toImageMsg();
				obs_pub.publish(msg);
			}
			
			
			mutexRES.lock();
		}
		mutexRES.unlock();

	}
	
	inline void setContinuer(bool c)
	{
		mutexRES.lock();
		continuer = c;
		mutexRES.unlock();	
	}
	
	

};

#endif
