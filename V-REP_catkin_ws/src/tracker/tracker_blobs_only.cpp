/*
 * main.cpp
 *
 *  Created on: Jan 12, 2016
 *      Author: firas
 */

#include <visp3/gui/vpDisplayGDI.h>
//#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpImage.h>

#include <visp_ros/vpROSGrabber.h>
#include <visp_ros/vpROSRobot.h>

#include <visp3/core/vpConfig.h>
#include <visp3/blob/vpDot2.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/vision/vpPose.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/mbt/vpMbEdgeTracker.h>
#include <visp3/mbt/vpMbEdgeKltTracker.h>

#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>


#include <algorithm>
#include <iostream>
#include <vector>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include<geometry_msgs/Vector3Stamped.h>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>

#include <sstream>

void learnBlob(vpDot2 &blob, vpROSGrabber &g, bool &init_done, vpImage<unsigned char> &I)
{
    // Learn the characteristics of the blob to auto detect
    blob.setGraphics(true);
    std::cout << "Initializing the blobs..." << std::endl;
    blob.setGraphicsThickness(1);
    vpImagePoint germ;
    while (! init_done) {
//        g.acquire(I);
//        vpDisplay::display(I);
        vpDisplay::displayText(I, vpImagePoint(10,10), "Please select a blob to initiate the track", vpColor::red);
      if (vpDisplay::getClick(I, germ, false)) {
        blob.initTracking(I, germ);
        init_done = true;
      }
//      vpDisplay::flush(I);
    }
    blob.track(I);
    std::cout << "Blob characteristics: " << std::endl;
    std::cout << " width : " << blob.getWidth() << std::endl;
    std::cout << " height: " << blob.getHeight() << std::endl;
#if VISP_VERSION_INT > VP_VERSION_INT(2,7,0)
    std::cout << " area: " << blob.getArea() << std::endl;
#endif
    std::cout << " gray level min: " << blob.getGrayLevelMin() << std::endl;
    std::cout << " gray level max: " << blob.getGrayLevelMax() << std::endl;
    std::cout << " grayLevelPrecision: " << blob.getGrayLevelPrecision() << std::endl;
    std::cout << " sizePrecision: " << blob.getSizePrecision() << std::endl;
    std::cout << " ellipsoidShapePrecision: " << blob.getEllipsoidShapePrecision() << std::endl;
}

void initiateBlobUsingDefaultParameters(vpDot2 &blob)
{
    // Set blob characteristics for the auto detection
    blob.setWidth(50);
    blob.setHeight(50);
#if VISP_VERSION_INT > VP_VERSION_INT(2,7,0)
    blob.setArea(1700);
#endif
    blob.setGrayLevelMin(0);
    blob.setGrayLevelMax(30);
    blob.setGrayLevelPrecision(0.8);
    blob.setSizePrecision(0.65);
    blob.setEllipsoidShapePrecision(0.65);
}
void addManuallySelectedBlobToTheList (vpDot2 &blob, std::list<vpDot2> &blob_list)
{
    bool blob_is_there=false;
    double thresh=0.001;
    vpImagePoint O, O_blob;
    O_blob=blob.getCog();
    std::cout << "Manually selected blob coordinates x=" << O_blob.get_i() << "y=" << O_blob.get_j() << std::endl;
    for(std::list<vpDot2>::iterator it=blob_list.begin(); it != blob_list.end(); ++it) {
        O=(*it).getCog();
        std::cout << "blob coordinates x=" << O.get_i() << "y=" << O.get_j() << std::endl;
        if (fabs(O.get_i()-O_blob.get_i())<thresh && fabs(O.get_j()-O_blob.get_j())<thresh)
                blob_is_there=true;
    }
    if (!blob_is_there)
        blob_list.push_back(blob);
}

void SelectManuallyNonDetectedBlobs (std::list<vpDot2> &blob_list, vpROSGrabber &g, vpImage<unsigned char> &I)
{
    vpDot2 blob_temp;
    vpImagePoint germ_temp;
    std::cout << "Please select the non-detected blobs" << std::endl;
    while (blob_list.size()<4){
        g.acquire(I);
        vpDisplay::display(I);
        vpDisplay::displayText(I, vpImagePoint(10,10), "Please select the non-detected blob(s)", vpColor::red);
        for(std::list<vpDot2>::iterator it=blob_list.begin(); it != blob_list.end(); ++it) {
                (*it).setGraphics(true);
                (*it).setGraphicsThickness(3);
                (*it).track(I);
              }
        if (vpDisplay::getClick(I, germ_temp, false)) {
              blob_temp.initTracking(I, germ_temp);
              blob_list.push_back(blob_temp);
        }
        vpDisplay::flush(I);
    }
}

std::vector<double> CrossProduct1D(const std::vector<double> &a, const std::vector<double> &b)
{
  std::vector<double> r (3);
  r[0] = a[1]*b[2]-a[2]*b[1];
  r[1] = a[2]*b[0]-a[0]*b[2];
  r[2] = a[0]*b[1]-a[1]*b[0];
  return r;
}

std::vector<vpDot2> reorderBlobList(vpDot2 &blob, std::list<vpDot2> &blob_list, vpImage<unsigned char> &I)
{
    std::vector<vpDot2> dot(4);
    std::vector<vpImagePoint> point(4); int i=0;
    vpImagePoint manually_selected_point;
    std::vector<double> origin_to_point_distance(4);
    manually_selected_point=blob.getCog();
    for(std::list<vpDot2>::iterator it=blob_list.begin(); it != blob_list.end(); ++it) {
        point[i]=(*it).getCog();
        origin_to_point_distance[i]=pow((manually_selected_point.get_i()-point[i].get_i()),2) + pow((manually_selected_point.get_j()-point[i].get_j()),2);
        ++i;
    }
    std::vector<double>::iterator biggest = std::max_element(origin_to_point_distance.begin(), origin_to_point_distance.end());
    std::vector<double>::iterator smallest = std::min_element(origin_to_point_distance.begin(), origin_to_point_distance.end());
    int origin_position=std::distance(origin_to_point_distance.begin(), smallest);
    int diagonal_point_position=std::distance(origin_to_point_distance.begin(), biggest);
    // Set the first blob in dot to coprrespond to the origin
    std::list<vpDot2>::iterator it_origin=blob_list.begin();
    if (origin_position!=0){
        for (i=1; i<=origin_position; ++i)
            ++it_origin;
    }
    dot[0]=*it_origin;
    // Set the third blob in dot to coprrespond to the diagonal point
    std::list<vpDot2>::iterator it_diagonal_point_position=blob_list.begin();
    if (diagonal_point_position!=0){
        for (i=1; i<=diagonal_point_position; ++i)
            ++it_diagonal_point_position;
    }
    dot[2]=*it_diagonal_point_position;
    // Find out which of the two remaining blobs correspond to which point
    i=0;
    int third_point=10;
    int fourth_point=0;
    while (i<4){
        if (i!=origin_position && i!=diagonal_point_position &&third_point==10){
            third_point=i;
        }
        else{
            if (i!=origin_position && i!=diagonal_point_position &&third_point!=10)
                fourth_point=i;
        }
        ++i;
    }
    std::vector<double> diagonal(3), side(3), cross_product(3);
    diagonal[0]=point[diagonal_point_position].get_i()-point[origin_position].get_i();
    diagonal[1]=point[diagonal_point_position].get_j()-point[origin_position].get_j();
    diagonal[2]=0;
    side[0]=point[third_point].get_i()-point[origin_position].get_i();
    side[1]=point[third_point].get_j()-point[origin_position].get_j();
    side[2]=0;
    cross_product=CrossProduct1D(diagonal, side);
    if (cross_product[2]<0){
        std::list<vpDot2>::iterator it_third_point=blob_list.begin();
        if (third_point!=0){
            for (i=1; i<=third_point; ++i)
                ++it_third_point;
        }
        dot[1]=*it_third_point;
        std::list<vpDot2>::iterator it_fourth_point=blob_list.begin();
        if (fourth_point!=0){
            for (i=1; i<=fourth_point; ++i)
                ++it_fourth_point;
        }
        dot[3]=*it_fourth_point;
    }
    else {
        std::list<vpDot2>::iterator it_third_point=blob_list.begin();
        if (third_point!=0){
            for (i=1; i<=third_point; ++i)
                ++it_third_point;
        }
        dot[3]=*it_third_point;
        std::list<vpDot2>::iterator it_fourth_point=blob_list.begin();
        if (fourth_point!=0){
            for (i=1; i<=fourth_point; ++i)
                ++it_fourth_point;
        }
        dot[1]=*it_fourth_point;
    }
    return dot;
}

void computePose(std::vector<vpPoint> &point, std::vector<vpDot2> &dot,
                 const vpCameraParameters &cam, bool init, vpHomogeneousMatrix &cMo)
{
  vpPose pose;     double x=0, y=0;
  for (unsigned int i=0; i < point.size(); i ++) {
    vpPixelMeterConversion::convertPoint(cam, dot[i].getCog(), x, y);
    point[i].set_x(x);
    point[i].set_y(y);
    pose.addPoint(point[i]);
  }
  if (init == true) {
      vpHomogeneousMatrix cMo_dem;
      vpHomogeneousMatrix cMo_lag;
    pose.computePose(vpPose::DEMENTHON, cMo_dem);
    pose.computePose(vpPose::LAGRANGE, cMo_lag);
    double residual_dem = pose.computeResidual(cMo_dem);
      double residual_lag = pose.computeResidual(cMo_lag);
      if (residual_dem < residual_lag)
          cMo = cMo_dem;
      else
          cMo = cMo_lag;
  }
  pose.computePose(vpPose::VIRTUAL_VS, cMo);
}

std::vector<vpDot2> initializeBlobsToTrack(vpROSGrabber &g, bool &init_done, bool &learn, vpImage<unsigned char> &I)
{
    vpDot2 blob, blob_copy;
    if (learn)
          learnBlob(blob, g, init_done, I);
    else
        initiateBlobUsingDefaultParameters(blob);
    blob_copy=blob;

    std::list<vpDot2> blob_list;
    std::cout << "Size of image search area is: " << I.getWidth() << I.getHeight() << std::endl;
    vpImagePoint manually_selected_point;
    manually_selected_point=blob.getCog();
    std::cout << "blob u: " << manually_selected_point.get_u() << "\n blob v: " << manually_selected_point.get_v() << "blob height: " << blob.getHeight() << std::endl;

    blob.searchDotsInArea(I, manually_selected_point.get_u()-5*blob.getHeight(), manually_selected_point.get_v()-5*blob.getHeight(), manually_selected_point.get_u()+5*blob.getHeight(), manually_selected_point.get_u()+5*blob.getHeight(), blob_list);
    addManuallySelectedBlobToTheList (blob, blob_list);  //Add manually selected Blob to the list if not there

    std::cout << "Number of detected blob: " << blob_list.size() << std::endl;
    if (blob_list.size()<4)
        SelectManuallyNonDetectedBlobs (blob_list, g, I);
    std::vector<vpDot2> dot(4);
    dot= reorderBlobList(blob_copy, blob_list, I);

    std::cout << "All blobs are now selected" << std::endl;
    return dot;
}

void getobjectcenter (std::vector<vpPoint> &point, double x[])
{
    x[0]=0;
    x[1]=0;
    for (unsigned int i=0; i < point.size(); i ++) {
      x[0] =x[0] + point[i].get_x();
      x[1] =x[1] + point[i].get_y();
    }
    x[0]=x[0]/point.size();
    x[1]=x[1]/point.size();
}

void trackBlobs (std::vector<vpDot2> &dot, std::vector<vpPoint> &point, vpCameraParameters &cam, vpHomogeneousMatrix &cMo, bool &init, vpImage<unsigned char> &I, geometry_msgs::PointStamped  &blob_loss)
{
    for (int i=0; i<4; ++i){
        try{
            dot[i].setGraphics(true);
            dot[i].setGraphicsThickness(3);
            dot[i].track(I);
        }
        catch(...) {
            blob_loss.point.x=1;
        }
          }
    computePose(point, dot, cam, init, cMo);
//    double x_double[]={0, 0};
//    getobjectcenter (point, x_double);
//    std::cout << x_double[0] << "," << x_double[1] << std::endl;
//    x.point.x=x_double[0];
//    x.point.y=x_double[1];
    //write code to get center of tracked object
    vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::none);
}

void rotationToQuaternionAndFillPose (vpHomogeneousMatrix &cMo, geometry_msgs::PoseStamped &P)
{//eigen-eigen-b30b87236a1b::
    Eigen::Matrix3d R;
    Eigen::Quaterniond q;
    R(0,0)=cMo[0][0];
    R(0,1)=cMo[0][1];
    R(0,2)=cMo[0][2];
    R(1,0)=cMo[1][0];
    R(1,1)=cMo[1][1];
    R(1,2)=cMo[1][2];
    R(2,0)=cMo[2][0];
    R(2,1)=cMo[2][1];
    R(2,2)=cMo[2][2];
//    q.Quaternion(R);
    q=R;
    P.pose.orientation.x=q.x();
    P.pose.orientation.y=q.y();
    P.pose.orientation.z=q.z();
    P.pose.orientation.w=q.w();

    P.pose.position.x=cMo[0][3];
    P.pose.position.y=cMo[1][3];
    P.pose.position.z=cMo[2][3];

}

int main(int argc, char **argv)
{

  // Reading input arguments and initiating the tracked object's naming
//  std::string topic_name = "/vrep/image_rect_0";/camera/image_raw
//  std::string cam_info_topic = "/vrep/camera_info";/camera/camera_info
  std::string topic_name = "/stereo/left/image_raw";
  std::string cam_info_topic = "/stereo/left/camera_info";
  int opt_tracker = 0;
  for (int i=0; i<argc; i++) {
      if (std::string(argv[i]) == "--topic")
          topic_name = std::string(argv[i+1]);
      if (std::string(argv[i]) == "--camInfoTopic")
          cam_info_topic = std::string(argv[i+1]);
      else if (std::string(argv[i]) == "--help") {
          std::cout << "\nUsage: " << argv[0] << " [--topic <Ros Topic name>] [--camInfoTopic <0=egde|1=keypoint|2=hybrid>] [--help]\n" << std::endl;
          return 0;
      }
  }

  // Initiating the grabber
  bool learn = true;
  bool init_done_1 = false;
  vpImage<unsigned char> I;
  vpROSGrabber g;
  vpCameraParameters cam;
  g.setImageTopic(topic_name);
  g.setCameraInfoTopic(cam_info_topic);
  g.setRectify(true);
  g.open(I);
  g.getCameraInfo(cam);
  std::cout << "VISP_HAVE_OPENCV" << std:: endl;
  //std::cout << a << std:: endl;

    try {
      //Initiate an image viewer
    #if defined(VISP_HAVE_OPENCV)
        vpDisplayOpenCV d(I);
        std::cout << "VISP_HAVE_OPENCV" << std:: endl;
    #elif defined(VISP_HAVE_GTK)
        vpDisplayGTK d(I);
        std::cout << "VISP_HAVE_GTK" << std:: endl;
    #elif defined(VISP_HAVE_GDI)
        vpDisplayGDI d(I);
        std::cout << "VISP_HAVE_GDI" << std:: endl;
    #elif defined(VISP_HAVE_D3D9)
        vpDisplayD3d d(I);
        std::cout << "VISP_HAVE_D3D9" << std:: endl;
    #else
        std::cout << "No image viewer is available..." << std:: endl;
    #endif

        //Initialize the blobs to track (gripper)
        g.acquire(I);
        vpDisplay::display(I);

        std::vector<vpDot2> dot_object_1(4);
        dot_object_1 = initializeBlobsToTrack(g, init_done_1, learn, I); // I should fix the reordering of blobs to take into consideration the angles rather than the distances
        vpDisplay::flush(I);

        //Initiate the 3D coordinates of the points corresponding to the detected blobs
        std::vector<vpPoint> point1;
        double L1=0.015;

        // point1.push_back( vpPoint(-L1, 0, L1) );
        // point1.push_back( vpPoint( L1, 0, L1) );
        // point1.push_back( vpPoint( L1, 0, -L1) );
        // point1.push_back( vpPoint(-L1, 0, -L1) );

        point1.push_back( vpPoint(-0.03, -0.02, 0) );
        point1.push_back( vpPoint(-0.03, +0.02, 0) );
        point1.push_back( vpPoint( 0.03, +0.02, 0) );
        point1.push_back( vpPoint( 0.03, -0.02, 0) );

        // Initiate the homogenous matrices and other parameters for the pose estimation
        vpHomogeneousMatrix cMo_1;
        bool init_1 = true;

        //Initiate the Ros Publishers
        ros::init(argc, argv, "rospublisher_track_and_estimate_pose_two_objects");
        ros::NodeHandle n_1;
        ros::Publisher object_pose_pub_1 = n_1.advertise<geometry_msgs::PoseStamped>("object_1/pose", 1);
        ros::NodeHandle blob_loss_ros;
        ros::Publisher blob_loss_pub = blob_loss_ros.advertise<geometry_msgs::PointStamped>("gripper_0/blob_loss", 1);
        ros::Rate loop_rate(10);
        int count = 0;

        //Track and compute pose
        std::cout << "A click to exit..." << std::endl;
        while(!vpDisplay::getClick(I, false) && ros::ok()){
          try {
            g.acquire(I);
            vpDisplay::display(I);
            geometry_msgs::PoseStamped P_1;
            P_1.header.stamp = ros::Time::now();
            P_1.header.frame_id=1;
            geometry_msgs::PointStamped blobs_loss;
            blobs_loss.header.stamp = ros::Time::now();
            blobs_loss.header.frame_id=1;

            blobs_loss.point.x=0;
            trackBlobs (dot_object_1, point1, cam, cMo_1, init_1, I, blobs_loss);
            //std::cout << "object 3D position: \n" << cMo_1 << std::endl;
            std::cout << "blob position: " << std::endl; 
            for (unsigned int i=0; i < dot_object_1.size(); i ++) {
				std::cout << dot_object_1[i].getCog() << ' ';
			}
			std::cout << std::endl;
            

            rotationToQuaternionAndFillPose (cMo_1, P_1);
            object_pose_pub_1.publish(P_1);
            blob_loss_pub.publish(blobs_loss);
            ros::spinOnce();
            loop_rate.sleep();
            ++count;
            vpDisplay::flush(I);
            if (init_1) init_1 = false; // turn off pose initialisation
//            vpTime::wait(40);
          }
          catch(...) {
            std::cout << "Catch an exception 1"<<std::endl;
          }
        }
      }
      catch(vpException e) {
        std::cout << "Catch an exception: " << e << std::endl;
      }
  return 0;
}
