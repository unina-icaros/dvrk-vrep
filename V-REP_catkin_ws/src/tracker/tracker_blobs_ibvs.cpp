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

#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpProjectionDisplay.h>
#include <visp3/vs/vpServoDisplay.h>


#include <algorithm>
#include <iostream>
#include <vector>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>

#include <sstream>

void display_trajectory(const vpImage<unsigned char> &I, std::vector<vpPoint> &point, const vpHomogeneousMatrix &cMo,
                        const vpCameraParameters &cam);
void display_trajectory(const vpImage<unsigned char> &I, std::vector<vpPoint> &point, const vpHomogeneousMatrix &cMo,
                        const vpCameraParameters &cam)
{
  static std::vector<vpImagePoint> traj[4];
  vpImagePoint cog;
  for (unsigned int i = 0; i < 4; i++) {
    // Project the point at the given camera position
    point[i].project(cMo);
    vpMeterPixelConversion::convertPoint(cam, point[i].get_x(), point[i].get_y(), cog);
    traj[i].push_back(cog);
  }
  for (unsigned int i = 0; i < 4; i++) {
    for (unsigned int j = 1; j < traj[i].size(); j++) {
      vpDisplay::displayLine(I, traj[i][j - 1], traj[i][j], vpColor::green);
    }
  }
}

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

int main(int argc, char **argv)
{

  // Reading input arguments and initiating the tracked object's naming
//  std::string topic_name = "/vrep/image_rect_0";/camera/image_raw
//  std::string cam_info_topic = "/vrep/camera_info";/camera/camera_info


  // Initiating the grabber
  //std::cout << a << std:: endl;


        //Initialize the blobs to track (gripper)
    vpHomogeneousMatrix cdMo(0, 0, 0.2, 0, 0, 0);
    vpHomogeneousMatrix cMo(0.0, 0.0, 0.1, vpMath::rad(0), vpMath::rad(0), vpMath::rad(0));
    std::vector<vpPoint> point;
    point.push_back( vpPoint(-0.03, -0.02, 0) );
    point.push_back( vpPoint(-0.03, +0.02, 0) );
    point.push_back( vpPoint( 0.03, +0.02, 0) );
    point.push_back( vpPoint( 0.03, -0.02, 0) );
    vpServo task;
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);
    task.setLambda(0.5);
    point[0].set_x(0.5*-0.269642);
    point[0].set_y(0.5*-0.397292);
    point[1].set_x(0.5*-0.3343);
    point[1].set_y(0.5*-0.0760419);
    point[2].set_x(0.5*0.354937);
    point[2].set_y(0.5*-0.0609412);
    point[3].set_x(0.5*0.308907);
    point[3].set_y(0.5*-0.392029); 
      
    vpFeaturePoint p[4], pd[4];
    for (unsigned int i = 0; i < 4; i++) {
      //point[i].track(cdMo);
      vpFeatureBuilder::create(pd[i], point[i]);
      point[i].track(cMo);
      vpFeatureBuilder::create(p[i], point[i]);
      task.addFeature(p[i], pd[i]);
    }
    vpHomogeneousMatrix wMc, wMo;
    vpSimulatorCamera robot;
    robot.setSamplingTime(0.040);
    robot.getPosition(wMc);
    wMo = wMc * cMo;

    //Initiate the Ros Publishers
    ros::init(argc, argv, "rospublisher_track_and_estimate_pose_two_objects");
    ros::NodeHandle n_1;
    ros::Rate loop_rate(10);

    //Track and compute pose
    std::cout << "A click to exit..." << std::endl;
    vpImage<unsigned char> Iint(480, 640, 255);
    vpImage<unsigned char> Iext(480, 640, 255);
#if defined(VISP_HAVE_X11)
    vpDisplayX displayInt(Iint, 0, 0, "Internal view");
    vpDisplayX displayExt(Iext, 670, 0, "External view");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI displayInt(Iint, 0, 0, "Internal view");
    vpDisplayGDI displayExt(Iext, 670, 0, "External view");
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV displayInt(Iint, 0, 0, "Internal view");
    vpDisplayOpenCV displayExt(Iext, 670, 0, "External view");
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif
#if defined(VISP_HAVE_DISPLAY)
    vpProjectionDisplay externalview;
    for (unsigned int i = 0; i < 4; i++)
      externalview.insert(point[i]);
#endif
    vpCameraParameters cam(840, 840, Iint.getWidth() / 2, Iint.getHeight() / 2);
    vpHomogeneousMatrix cextMo(0, 0, 1, 0, 0, 0);
        while(ros::ok()){
                robot.getPosition(wMc);
      cMo = wMc.inverse() * wMo;
      for (unsigned int i = 0; i < 4; i++) {
        point[i].track(cMo);
        vpFeatureBuilder::create(p[i], point[i]);
      }
      vpColVector v = task.computeControlLaw();
      std::cout << "computed velocity is: " << v << std::endl;
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);
      vpDisplay::display(Iint);
      vpDisplay::display(Iext);
      display_trajectory(Iint, point, cMo, cam);
      vpServoDisplay::display(task, cam, Iint, vpColor::green, vpColor::red);
#if defined(VISP_HAVE_DISPLAY)
      externalview.display(Iext, cextMo, cMo, cam, vpColor::red, true);
#endif
      vpDisplay::flush(Iint);
      vpDisplay::flush(Iext);
      // A click to exit
      if (vpDisplay::getClick(Iint, false) || vpDisplay::getClick(Iext, false))
        break;
      vpTime::wait(robot.getSamplingTime() * 1000);
      }
      task.kill();
  return 0;
}
