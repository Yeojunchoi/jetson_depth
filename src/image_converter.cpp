/* bridge_test.cpp
 * subscribe /camera/depth/image_raw
 * publisher /bridge_test/output_video
*/
#include <std_msgs/Float32.h> 
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointPull.h>
#include "mavros_msgs/PositionTarget.h"
#include "nav_msgs/Odometry.h"
#include "cmath"
#include <iostream>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Vector3.h"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <mavros_msgs/VFR_HUD.h>
#include <sensor_msgs/NavSatFix.h>



static const std::string OPENCV_WINDOW = "Image window";

int width_block = 15;
int height_block = 9;
float fov_h=87;
float fov_v=87*(480.0/848.0);
cv::Mat depth=cv::Mat::zeros(height_block,width_block,CV_32F);
cv::Mat depth_cost=cv::Mat::zeros(height_block,width_block,CV_32F);
cv::Mat binary=cv::Mat::zeros(height_block,width_block,CV_8UC1);

sensor_msgs::NavSatFix local_pose_target;
geometry_msgs::PoseStamped local_pos;
mavros_msgs::WaypointList waypoint;
mavros_msgs::VFR_HUD instrument;
mavros_msgs::State current_state;
void local_pos_target_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)  /////mavros_msgs::(mavros msg type)::ConstPtr
{
    local_pose_target = *msg;
}

void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)  /////mavros_msgs::(mavros msg type)::ConstPtr
{
    local_pos = *msg;
}

void waypoint_cb(const mavros_msgs::WaypointList::ConstPtr& msg)  /////mavros_msgs::(mavros msg type)::ConstPtr
{
    waypoint= *msg;
    std::cout << waypoint.waypoints[waypoint.current_seq].x_lat << std::endl;
    std::cout << waypoint.waypoints[waypoint.current_seq].y_long << std::endl;
    std::cout << waypoint.waypoints[waypoint.current_seq].z_alt << std::endl;
   
}

void instrument_cb(const mavros_msgs::VFR_HUD::ConstPtr& msg)  /////mavros_msgs::(mavros msg type)::ConstPtr
{
    instrument= *msg;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}



struct EulerAngles {
    double roll, pitch, yaw;
};

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/depth/image_rect_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/depth/lklk", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat &image = cv_ptr->image;
    cv::Mat blurred_image;                                                             ///////////////@#$%@$#%@$#%        블러 추가부분
    cv::GaussianBlur(image,blurred_image,cv::Size(9,9),1.0); 
    int width_block = 15;
    int height_block = 9;
    int width_cut =56;
    int height_cut=53;
    double margin_dist =1.0; 
    cv::Mat depth_cal=cv::Mat::zeros(height_block,width_block,CV_32FC1);
    cv::Mat binary_cal=cv::Mat::zeros(height_block,width_block,CV_8UC1); 
    //std::cout << image <<std::endl;
    //std::cout << blurred_image<<std::endl;
    for(int k=0;k<height_block;k++)
        {
            for(int l=0;l<width_block;l++)
            {
                for(int i=0;i<height_cut;i++)
                {
                    for(int j=0;j<width_cut;j++)
                    {
                      depth_cal.at<float>(k,l)+=blurred_image.at<float>(i+1+k*height_cut,j+4+l*width_cut)/1000.0;
                      depth_cost.at<float>(k,l)+=11.0-blurred_image.at<float>(i+1+k*height_cut,j+4+l*width_cut)/1000.0;      //리얼센스 최대거리 11미터
                    }
                }
                if(depth_cal.at<float>(k,l)/(width_cut*height_cut) - margin_dist<0)           //// 평균*1000으로 나눠서 미터단위 변환
                  {
                      binary_cal.at<uint8_t>(k,l)=1;
                  }
                else
                binary_cal.at<uint8_t>(k,l)=0;                                       /////////////////평균으로 mat을 만들면 아주 작은물체가 가까이있으면 인식을 못할수도 있음
            }
        }///for
        //depth_cal=~depth_cal;
       depth= depth_cal/(width_cut*height_cut);                               // 나누기 픽셀수, 나누기 1000단위
       depth_cost=depth_cost/(width_cut*height_cut);
       binary = binary_cal;
       
      
    }//callback

};//class




int main(int argc, char** argv)
{
  ros::init(argc, argv, "bridge_test");
  ros::NodeHandle nh;
  ros::Subscriber local_target_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 10,local_pos_target_cb);
  ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10,local_pos_cb);
  ros::Subscriber mission_waypoint = nh.subscribe<mavros_msgs::WaypointList>
            ("mavros/mission/waypoints", 10,waypoint_cb);
  ros::Subscriber climb = nh.subscribe<mavros_msgs::VFR_HUD>
            ("mavros/vfr_hud", 10,instrument_cb);
  ros::Publisher movement = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
  ros::ServiceClient waypoints = nh.serviceClient<mavros_msgs::WaypointPull>
            ("mavros/mission/pull");

  ros::Time::init();
  ros::Rate loop_rate(10);
  ImageConverter ic;
  double minVal;
  double maxVal;
  cv::Point minPixel;
  cv::Point maxPixel;

  



  while(ros::ok())
  {
    std::cout << depth <<"\n"<< std::endl;
    std::cout << depth_cost << "\n" << std::endl;
    std::cout << binary << std::endl;
    
               
    

    ros::Time last_request = ros::Time::now();

    tf2::Quaternion q (local_pos.pose.orientation.x,local_pos.pose.orientation.y,local_pos.pose.orientation.z,local_pos.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    tfScalar current_roll,current_pitch,current_yaw;
    m.getRPY(current_roll,current_pitch,current_yaw,1);                             ///////////// 마지막에 1 넣어줘야댐(sovlver 설정)

    cv::Mat cost_mat=cv::Mat::zeros(height_block,width_block,CV_32FC1);                     ///////////////@#$%@$#%@$#%        블러 추가부분
    cv::Mat blurred_depth_cost;                                                             ///////////////@#$%@$#%@$#%        블러 추가부분
    cv::GaussianBlur(depth_cost,blurred_depth_cost,cv::Size(9,9),1.0);                      ///////////////@#$%@$#%@$#%        블러 추가부분
    //#$#@$#@std::cout <<blurred_depth_cost <<std::endl;
    float Kd =100.0;
    float Kh =1.0;
    float Ke =1.0;


    for(int i=0;i<height_block;i++){   ///i 0~7 8칸
      for(int j=0;j<width_block;j++){   ///j 0~13 14칸
          
          cost_mat.at<float>(i,j)=Kd*blurred_depth_cost.at<float>(i,j)+Kh*abs((fov_h/2.0)-(fov_h/width_block)*(j+1/2.0))+Ke*abs((fov_v/2.0)-(fov_v/height_block)*(((height_block-1)-i)+1/2.0));
                              //#$%^ 블러된 depth맵으로 바꿈 #$%^
          
        
        }
      }
    
    //%^&%^&%std::cout << depth_cost << std::endl << std::endl;
    //%^&%^&%std::cout << blurred_depth_cost << std::endl;
    //&%^&%^&std::cout << cost_mat << std::endl << std::endl;
    std::cout << cost_mat << std::endl;
    cv::minMaxLoc(cost_mat,&minVal,&maxVal,&minPixel,&maxPixel,~binary);      ///////////////masking layer CV_8UC1으로 해야댐
    
    std:: cout << "minmax = \n" << minVal <<";" << minPixel <<";" << std::endl << std::endl;
    
    

          /////////direction_pixel
    float azimuth= ((minPixel.x-7)*(fov_h/15.0))*(M_PI/180.0);//*(M_PI/180.0);  //degree
    float elevation= ((4-minPixel.y)*(fov_v/9.0))*(M_PI/180.0);
    float d=5.0;
    float target_heading= -azimuth+current_yaw;
    
    tf2::Quaternion target_q;
    target_q.setRPY(0,0,target_heading);         //target_heading
   
    geometry_msgs::PoseStamped target_xyz;//d*cos(elevation)*cos(azimuth),d*cos(elevation)*sin(azimuth),d*sin(elevation));
    target_xyz.pose.position.x=d*cos(elevation)*cos((target_heading))+local_pos.pose.position.x;
    target_xyz.pose.position.y=d*cos(elevation)*sin((target_heading))+local_pos.pose.position.y;
    target_xyz.pose.position.z =d*sin(elevation)+local_pos.pose.position.z;
    target_xyz.pose.orientation.x=target_q.x();
    target_xyz.pose.orientation.y=target_q.y();
    target_xyz.pose.orientation.z=target_q.z();
    target_xyz.pose.orientation.w=target_q.w();
    //std::cout << target_xyz.pose << std::endl;
  
    
    movement.publish(target_xyz);
    
    loop_rate.sleep();
    ros::spinOnce();
  }
  
  return 0;
}