#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>

# include <boost/filesystem.hpp>
#include <cv.h>
#include <highgui.h>
#include "cxcore.hpp"
#include <pthread.h>
#include <sys/time.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "loitorusbcam.h"
#include "loitorimu.h"

#include <sstream>


using namespace std;
using namespace cv;


ros::Publisher pub_imu;

/*
*  Used to construct the left and right eye images of cv :: Mat
*/
cv::Mat img_left;
cv::Mat img_right;

/*
*  The current timestamp of the left and right images
*/
timeval left_stamp,right_stamp;

/*
*  imu viewer
*/
bool visensor_Close_IMU_viewer=false;
bool imu_start_transfer=false;
void* imu_data_stream(void *)
{
	int counter=0;
	imu_start_transfer=false;


	while((!visensor_Close_IMU_viewer)&&!imu_start_transfer)usleep(1000);
	while(!visensor_Close_IMU_viewer)
	{
		if(visensor_imu_have_fresh_data())
			{
			counter++;
			// Display imu data every 20 frames
			/**
			if(counter>=20)
			{
				//cout<<"visensor_imudata_pack->a : "<<visensor_imudata_pack.ax<<" , "<<visensor_imudata_pack.ay<<" , "<<visensor_imudata_pack.az<<endl;
				float ax=visensor_imudata_pack.ax;
				float ay=visensor_imudata_pack.ay;
				float az=visensor_imudata_pack.az;
				cout<<"visensor_imudata_pack->a : "<<sqrt(ax*ax+ay*ay+az*az)<<endl;
				cout<<"imu_time : "<<visensor_imudata_pack.imu_time<<endl;
				cout<<"imu_time : "<<visensor_imudata_pack.system_time.tv_usec<<endl;
				counter=0;
			}
			**/
			sensor_msgs::Imu imu_msg;
			imu_msg.header.frame_id = "/imu";
			ros::Time imu_time;
			imu_time.sec=visensor_imudata_pack.system_time.tv_sec;
			imu_time.nsec=1000*visensor_imudata_pack.system_time.tv_usec;
			imu_msg.header.stamp = imu_time;
			imu_msg.header.seq=0;

			imu_msg.linear_acceleration.x=visensor_imudata_pack.ax;
			imu_msg.linear_acceleration.y=visensor_imudata_pack.ay;
			imu_msg.linear_acceleration.z=visensor_imudata_pack.az;
			imu_msg.angular_velocity.x=3.1415926f*visensor_imudata_pack.rx/180.0f;
			imu_msg.angular_velocity.y=3.1415926f*visensor_imudata_pack.ry/180.0f;
			imu_msg.angular_velocity.z=3.1415926f*visensor_imudata_pack.rz/180.0f;
			imu_msg.orientation.w=visensor_imudata_pack.qw;
			imu_msg.orientation.x=visensor_imudata_pack.qx;
			imu_msg.orientation.y=visensor_imudata_pack.qy;
			imu_msg.orientation.z=visensor_imudata_pack.qz;

			pub_imu.publish(imu_msg);
		}
		usleep(10);
	}
	pthread_exit(NULL);
}



void loadIntrinsicsFile(string config_file_path, string &camera_name, sensor_msgs::CameraInfoPtr cam_info)
{

	if (camera_calibration_parsers::readCalibration (config_file_path, camera_name, *cam_info))
	{
		cam_info->header.frame_id = "/" + camera_name;
		ROS_INFO_STREAM ("Loaded intrinsics parameters for [" << camera_name << "]");
	}
}


bool saveIntrinsicsFile(string config_file_path, string &camera_name, sensor_msgs::CameraInfoPtr cam_info)
{
	if (camera_calibration_parsers::writeCalibration (config_file_path, camera_name, *cam_info))
	{
		ROS_INFO_STREAM("Saved intrinsics parameters for [" << camera_name << "] to " << config_file_path);
		return true;
	}
	return false;
}


bool setCamInfo (sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &rsp , string config_file_path, string &camera_name, sensor_msgs::CameraInfoPtr cam_info )
{
	*cam_info = req.camera_info;
	cam_info->header.frame_id = "/" + camera_name;
	rsp.success = saveIntrinsicsFile(config_file_path, camera_name, cam_info);
	rsp.status_message = (rsp.success) ?
						 "successfully wrote camera info to file" :
						 "failed to write camera info to file";
	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "loitor_stereo_visensor");

	ros::NodeHandle local_nh("~");

	string configPath= "";
	std::string camera_id = "LTR000";
	std::string configFile = "Loitor_VISensor_Setups.txt";

	/************************ Start Cameras ************************/

	local_nh.param<string> ("config_path", configPath, configPath);
	local_nh.param<string> ("camera_id", camera_id, camera_id);
	local_nh.param<string> ("config_file", configFile, configFile);

	ROS_INFO_STREAM("Config path:" << configPath);
	ROS_INFO_STREAM("Camera id:" << camera_id);
	ROS_INFO_STREAM("Config File:" << configFile);

	//by default settins path is where the config file resides
	boost::filesystem::path settingsPathB = boost::filesystem::path(configPath);
	if(!boost::filesystem::exists(settingsPathB)  || ! boost::filesystem::is_directory(settingsPathB))
	{
		ROS_FATAL_STREAM("The config path does not exists!: " << settingsPathB.string());
		exit( -1);
	}
	if(camera_id.empty())
	{
		ROS_FATAL_STREAM("The camera id can not be empty: " << settingsPathB.string());
		exit( -1);
	}

	settingsPathB = settingsPathB / camera_id;

	//create the dir if does not exists
	if(!boost::filesystem::exists(settingsPathB))
	{
		boost::filesystem::create_directory(settingsPathB);
	}

	string cam0IntrinsicFilePath = (settingsPathB / "cam0_camera_info.yaml").string();
	string cam1IntrinsicFilePath = (settingsPathB / "cam1_camera_info.yaml").string();
	ROS_INFO_STREAM("CAM0 intrinsic file:" << cam0IntrinsicFilePath);
	ROS_INFO_STREAM("CAM1 intrinsic file:" << cam1IntrinsicFilePath);

	sensor_msgs::CameraInfoPtr cameraInfo0Ptr = boost::make_shared<sensor_msgs::CameraInfo>();
	sensor_msgs::CameraInfoPtr cameraInfo1Ptr = boost::make_shared<sensor_msgs::CameraInfo>();
	string cam0Name = "left";
	string cam1Name = "right";


	loadIntrinsicsFile(cam0IntrinsicFilePath, cam0Name, cameraInfo0Ptr);
	loadIntrinsicsFile(cam1IntrinsicFilePath, cam1Name, cameraInfo1Ptr);
	visensor_load_settings((settingsPathB / configFile).string().c_str());



	int ros_eg_mode = 3;
	int ros_manual_exposure = 20;
	int ros_manual_gain = 20;
	int ros_min_auto_exposure = 0;
	int ros_max_auto_exposure = 255;

    //default rotation so X facing forward along optical axes, y left, z up
    float imu_rotation_q_w_ = 0.707106781187;
    float imu_rotation_q_x_ = 0.0;
    float imu_rotation_q_y_ = -0.707106781187;
    float imu_rotation_q_z_ = 0.0;

    double image_time_shift_sec = 0.0;
    local_nh.param<double> ("image_time_shift_sec", image_time_shift_sec, image_time_shift_sec);
    ros::Duration imageTimeShiftDuration(image_time_shift_sec);
    ROS_INFO("Image Time Shift set to %.6f [sec]", imageTimeShiftDuration.toSec());

    uint32_t publish_every_nth_image_ = 1;
    int temp;



    local_nh.param<int> ("publish_every_nth_image", temp, static_cast<int>(publish_every_nth_image_));
    if(temp <= 0)
    {
        ROS_ERROR("parameter publish_every_nth_image has to be > 0");
    }
    else
    {
        publish_every_nth_image_ = static_cast<uint32_t>(temp);
    }


    local_nh.param<int> ("eg_mode", ros_eg_mode, ros_eg_mode);
    local_nh.param<int> ("manual_exposure", ros_manual_exposure, ros_manual_exposure);
	local_nh.param<int> ("manual_gain", ros_manual_gain, ros_manual_gain);
	local_nh.param<int> ("min_auto_exposure", ros_min_auto_exposure, ros_min_auto_exposure);
    local_nh.param<int> ("max_auto_exposure", ros_max_auto_exposure, ros_max_auto_exposure);

    local_nh.param<float> ("imu_rotation_q_w", imu_rotation_q_w_, imu_rotation_q_w_);
    local_nh.param<float> ("imu_rotation_q_x", imu_rotation_q_x_, imu_rotation_q_x_);
    local_nh.param<float> ("imu_rotation_q_y", imu_rotation_q_y_, imu_rotation_q_y_);
    local_nh.param<float> ("imu_rotation_q_z", imu_rotation_q_z_, imu_rotation_q_z_);

    visensor_set_imu_rotation(imu_rotation_q_w_, imu_rotation_q_x_, imu_rotation_q_y_, imu_rotation_q_z_);
    visensor_set_auto_EG(ros_eg_mode);
	visensor_set_exposure(ros_manual_exposure);
	visensor_set_gain(ros_manual_gain);
	visensor_set_min_autoExp(ros_min_auto_exposure);
	visensor_set_max_autoExp(ros_max_auto_exposure);


	// Set the camera parameters manually
	//set_current_mode(5);
	//set_auto_EG(0);
	//visensor_set_exposure(50);
	//visensor_set_gain(100);
	//set_visensor_cam_selection_mode(2);
	//set_resolution(false);
	//set_fps_mode(true);

	// Save the camera parameters to the original configuration file
	//save_current_settings();

	ros::ServiceServer set_cam_info_srv_0 = local_nh.advertiseService<sensor_msgs::SetCameraInfo::Request, sensor_msgs::SetCameraInfo::Response> (
				cam0Name + "/set_camera_info", boost::bind(setCamInfo, _1, _2, cam0IntrinsicFilePath, cam0Name, cameraInfo0Ptr));
	ros::ServiceServer set_cam_info_srv_1 = local_nh.advertiseService<sensor_msgs::SetCameraInfo::Request, sensor_msgs::SetCameraInfo::Response> (
				cam1Name + "/set_camera_info", boost::bind(setCamInfo, _1, _2, cam1IntrinsicFilePath, cam1Name, cameraInfo1Ptr));

	int r = visensor_Start_Cameras();
	if(r<0)
	{
		printf("Opening cameras failed...\r\n");
		return r;
	}
	// Create an image to receive camera data
	if(!visensor_resolution_status)
	{
		img_left.create(cv::Size(640,480),CV_8U);
		img_right.create(cv::Size(640,480),CV_8U);
		img_left.data=new unsigned char[IMG_WIDTH_VGA*IMG_HEIGHT_VGA];
		img_right.data=new unsigned char[IMG_WIDTH_VGA*IMG_HEIGHT_VGA];
	}
	else
	{
		img_left.create(cv::Size(752,480),CV_8U);
		img_right.create(cv::Size(752,480),CV_8U);
		img_left.data=new unsigned char[IMG_WIDTH_WVGA*IMG_HEIGHT_WVGA];
		img_right.data=new unsigned char[IMG_WIDTH_WVGA*IMG_HEIGHT_WVGA];
	}
	float hardware_fps=visensor_get_hardware_fps();
	/************************** Start IMU **************************/
	int fd=visensor_Start_IMU();
	if(fd<0)
	{
		printf("open_port error...\r\n");
		return 0;
	}
	printf("open_port success...\r\n");
	usleep(100000);
	/************************ ************ ************************/

	//Create imu_data_stream thread
	pthread_t imu_data_thread;
	if(temp = pthread_create(&imu_data_thread, NULL, imu_data_stream, NULL))
	printf("Failed to create thread imu_data_stream\r\n");



	// imu publisher
	pub_imu = local_nh.advertise<sensor_msgs::Imu>("imu0", 200);

	// publish to those two topic
	image_transport::ImageTransport it(local_nh);
	image_transport::CameraPublisher pub0 = it.advertiseCamera(cam0Name + "/image_raw", 1);
	sensor_msgs::ImagePtr msg0;

	image_transport::ImageTransport it1(local_nh);
	image_transport::CameraPublisher pub1 = it1.advertiseCamera(cam1Name + "/image_raw", 1);
	sensor_msgs::ImagePtr msg1;

	// Use the camera hardware frame rate to set the publishing frequency
	ros::Rate loop_rate((int)hardware_fps);

	int static_ct=0;
    uint32_t running_counter = 0;

	timeval img_time_test,img_time_offset;
	img_time_test.tv_usec=0;
	img_time_test.tv_sec=0;
	img_time_offset.tv_usec=50021;
	img_time_offset.tv_sec=0;

	while (ros::ok())
	{
		imu_start_transfer=true;

		//cout<<"visensor_get_hardware_fps() ==== "<<visensor_get_hardware_fps()<<endl;

        running_counter++;
        if(running_counter % publish_every_nth_image_ == 0)
        {
            if(visensor_cam_selection==0)
            {

                visensor_imudata paired_imu=visensor_get_stereoImg((char *)img_left.data,(char *)img_right.data,left_stamp,right_stamp);

                // Display the timestamp of the synchronization data (in units of microseconds)
                //cout<<"left_time : "<<left_stamp.tv_usec<<endl;
                //cout<<"right_time : "<<right_stamp.tv_usec<<endl;
                //cout<<"paired_imu time ===== "<<paired_imu.system_time.tv_usec<<endl<<endl;
                //cout<<"visensor_get_hardware_fps() ==== "<<1.0f/visensor_get_hardware_fps()<<endl;

                cv_bridge::CvImage t_left=cv_bridge::CvImage(std_msgs::Header(), "mono8", img_left);
                cv_bridge::CvImage t_right=cv_bridge::CvImage(std_msgs::Header(), "mono8", img_right);

                // Plus timestamp (right_time=left_time)
                ros::Time msg_time;
                msg_time.sec=left_stamp.tv_sec;
                msg_time.nsec=1000*left_stamp.tv_usec;
                if(image_time_shift_sec != 0.0)
                {
                    ROS_INFO("TimeStamp before shift: %.6f", msg_time.toSec());
                    msg_time += imageTimeShiftDuration;
                    ROS_INFO("TimeStamp after  shift: %f.6", msg_time.toSec());
                }

                t_left.header.stamp = msg_time;
                t_left.header.seq=0;
                t_left.header.frame_id=cam0Name;

                ros::Time msg1_time;
                msg1_time.sec=left_stamp.tv_sec;
                msg1_time.nsec=1000*left_stamp.tv_usec;
                if(image_time_shift_sec != 0.0)
                {
                    msg1_time += imageTimeShiftDuration;
                }

                t_right.header.stamp = msg1_time;
                t_right.header.seq=0;
                t_right.header.frame_id=cam1Name;

                msg0 = t_left.toImageMsg();
                msg1 = t_right.toImageMsg();

                static_ct++;
                {
                    cameraInfo0Ptr->header = msg0->header;
                    cameraInfo1Ptr->header = msg1->header;
                    pub0.publish(msg0, cameraInfo0Ptr);
                    pub1.publish(msg1, cameraInfo1Ptr);
                    static_ct=0;
                }

                // Show timestamp
                //cout<<"left_time : "<<left_stamp.tv_usec<<endl;
                //cout<<"right_time : "<<right_stamp.tv_usec<<endl<<endl;

            }
            else if(visensor_cam_selection==1)
            {
                visensor_imudata paired_imu=visensor_get_rightImg((char *)img_right.data,right_stamp);

                // Display the timestamp of the synchronization data (in units of microseconds)
                //cout<<"right_time : "<<right_stamp.tv_usec<<endl;
                //cout<<"paired_imu time ===== "<<paired_imu.system_time.tv_usec<<endl<<endl;

                cv_bridge::CvImage t_right=cv_bridge::CvImage(std_msgs::Header(), "mono8", img_right);

                // 加时间戳
                ros::Time msg1_time;
                msg1_time.sec=right_stamp.tv_sec;
                msg1_time.nsec=1000*right_stamp.tv_usec;
                if(image_time_shift_sec != 0.0)
                {
                    ROS_INFO("TimeStamp before shift: %.6f", msg1_time.toSec());
                    msg1_time += imageTimeShiftDuration;
                    ROS_INFO("TimeStamp after  shift: %.6f", msg1_time.toSec());
                }
                t_right.header.stamp = msg1_time;
                t_right.header.seq=0;
                t_right.header.frame_id = cam1Name;

                msg1 = t_right.toImageMsg();

                cameraInfo1Ptr->header = msg1->header;
                pub1.publish(msg1, cameraInfo1Ptr);
            }
            else if(visensor_cam_selection==2)
            {
                visensor_imudata paired_imu=visensor_get_leftImg((char *)img_left.data,left_stamp);

                // Display the timestamp of the synchronization data (in units of microseconds)
                // cout<<"left_time : "<<left_stamp.tv_usec<<endl;
                // cout<<"paired_imu time ===== "<<paired_imu.system_time.tv_usec<<endl<<endl;

                cv_bridge::CvImage t_left=cv_bridge::CvImage(std_msgs::Header(), "mono8", img_left);

                // Plus timestamp
                ros::Time msg_time;
                msg_time.sec=left_stamp.tv_sec;
                msg_time.nsec=1000*left_stamp.tv_usec;
                if(image_time_shift_sec != 0.0)
                {
                    ROS_INFO("TimeStamp before shift: %.6f", msg_time.toSec());
                    msg_time += imageTimeShiftDuration;
                    ROS_INFO("TimeStamp after  shift: %.6f", msg_time.toSec());
                }
                t_left.header.stamp = msg_time;
                t_left.header.seq=0;
                t_left.header.frame_id = cam0Name;


                msg0 = t_left.toImageMsg();


                static_ct++;
                if(static_ct>=5)
                {
                    cameraInfo0Ptr->header = msg0->header;
                    pub0.publish(msg0, cameraInfo0Ptr);
                    static_ct=0;
                }
            }
        }

		ros::spinOnce();

		loop_rate.sleep();

	}

	/* shut-down viewers */
	visensor_Close_IMU_viewer=true;
	if(imu_data_thread !=0)
	{
		pthread_join(imu_data_thread,NULL);
	}

	cout<<endl<<"shutting-down Cameras"<<endl;

	/* close cameras */
	visensor_Close_Cameras();
	/* close IMU */
	visensor_Close_IMU();

	return 0;
}











