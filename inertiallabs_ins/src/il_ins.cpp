#include <iostream>
#include <mutex>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>zz

// Inertial Labs source header
#include "ILDriver.h"

// adding message type headers
#include <inertiallabs_msgs/sensor_data.h>
#include <inertiallabs_msgs/ins_data.h>
#include <inertiallabs_msgs/gps_data.h>
#include <inertiallabs_msgs/gnss_data.h>
#include <inertiallabs_msgs/marine_data.h>

#define G 9.80665
// Publishers

struct Context
{
	ros::Publisher publishers[6];
	std::string imu_frame_id;
};

std::mutex data_mutex;
inertiallabs_msgs::sensor_data g_msg_sensor_data;
inertiallabs_msgs::ins_data g_msg_ins_data;
inertiallabs_msgs::gps_data g_msg_gps_data;
inertiallabs_msgs::gnss_data g_msg_gnss_data;
inertiallabs_msgs::marine_data g_msg_marine_data;

void publish_data(Context *context)
{
	static int seq = 0;
	seq++;

	// ros::Time timestamp = ros::Time::now();

	inertiallabs_msgs::sensor_data msg_sensor_data;
	inertiallabs_msgs::ins_data msg_ins_data;
	inertiallabs_msgs::gps_data msg_gps_data;
	inertiallabs_msgs::gnss_data msg_gnss_data;
	inertiallabs_msgs::marine_data msg_marine_data;
	sensor_msgs::Imu msg_standard_imu_data;

	// Lock, copy and unlock
	{
		std::lock_guard<std::mutex> lock(data_mutex);
		if (context->publishers[0].getNumSubscribers() > 0)
		{
			msg_sensor_data = g_msg_sensor_data;
		}

		if (context->publishers[1].getNumSubscribers() > 0)
		{
			msg_ins_data = g_msg_ins_data;
		}

		if (context->publishers[2].getNumSubscribers() > 0)
		{
			msg_gps_data = g_msg_gps_data;
		}

		if (context->publishers[3].getNumSubscribers() > 0)
		{
			msg_gnss_data = g_msg_gnss_data;
		}

		if (context->publishers[4].getNumSubscribers() > 0)
		{
			msg_marine_data = g_msg_marine_data;
		}

		if (context->publishers[5].getNumSubscribers() > 0)
		{
			msg_standard_imu_data.header = g_msg_ins_data.header;
			msg_standard_imu_data.header.frame_id = "base_link";
			msg_standard_imu_data.orientation = g_msg_ins_data.OriQuat;
			msg_standard_imu_data.angular_velocity = g_msg_sensor_data.Gyro;
			msg_standard_imu_data.linear_acceleration.x = g_msg_sensor_data.Accel.x * G;
			msg_standard_imu_data.linear_acceleration.y = g_msg_sensor_data.Accel.y * G;
			msg_standard_imu_data.linear_acceleration.z = g_msg_sensor_data.Accel.z * G;

			msg_standard_imu_data.orientation_covariance[0] = 8.512e-6;
			msg_standard_imu_data.orientation_covariance[1] = 0.0;
			msg_standard_imu_data.orientation_covariance[2] = 0.0;
			msg_standard_imu_data.orientation_covariance[3] = 0.0;
			msg_standard_imu_data.orientation_covariance[4] = 1.0263e-8;
			msg_standard_imu_data.orientation_covariance[5] = 0.0;
			msg_standard_imu_data.orientation_covariance[6] = 0.0;
			msg_standard_imu_data.orientation_covariance[7] = 0.0;
			msg_standard_imu_data.orientation_covariance[8] = 1.89219e-8;

			msg_standard_imu_data.angular_velocity_covariance[0] = 0.0007009;
			msg_standard_imu_data.angular_velocity_covariance[1] = 0.0;
			msg_standard_imu_data.angular_velocity_covariance[2] = 0.0;
			msg_standard_imu_data.angular_velocity_covariance[3] = 0.0;
			msg_standard_imu_data.angular_velocity_covariance[4] = 0.00186523;
			msg_standard_imu_data.angular_velocity_covariance[5] = 0.0;
			msg_standard_imu_data.angular_velocity_covariance[6] = 0.0;
			msg_standard_imu_data.angular_velocity_covariance[7] = 0.0;
			msg_standard_imu_data.angular_velocity_covariance[8] = 0.00092446;

			msg_standard_imu_data.linear_acceleration_covariance[0] = 1.1499e-6;
			msg_standard_imu_data.linear_acceleration_covariance[1] = 0.0;
			msg_standard_imu_data.linear_acceleration_covariance[2] = 0.0;
			msg_standard_imu_data.linear_acceleration_covariance[3] = 0.0;
			msg_standard_imu_data.linear_acceleration_covariance[4] = 1.0626e-6;
			msg_standard_imu_data.linear_acceleration_covariance[5] = 0.0;
			msg_standard_imu_data.linear_acceleration_covariance[6] = 0.0;
			msg_standard_imu_data.linear_acceleration_covariance[7] = 0.0;
			msg_standard_imu_data.linear_acceleration_covariance[8] = 9.4396e-6;
		}
	}

	// Publish
	if (context->publishers[0].getNumSubscribers() > 0)
	{
		msg_sensor_data.header.seq = seq;
		// msg_sensor_data.header.stamp = timestamp;
		context->publishers[0].publish(msg_sensor_data);
	}

	if (context->publishers[1].getNumSubscribers() > 0)
	{
		msg_ins_data.header.seq = seq;
		// msg_ins_data.header.stamp = timestamp;
		context->publishers[1].publish(msg_ins_data);
	}

	if (context->publishers[2].getNumSubscribers() > 0)
	{
		msg_gps_data.header.seq = seq;
		// msg_gps_data.header.stamp = timestamp;
		context->publishers[2].publish(msg_gps_data);
	}

	if (context->publishers[3].getNumSubscribers() > 0)
	{
		msg_gnss_data.header.seq = seq;
		// msg_gnss_data.header.stamp = timestamp;
		context->publishers[3].publish(msg_gnss_data);
	}

	if (context->publishers[4].getNumSubscribers() > 0)
	{
		msg_marine_data.header.seq = seq;
		// msg_marine_data.header.stamp = timestamp;
		context->publishers[4].publish(msg_marine_data);
	}

	if (context->publishers[5].getNumSubscribers() > 0)
	{
		msg_marine_data.header.seq = seq;
		// msg_marine_data.header.stamp = timestamp;
		context->publishers[5].publish(msg_standard_imu_data);
	}
}

void save_data(IL::INSDataStruct *data, void *contextPtr)
{
	Context *context = reinterpret_cast<Context *>(contextPtr);

	std::lock_guard<std::mutex> lock(data_mutex);

	ros::Time timestamp = ros::Time::now();

	if (context->publishers[0].getNumSubscribers() > 0 || context->publishers[5].getNumSubscribers() > 0)
	{
		g_msg_sensor_data.header.frame_id = context->imu_frame_id;
		g_msg_sensor_data.header.stamp = timestamp;
		g_msg_sensor_data.Mag.x = data->Mag[0];
		g_msg_sensor_data.Mag.y = data->Mag[1];
		g_msg_sensor_data.Mag.z = data->Mag[2];
		g_msg_sensor_data.Accel.x = data->Acc[0];
		g_msg_sensor_data.Accel.y = data->Acc[1];
		g_msg_sensor_data.Accel.z = data->Acc[2];
		g_msg_sensor_data.Gyro.x = data->Gyro[0];
		g_msg_sensor_data.Gyro.y = data->Gyro[1];
		g_msg_sensor_data.Gyro.z = data->Gyro[2];
		g_msg_sensor_data.Temp = data->Temp;
		g_msg_sensor_data.Vinp = data->VSup;
		g_msg_sensor_data.Pressure = data->hBar;
		g_msg_sensor_data.Barometric_Height = data->pBar;
	}

	if (context->publishers[1].getNumSubscribers() > 0 || context->publishers[5].getNumSubscribers() > 0)
	{
		g_msg_ins_data.header.frame_id = context->imu_frame_id;
		g_msg_ins_data.header.stamp = timestamp;
		g_msg_ins_data.YPR.x = data->Heading;
		g_msg_ins_data.YPR.y = data->Pitch;
		g_msg_ins_data.YPR.z = data->Roll;
		g_msg_ins_data.OriQuat.w = data->Quat[0];
		g_msg_ins_data.OriQuat.x = data->Quat[1];
		g_msg_ins_data.OriQuat.y = data->Quat[2];
		g_msg_ins_data.OriQuat.z = data->Quat[3];
		g_msg_ins_data.LLH.x = data->Latitude;
		g_msg_ins_data.LLH.y = data->Longitude;
		g_msg_ins_data.LLH.z = data->Altitude;
		g_msg_ins_data.Vel_ENU.x = data->VelENU[0];
		g_msg_ins_data.Vel_ENU.y = data->VelENU[1];
		g_msg_ins_data.Vel_ENU.z = data->VelENU[2];
		g_msg_ins_data.GPS_INS_Time = data->GPS_INS_Time;
		g_msg_ins_data.GPS_IMU_Time = data->GPS_IMU_Time;
		g_msg_ins_data.GPS_mSOW.data = data->ms_gps;
		g_msg_ins_data.Solution_Status.data = data->INSSolStatus;
		g_msg_ins_data.USW = data->USW;
	}

	if (context->publishers[2].getNumSubscribers() > 0)
	{
		g_msg_gps_data.header.frame_id = context->imu_frame_id;
		g_msg_gps_data.header.stamp = timestamp;
		g_msg_gps_data.LLH.x = data->LatGNSS;
		g_msg_gps_data.LLH.y = data->LonGNSS;
		g_msg_gps_data.LLH.z = data->AltGNSS;
		g_msg_gps_data.HorSpeed = data->V_Hor;
		g_msg_gps_data.SpeedDir = data->Trk_gnd;
		g_msg_gps_data.VerSpeed = data->V_ver;
	}

	if (context->publishers[3].getNumSubscribers() > 0)
	{
		g_msg_gnss_data.header.frame_id = context->imu_frame_id;
		g_msg_gnss_data.header.stamp = timestamp;
		g_msg_gnss_data.GNSS_info_1 = data->GNSSInfo1;
		g_msg_gnss_data.GNSS_info_2 = data->GNSSInfo2;
		g_msg_gnss_data.Number_Sat = data->SVsol;
		g_msg_gnss_data.GNSS_Velocity_Latency = data->GNSSVelLatency;
		g_msg_gnss_data.GNSS_Angles_Position_Type = data->AnglesType;
		g_msg_gnss_data.GNSS_Heading = data->Heading_GNSS;
		g_msg_gnss_data.GNSS_Pitch = data->Pitch_GNSS;
		g_msg_gnss_data.GNSS_GDOP = data->GDOP;
		g_msg_gnss_data.GNSS_PDOP = data->PDOP;
		g_msg_gnss_data.GNSS_HDOP = data->HDOP;
		g_msg_gnss_data.GNSS_VDOP = data->VDOP;
		g_msg_gnss_data.GNSS_TDOP = data->TDOP;
		g_msg_gnss_data.New_GNSS_Flags = data->NewGPS;
		g_msg_gnss_data.Diff_Age = data->DiffAge;
	}

	if (context->publishers[4].getNumSubscribers() > 0)
	{
		g_msg_marine_data.header.frame_id = context->imu_frame_id;
		g_msg_marine_data.header.stamp = timestamp;
		g_msg_marine_data.Heave = data->Heave;
		g_msg_marine_data.Surge = data->Surge;
		g_msg_marine_data.Sway = data->Sway;
		g_msg_marine_data.Heave_velocity = data->Heave_velocity;
		g_msg_marine_data.Surge_velocity = data->Surge_velocity;
		g_msg_marine_data.Sway_velocity = data->Sway_velocity;
		g_msg_marine_data.Significant_wave_height = data->significant_wave_height;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "il_ins");
	ros::NodeHandle n;
	ros::NodeHandle np("~");
	std::string port;
	IL::Driver ins;
	int ins_output_type;
	std::string imu_frame_id;
	Context context;

	std::string serial_port;
	int publish_rate, serial_baud;

	// command line varibales

	np.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
	np.param<int>("serial_baud", serial_baud, 2000000);
	np.param<int>("publish_rate", publish_rate, 100);

	ros::Rate rate(publish_rate); // 100 hz
	port = "serial:" + serial_port + ":" + std::to_string(serial_baud);

	// np.param<std::string>("ins_url", port, "serial:/dev/ttyUSB0:2000000");
	np.param<int>("output_type", ins_output_type, 0x95);

	// Initializing Publishers
	context.publishers[0] = np.advertise<inertiallabs_msgs::sensor_data>("/Inertial_Labs/sensor_data", 1);
	context.publishers[1] = np.advertise<inertiallabs_msgs::ins_data>("/Inertial_Labs/ins_data", 1);
	context.publishers[2] = np.advertise<inertiallabs_msgs::gps_data>("/Inertial_Labs/gps_data", 1);
	context.publishers[3] = np.advertise<inertiallabs_msgs::gnss_data>("/Inertial_Labs/gnss_data", 1);
	context.publishers[4] = np.advertise<inertiallabs_msgs::marine_data>("/Inertial_Labs/marine_data", 1);
	context.publishers[5] = np.advertise<sensor_msgs::Imu>("/Inertial_Labs/standard_imu_data", 1);

	ROS_INFO("connecting to INS at URL %s, ins_output_format %d, publish_rate %d", port.c_str(), ins_output_type, publish_rate);

	auto il_err = ins.connect(port.c_str());
	if (il_err != 0)
	{
		ROS_FATAL("Could not connect to the INS on this URL %s\n",
				  port.c_str());
		exit(EXIT_FAILURE);
	}

	if (ins.isStarted())
	{
		ins.stop();
	}
	auto devInfo = ins.getDeviceInfo();
	auto devParams = ins.getDeviceParams();
	std::string SN(reinterpret_cast<const char *>(devInfo.IDN), 8);
	ROS_INFO("Found INS S/N %s\n", SN.c_str());
	context.imu_frame_id = SN;
	il_err = ins.start(ins_output_type);
	if (il_err != 0)
	{
		ROS_FATAL("Could not start the INS: %i\n", il_err);
		ins.disconnect();
		exit(EXIT_FAILURE);
	}
	ins.setCallback(&save_data, &context);
	ROS_INFO("publishing at %d Hz\n", publish_rate);
	ROS_INFO("rostopic echo the topics to see the data");

	while (ros::ok())
	{
		publish_data(&context);
		ros::spinOnce();
		rate.sleep();
	}

	std::cout << "Stopping INS... " << std::flush;
	ins.stop();
	std::cout << "Disconnecting... " << std::flush;
	ins.disconnect();
	std::cout << "Done." << std::endl;
	return 0;
}
