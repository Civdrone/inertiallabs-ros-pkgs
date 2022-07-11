#include <iostream>
#include <mutex>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

// Inertial Labs source header
#include "ILDriver.h"

//adding message type headers
#include <inertiallabs_msgs/msg/sensor_data.hpp>
#include <inertiallabs_msgs/msg/ins_data.hpp>
#include <inertiallabs_msgs/msg/gps_data.hpp>
#include <inertiallabs_msgs/msg/gnss_data.hpp>
#include <inertiallabs_msgs/msg/marine_data.hpp>
#include <connectivity_client2/connectivity_client2.hpp>
#include <civros_utils/parameters.hpp>

#define G 9.80665
// Publishers

struct Context 
{
	std::shared_ptr<rclcpp::Node> node_;
	rclcpp::Publisher<inertiallabs_msgs::msg::SensorData>::SharedPtr _publisher_0;
	rclcpp::Publisher<inertiallabs_msgs::msg::INSData>::SharedPtr _publisher_1;
	rclcpp::Publisher<inertiallabs_msgs::msg::GPSData>::SharedPtr _publisher_2;
	rclcpp::Publisher<inertiallabs_msgs::msg::GNSSData>::SharedPtr _publisher_3;
	rclcpp::Publisher<inertiallabs_msgs::msg::MarineData>::SharedPtr _publisher_4;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _publisher_5;

	std::string imu_frame_id;
};

std::mutex data_mutex;
inertiallabs_msgs::msg::SensorData g_msg_sensor_data;
inertiallabs_msgs::msg::INSData g_msg_ins_data;
inertiallabs_msgs::msg::GPSData g_msg_gps_data;
inertiallabs_msgs::msg::GNSSData g_msg_gnss_data;
inertiallabs_msgs::msg::MarineData g_msg_marine_data;

void publish_data(Context *context)
{
	rclcpp::Time timestamp = context->node_->now();

	inertiallabs_msgs::msg::SensorData msg_sensor_data;
	inertiallabs_msgs::msg::INSData msg_ins_data;
	inertiallabs_msgs::msg::GPSData msg_gps_data;
	inertiallabs_msgs::msg::GNSSData msg_gnss_data;
	inertiallabs_msgs::msg::MarineData msg_marine_data;
	sensor_msgs::msg::Imu msg_standard_imu_data;

	// Lock, copy and unlock
	{
		std::lock_guard<std::mutex> lock(data_mutex);
		if (context->_publisher_0->get_subscription_count() > 0)
		{
			msg_sensor_data = g_msg_sensor_data;
		}

		if (context->_publisher_1->get_subscription_count() > 0)
		{
			msg_ins_data = g_msg_ins_data;
		}

		if (context->_publisher_2->get_subscription_count() > 0)
		{
			msg_gps_data = g_msg_gps_data;
		}

		if (context->_publisher_3->get_subscription_count() > 0)
		{
			msg_gnss_data = g_msg_gnss_data;
		}

		if (context->_publisher_4->get_subscription_count() > 0)
		{
			msg_marine_data = g_msg_marine_data;
		}

		if (context->_publisher_5->get_subscription_count() > 0)
		{
			msg_standard_imu_data.header = g_msg_ins_data.header;
			msg_standard_imu_data.header.frame_id = "base_link";
			msg_standard_imu_data.orientation = g_msg_ins_data.ori_quat;
			msg_standard_imu_data.angular_velocity = g_msg_sensor_data.gyro;
			msg_standard_imu_data.linear_acceleration.x = g_msg_sensor_data.accel.x * G;
			msg_standard_imu_data.linear_acceleration.y = g_msg_sensor_data.accel.y * G;
			msg_standard_imu_data.linear_acceleration.z = g_msg_sensor_data.accel.z * G;

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
	if (context->_publisher_0->get_subscription_count() > 0)
	{
		msg_sensor_data.header.stamp = timestamp;
		context->_publisher_0->publish(msg_sensor_data);
	}

	if (context->_publisher_1->get_subscription_count() > 0)
	{
		msg_ins_data.header.stamp = timestamp;
		context->_publisher_1->publish(msg_ins_data);
	}

	if (context->_publisher_2->get_subscription_count() > 0)
	{
		msg_gps_data.header.stamp = timestamp;
		context->_publisher_2->publish(msg_gps_data);
	}

	if (context->_publisher_3->get_subscription_count() > 0)
	{
		msg_gnss_data.header.stamp = timestamp;
		context->_publisher_3->publish(msg_gnss_data);
	}

	if (context->_publisher_4->get_subscription_count() > 0)
	{
		msg_marine_data.header.stamp = timestamp;
		context->_publisher_4->publish(msg_marine_data);
	}

	if (context->_publisher_5->get_subscription_count() > 0)
	{
		msg_standard_imu_data.header.stamp = timestamp;
		// msg_marine_data.header.stamp = timestamp;
		context->_publisher_5->publish(msg_standard_imu_data);
	}
}

void save_data(IL::INSDataStruct *data, void *contextPtr)
{
	Context *context = reinterpret_cast<Context *>(contextPtr);

	std::lock_guard<std::mutex> lock(data_mutex);
	
	if (context->_publisher_0->get_subscription_count() > 0)
	{
		g_msg_sensor_data.header.frame_id = context->imu_frame_id;
		g_msg_sensor_data.mag.x = data->Mag[0];
		g_msg_sensor_data.mag.y = data->Mag[0];
		g_msg_sensor_data.mag.z = data->Mag[0];
		g_msg_sensor_data.accel.x = data->Acc[0];
		g_msg_sensor_data.accel.y = data->Acc[1];
		g_msg_sensor_data.accel.z = data->Acc[2];
		g_msg_sensor_data.gyro.x = data->Gyro[0];
		g_msg_sensor_data.gyro.y = data->Gyro[1];
		g_msg_sensor_data.gyro.z = data->Gyro[2];
		g_msg_sensor_data.temp = data->Temp;
		g_msg_sensor_data.vinp = data->VSup;
		g_msg_sensor_data.pressure = data->hBar;
		g_msg_sensor_data.barometric_height = data->pBar;
	}

	if (context->_publisher_1->get_subscription_count() > 0)
	{
		g_msg_ins_data.header.frame_id = context->imu_frame_id;
		g_msg_ins_data.ypr.x = data->Heading;
		g_msg_ins_data.ypr.y = data->Pitch;
		g_msg_ins_data.ypr.z = data->Roll;
		g_msg_ins_data.ori_quat.w = data->Quat[0];
		g_msg_ins_data.ori_quat.x = data->Quat[1];
		g_msg_ins_data.ori_quat.y = data->Quat[2];
		g_msg_ins_data.ori_quat.z = data->Quat[3];
		g_msg_ins_data.llh.x = data->Latitude;
		g_msg_ins_data.llh.y = data->Longitude;
		g_msg_ins_data.llh.z = data->Altitude;
		g_msg_ins_data.vel_enu.x = data->VelENU[0];
		g_msg_ins_data.vel_enu.y = data->VelENU[1];
		g_msg_ins_data.vel_enu.z = data->VelENU[2];
		g_msg_ins_data.gps_ins_time = data->GPS_INS_Time;
		g_msg_ins_data.gps_imu_time = data->GPS_IMU_Time;
		g_msg_ins_data.gps_m_sow.data = data->ms_gps;
		g_msg_ins_data.solution_status.data = data->INSSolStatus;
        g_msg_ins_data.usw = data->USW;
	}

	if (context->_publisher_2->get_subscription_count() > 0)
	{
		g_msg_gps_data.header.frame_id = context->imu_frame_id;
		g_msg_gps_data.llh.x = data->LatGNSS;
		g_msg_gps_data.llh.y = data->LonGNSS;
		g_msg_gps_data.llh.z = data->AltGNSS;
		g_msg_gps_data.hor_speed = data->V_Hor;
		g_msg_gps_data.speed_dir = data->Trk_gnd;
		g_msg_gps_data.ver_speed = data->V_ver;
	}

	if (context->_publisher_3->get_subscription_count() > 0)
	{
		g_msg_gnss_data.header.frame_id = context->imu_frame_id;
		g_msg_gnss_data.gnss_info_1 = data->GNSSInfo1;
		g_msg_gnss_data.gnss_info_2 = data->GNSSInfo2;
		g_msg_gnss_data.number_sat = data->SVsol;
		g_msg_gnss_data.gnss_velocity_latency = data->GNSSVelLatency;
		g_msg_gnss_data.gnss_angles_position_type = data->AnglesType;
		g_msg_gnss_data.gnss_heading = data->Heading_GNSS;
		g_msg_gnss_data.gnss_pitch = data->Pitch_GNSS;
		g_msg_gnss_data.gnss_gdop = data->GDOP;
		g_msg_gnss_data.gnss_pdop = data->PDOP;
		g_msg_gnss_data.gnss_hdop = data->HDOP;
		g_msg_gnss_data.gnss_vdop = data->VDOP;
		g_msg_gnss_data.gnss_tdop = data->TDOP;
		g_msg_gnss_data.new_gnss_flags = data->NewGPS;
		g_msg_gnss_data.diff_age = data->DiffAge;
	}

	if (context->_publisher_4->get_subscription_count() > 0)
	{
		g_msg_marine_data.header.frame_id = context->imu_frame_id;
		g_msg_marine_data.heave = data->Heave;
		g_msg_marine_data.surge = data->Surge;
		g_msg_marine_data.sway = data->Sway;
		g_msg_marine_data.heave_velocity = data->Heave_velocity;
		g_msg_marine_data.surge_velocity = data->Surge_velocity;
		g_msg_marine_data.sway_velocity = data->Sway_velocity;
		g_msg_marine_data.significant_wave_height = data->significant_wave_height;
	}
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("il_ins");
    civros::connectivity::ConnectivityClient connectivity_client(node);
    connectivity_client.InitializeConnectivity();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

	std::string port;
	IL::Driver ins;
	int ins_output_type;
	std::string imu_frame_id;
	Context context;

	std::string serial_port;
	int publish_rate, serial_baud;

	// command line varibales

	//command line varibales
	using namespace civros::civros_utils;
	serial_port = Parameters::declare_param(node, "serial_port", std::string("/dev/ttyUSB0"));
	serial_baud = Parameters::declare_param(node, "serial_baud", 115200);
	publish_rate = Parameters::declare_param(node, "publish_rate", 100);
	
	rclcpp::Rate rate(publish_rate); // 100 hz
	port = "serial:" + serial_port + ":" + std::to_string(serial_baud);

	// np.param<std::string>("ins_url", port, "serial:/dev/ttyUSB0:2000000");
	ins_output_type = Parameters::declare_param(node, "output_type", 0x95);

	//Initializing Publishers
	context.node_ = node;

    context._publisher_0 = node->create_publisher<inertiallabs_msgs::msg::SensorData>("/Inertial_Labs/sensor_data", 1);
	context._publisher_1 = node->create_publisher<inertiallabs_msgs::msg::INSData>("/Inertial_Labs/ins_data", 1);
	context._publisher_2 = node->create_publisher<inertiallabs_msgs::msg::GPSData>("/Inertial_Labs/gps_data", 1);
	context._publisher_3 = node->create_publisher<inertiallabs_msgs::msg::GNSSData>("/Inertial_Labs/gnss_data", 1);
	context._publisher_4 = node->create_publisher<inertiallabs_msgs::msg::MarineData>("/Inertial_Labs/marine_data", 1);
	context._publisher_5 = node->create_publisher<sensor_msgs::msg::Imu>("/Inertial_Labs/standard_imu_data", 1);


	RCLCPP_INFO(node->get_logger(), "connecting to INS at URL %s, ins_output_format %d, publish_rate %d", port.c_str(), ins_output_type, publish_rate);

	diagnostic_msgs::msg::DiagnosticStatus diagnostic_message;
	while (ins.connect(port.c_str()) != 0)
	{
		diagnostic_message.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
		diagnostic_message.message = "disconnected";
		connectivity_client.SetMessage(diagnostic_message);
		int wait_milli(1000);
		RCLCPP_ERROR(node->get_logger(), "Could not connect to the INS on this URL %s. Retry in %d(Sec).\n", port.c_str(), int(wait_milli/1000));
		std::this_thread::sleep_for(std::chrono::milliseconds(wait_milli));
	}
	diagnostic_message.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
	diagnostic_message.message = "connected";
	connectivity_client.SetMessage(diagnostic_message);

	if (ins.isStarted())
	{
		ins.stop();
	}
	auto devInfo = ins.getDeviceInfo();
	auto devParams = ins.getDeviceParams();
	std::string SN(reinterpret_cast<const char *>(devInfo.IDN), 8);
	RCLCPP_INFO(node->get_logger(), "Found INS S/N %s\n", SN.c_str());
	context.imu_frame_id = SN;
	auto il_err = ins.start(ins_output_type);
	if (il_err != 0)
	{
		RCLCPP_FATAL(node->get_logger(), "Could not start the INS: %i\n", il_err);
		ins.disconnect();
		exit(EXIT_FAILURE);
	}
	ins.setCallback(&save_data, &context);
	RCLCPP_INFO(node->get_logger(), "publishing at %d Hz\n", publish_rate);
	RCLCPP_INFO(node->get_logger(), "rostopic echo the topics to see the data");
	
	while(rclcpp::ok())
	{
		publish_data(&context);
	    executor.spin_some();
		rate.sleep();
	}

	std::cout << "Stopping INS... " << std::flush;
	ins.stop();
	std::cout << "Disconnecting... " << std::flush;
	ins.disconnect();
	std::cout << "Done." << std::endl;

    rclcpp::shutdown();
	return 0;
}
