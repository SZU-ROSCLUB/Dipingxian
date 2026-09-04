
#ifndef _ORIGINCAR_BASE_H_
#define _ORIGINCAR_BASE_H_

#include <memory>
#include <inttypes.h>
#include "rclcpp/rclcpp.hpp"
#include <csignal>
#include <thread>

#include <iostream>
#include <string.h>
#include <string>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <rcl/types.h>
#include <sys/stat.h>

#include <serial/serial.h>
#include <fcntl.h>
#include <stdbool.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "origincar_msg/msg/data.hpp"
// #include "origincar_msg/msg/sign.hpp"  // 匹配信号发送
#include <sensor_msgs/msg/imu.hpp>
using namespace std;


#define SEND_DATA_CHECK   1          //Send data check flag bits //发送数据校验标志位
#define READ_DATA_CHECK   0          //Receive data to check flag bits //接收数据校验标志位
#define FRAME_HEADER      0X7B       //Frame head //帧头
#define FRAME_TAIL        0X7D       //Frame tail //帧尾
#define RECEIVE_DATA_SIZE 24         //The length of the data sent by the lower computer //下位机发送过来的数据的长度
#define SEND_DATA_SIZE    11         //The length of data sent by ROS to the lower machine //ROS向下位机发送的数据的长度
#define PI 				  3.1415926f //PI //圆周率

#define GYROSCOPE_RATIO   0.00026644f

#define ACCEl_RATIO 	  1671.84f

extern sensor_msgs::msg::Imu Mpu6050;

typedef struct __Vel_Pos_Data_
{
	float X;
	float Y;
	float Z;

} Vel_Pos_Data;

typedef struct __MPU6050_DATA_
{
	short accele_x_data;
	short accele_y_data;
	short accele_z_data;
    short gyros_x_data;
	short gyros_y_data;
	short gyros_z_data;

} MPU6050_DATA;

typedef struct _SEND_DATA_
{
	uint8_t tx[SEND_DATA_SIZE];
	float X_speed;
	float Y_speed;
	float Z_speed;
	unsigned char Frame_Tail;
} SEND_DATA;

typedef struct _RECEIVE_DATA_
{
	uint8_t rx[RECEIVE_DATA_SIZE];
	uint8_t Flag_Stop;
	unsigned char Frame_Header;
	float X_speed;
	float Y_speed;
	float Z_speed;
	float Power_Voltage;
	unsigned char Frame_Tail;
} RECEIVE_DATA;

class origincar_base : public rclcpp::Node

{
public:
	origincar_base();
	~origincar_base();
	void Control();

public : 
	serial::Serial Stm32_Serial;

private:
	void Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux);

	void Publish_ImuSensor();
	void Publish_WheelSpeed();

	bool Get_Sensor_Data();
	unsigned char Check_Sum(unsigned char Count_Number,unsigned char mode);
	short IMU_Trans(uint8_t Data_High,uint8_t Data_Low);
	float WheelSpeed_Trans(uint8_t Data_High,uint8_t Data_Low);

	void Sign_Switch_Callback(const std_msgs::msg::Int32::SharedPtr sign_switch);

private:
	rclcpp::Time _Now, _Last_Time;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Cmd_Vel_Sub;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher; 
	rclcpp::Publisher<origincar_msg::msg::Data>::SharedPtr robotvel_publisher;

	string usart_port_name;
	std::string cmd_vel;
	int serial_baud_rate;
	RECEIVE_DATA Receive_Data;
	SEND_DATA Send_Data;

	Vel_Pos_Data Robot_Vel;
	MPU6050_DATA Mpu6050_Data;
	float Power_voltage;

	int32_t reset_odom_flag;  // 清空里程计的标志位  TUDO
};


#endif //_ORIGINCAR_BASE_H_
