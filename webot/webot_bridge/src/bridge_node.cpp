/*
 * webot-bridge.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: webot
 */

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/FluidPressure.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/impl/point_types.hpp>
#include <pcl_ros/point_cloud.h>
#include <vector>
#include <pcl_conversions/pcl_conversions.h>

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <glib.h>
// Standard includes
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#ifdef __linux
#include <sys/ioctl.h>
#endif
#include <sys/time.h>
#include <time.h>

#include "../include/common/mavlink.h"
#include "../include/bridge/bridge.h"


using std::string;
using namespace std;
struct timeval tv;

int baud = 115200;                 ///< The serial baud rate
geometry_msgs::Twist cmd_vel;
uint16_t * payload_pointer;
std::string port = "/dev/ttyUSB0";              ///< The serial port name, e.g. /dev/ttyUSB0
bool silent = false;              ///< Wether console output should be enabled
bool verbose = false;             ///< Enable verbose output
bool debug = false;               ///< Enable debug functions and output
bool pc2serial = true;			  ///< Enable PC to serial push mode (send more stuff from pc over serial)
int fd;
int16_t linear_x_speed;
int16_t angular_speed;
static float q1, q2, q3, q4;

mavlink_raw_imu_t imu_raw;
ros::Subscriber webot_sub;
ros::Publisher webot_pub;
ros::Publisher imu_pub;
ros::Publisher imu_raw_pub;
ros::Publisher bridge_odom_pub;
ros::Publisher pcl_pub;

std::string odom_frame_id; //add
std::string base_frame_id; //add
std::string imu_frame_id;  //add

sensor_msgs::Imu imu_msg;
nav_msgs::Odometry odom;
mavlink_message_t msg;
mavlink_manual_control_t manual_control_msg;
mavlink_command_long_t command_long_msg;
mavlink_attitude_quaternion_t att;
geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(3.14);
geometry_msgs::TransformStamped trans_odom;
geometry_msgs::TransformStamped trans_imu_link;
sensor_msgs::PointCloud2 barrier_pcl;


typedef pcl::PointCloud<pcl::PointXYZ> pclPointCloudXYZ;
typedef boost::shared_ptr<pclPointCloudXYZ> pclPointCloudXYZPtr;



sensor_msgs::PointCloud2Ptr  pROSCloud(new sensor_msgs::PointCloud2);


template<typename PointCloudType > void
    BuildPclPointCloud( boost::shared_ptr<pcl::PointCloud<PointCloudType> > pCloud)
    {
        PointCloudType target_pt;

        int j = 0;
       if(IR_DATA_LF > 0)
       {
        target_pt.x =  BARRIER_DISTANCE;
        target_pt.y =  BARRIER_DISTANCE;
        target_pt.z =  0;
        pCloud->points.push_back( target_pt );
       }
       if(IR_DATA_CF > 0)
       {
        target_pt.x =  BARRIER_DISTANCE;
        target_pt.y =  0;
        target_pt.z =  0.0;
        pCloud->points.push_back( target_pt );
       }
       if(IR_DATA_RF > 0)
       {
        target_pt.x =  BARRIER_DISTANCE;
        target_pt.y =  -BARRIER_DISTANCE;
        target_pt.z =  0;
        pCloud->points.push_back( target_pt );
       }
        pCloud->width = pCloud->points.size();
        pCloud->header.stamp = pcl_conversions::toPCL( ros::Time::now());
        pCloud->height = 1;
    }


int open_port(std::string& port)
{
  int fd; /* File descriptor for the port */

  // Open serial port
  // O_RDWR - Read and write
  // O_NOCTTY - Ignore special chars like CTRL-C
  fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1)
  {
    /* failure... */
    return (-1);
  }
  else
  {
    fcntl(fd, F_SETFL, 0);
  }
 return (fd);
}

bool setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
  struct termios config;
  if (!isatty(fd))
  {
    fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
    return false;
  }
  if (tcgetattr(fd, &config) < 0)
  {
    fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
    return false;
  }

  config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

  config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

#ifdef OLCUC
  config.c_oflag &= ~OLCUC;
#endif

#ifdef ONOEOT
  config.c_oflag &= ~ONOEOT;
#endif

  //
  // No line processing:
  // echo off, echo newline off, canonical mode off,
  // extended input processing off, signal chars off
  //
  config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
  //
  // Turn off character processing
  // clear current char size mask, no parity checking,
  // no output processing, force 8 bit input
  //
  config.c_cflag &= ~(CSIZE | PARENB);
  config.c_cflag |= CS8;
  //
  // One input byte is enough to return from read()
  // Inter-character timer off
  //
  config.c_cc[VMIN] = 1;
  config.c_cc[VTIME] = 10; // was 0
  switch (baud)
  {
    case 1200:
      if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
      {
        fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
        return false;
      }
      break;
    case 1800:
      cfsetispeed(&config, B1800);
      cfsetospeed(&config, B1800);
      break;
    case 9600:
      cfsetispeed(&config, B9600);
      cfsetospeed(&config, B9600);
      break;
    case 19200:
      cfsetispeed(&config, B19200);
      cfsetospeed(&config, B19200);
      break;
    case 38400:
      if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
      {
        fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
        return false;
      }
      break;
    case 57600:
      if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
      {
        fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
        return false;
      }
      break;
    case 115200:
      if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
      {
        fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
        return false;
      }
      break;

      // These two non-standard (by the 70'ties ) rates are fully supported on
      // current Debian and Mac OS versions (tested since 2010).
    case 460800:
      if (cfsetispeed(&config, 460800) < 0 || cfsetospeed(&config, 460800) < 0)
      {
        fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
        return false;
      }
      break;
    case 921600:
      if (cfsetispeed(&config, 921600) < 0 || cfsetospeed(&config, 921600) < 0)
      {
        fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
        return false;
      }
      break;
    default:
      fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
      return false;

      break;
  }

  if (tcsetattr(fd, TCSAFLUSH, &config) < 0)
  {
    fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
    return false;
  }
  return true;
}

void close_port(int fd)
{
  close(fd);
}

void* serial_wait(void* serial_ptr)
{
  int fd = *((int*)serial_ptr);
  mavlink_status_t lastStatus;
  static tf::TransformBroadcaster bridge_broadcaster_mobile;
  static tf::TransformBroadcaster bridge_odom_broadcaster_base_link;
  static tf::TransformBroadcaster bridge_odom_broadcaster_odom;
  static pclPointCloudXYZPtr pcl_cloud( new pclPointCloudXYZ );
  static geometry_msgs::Quaternion q_msg;
   lastStatus.packet_rx_drop_count = 0;
    while (1)
      {
      uint8_t cp;
      mavlink_message_t message;
      mavlink_status_t status;
      uint8_t msgReceived = false;

      //tcflush(fd, TCIFLUSH);

      if (read(fd, &cp, 1) > 0)
         {
           // Check if a message could be decoded, return the message in case yes
           msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
           if (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count)
           {
             if (verbose || debug)
               printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
             if (debug)
             {
               unsigned char v = cp;
               fprintf(stderr, "%02x ", v);
             }
           }
           lastStatus = status;
         }
         else
          {
            if (!silent)
              fprintf(stderr, "ERROR: Could not read from port %s\n", port.c_str());
          }

          // If a message could be decoded, handle it
          if (msgReceived)
          {
            // DEBUG output
            if (debug)
            {
              fprintf(stderr, "Forwarding SERIAL -> ROS: ");
              unsigned int i;
              uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
              unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);
              if (messageLength > MAVLINK_MAX_PACKET_LEN)
              {
                fprintf(stderr, "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
              }
              else
              {
                for (i = 0; i < messageLength; i++)
                {
                  unsigned char v = buffer[i];
                  fprintf(stderr, "%02x ", v);
                }
                fprintf(stderr, "\n");
              }
            }

            if (verbose || debug)
              ROS_INFO("Received message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid,
                       message.compid);

//            ROS_INFO("Received a message! #%d, #%d, #%d \n",IR_DATA_LF,IR_DATA_CF,IR_DATA_RF);

            switch (message.msgid)
                 {
                   case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
                   {
                	  
                	   imu_msg.header.stamp = ros::Time::now();
                	   imu_msg.header.seq = imu_raw.time_usec / 1000;
                	   imu_msg.header.frame_id = imu_frame_id;

                	  
                	        
                	        mavlink_msg_attitude_quaternion_decode(&message, &att);

                	        imu_msg.angular_velocity.x = att.rollspeed;
                	        imu_msg.angular_velocity.y = att.pitchspeed;
                	        imu_msg.angular_velocity.z = att.yawspeed;
                	        imu_msg.linear_acceleration.x = imu_raw.xacc;
                	        imu_msg.linear_acceleration.y = imu_raw.yacc;
                	        imu_msg.linear_acceleration.z = 1;//imu_raw.zacc;
                	        imu_msg.orientation.x = q1 = att.q1;
                	        imu_msg.orientation.y = q2 = att.q2;
                	        imu_msg.orientation.z = q3 = att.q3;
                	        imu_msg.orientation.w = q4 = att.q4;

                           
                           for (sensor_msgs::Imu::_angular_velocity_covariance_type::iterator it =
                           imu_msg.angular_velocity_covariance.begin(); it != imu_msg.angular_velocity_covariance.end(); ++it)
                           *it = 0;
                           for (sensor_msgs::Imu::_linear_acceleration_covariance_type::iterator it =
                           	   imu_msg.linear_acceleration_covariance.begin(); it != imu_msg.linear_acceleration_covariance.end();
                           	    ++it)
                           *it = 0;
                           imu_msg.orientation_covariance[0] = 0;
                           
                           imu_pub.publish(imu_msg);
                         
                           trans_imu_link.header.stamp = ros::Time::now();
                           trans_imu_link.header.frame_id = base_frame_id;
                           trans_imu_link.child_frame_id = imu_frame_id;
                           trans_imu_link.transform.translation.x = 0.0;
                           trans_imu_link.transform.translation.y = 0.0;
                           trans_imu_link.transform.translation.z = 0.0;


                           tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0,0,0), q_msg);
                           trans_imu_link.transform.rotation  = q_msg;
                           //send the transform
                           bridge_odom_broadcaster_base_link.sendTransform(trans_imu_link);
                           /*
                           trans_base_link.header.stamp = ros::Time::now();
                           trans_base_link.header.frame_id = "webot_tansf_base";
                           trans_base_link.child_frame_id = "webot_base_link";
                           trans_base_link.transform.translation.x = 0.0;
                           trans_base_link.transform.translation.y = 0.0;
                           trans_base_link.transform.translation.z = 0.0;
                           trans_base_link.transform.rotation  = imu_msg->orientation;
                           //send the transform
                           bridge_odom_broadcaster_base_link.sendTransform(trans_base_link);
                           */ 
                           
                	   break;
                   }

                   case MAVLINK_MSG_ID_COMMAND_LONG:
                   {
                       mavlink_msg_command_long_decode(&message, &command_long_msg);
                	   
                       if(command_long_msg.command == ODOM_DATA)
                       {
                	   //next, we'll publish the odometry message over ROS
                           
                	   
                           odom.header.stamp = ros::Time::now();
                           odom.header.frame_id = odom_frame_id;
                           odom.child_frame_id = base_frame_id;
                           

                           odom.twist.twist.linear.x =  LINEAR_X_SPEED;    //x speed in m/s
                           odom.twist.twist.angular.z = ANGULAR_Z_SPEED;  // angular speed in rad/s
                           //set the position

                           odom.pose.pose.position.x = Pos_X;//add
                           odom.pose.pose.position.y = Pos_Y;//add
                           odom.pose.pose.position.z = 0.0;//add
                           odom.pose.pose.orientation.x = q1;
                           odom.pose.pose.orientation.y = q2;
                           odom.pose.pose.orientation.z = q3;
                           odom.pose.pose.orientation.w = q4;
                          //set the velocityatt.yawspeed

                          //publish odom message
                           bridge_odom_pub.publish(odom);

                           trans_odom.header.stamp = ros::Time::now();
                	   trans_odom.header.frame_id = odom_frame_id;
                	   trans_odom.child_frame_id = base_frame_id;
                	   trans_odom.transform.translation.x = Pos_X;//add
                	   trans_odom.transform.translation.y = Pos_Y;//add
                	   trans_odom.transform.translation.z = 0.0;//add
                	   trans_odom.transform.rotation = odom.pose.pose.orientation;

				       //send the transform
                	   bridge_odom_broadcaster_odom.sendTransform(trans_odom);

                       }

                      if(command_long_msg.command == IR_DATA)
                      {
                    	 // publish the IR barrier_pcl message over ROS
                    	  if(IR_DATA_LF > 0 || IR_DATA_CF > 0 || IR_DATA_RF > 0)
                    	  {
                    	      BuildPclPointCloud( pcl_cloud );
                    	      pcl::toROSMsg( *pcl_cloud, barrier_pcl );
                    	      barrier_pcl.header.frame_id = "webot_base_link";
                    	      barrier_pcl.header.stamp = ros::Time::now();
                    	      pcl_pub.publish(barrier_pcl);
                    	  }

                      }
                       break;
                   }
                  default:break;
                 }
               }
      }
  return 0;
}

void callback(const geometry_msgs::Twist& cmd_vel)
{

/*
    ROS_INFO("Received a /cmd_vel message!");
    ROS_INFO("Linear Components:[%f,%f,%f]",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.linear.z);
    ROS_INFO("Angular Components:[%f,%f,%f]",cmd_vel.angular.x,cmd_vel.angular.y,cmd_vel.angular.z);
*/
    linear_x_speed = (int16_t)(1000*cmd_vel.linear.x);
    angular_speed = (int16_t)(1000*cmd_vel.angular.z);
    payload_pointer = (uint16_t *)msg.payload64;
    *payload_pointer     = linear_x_speed ;
    *(payload_pointer+3) = angular_speed;
    static uint8_t mavlink_crcs[] = MAVLINK_MESSAGE_CRCS;
    mavlink_finalize_message_chan(&msg, msg.sysid, msg.compid, MAVLINK_COMM_0, msg.len, mavlink_crcs[msg.msgid]);
    static uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int messageLength = mavlink_msg_to_send_buffer(buffer, &msg);
    int written = write(fd, (char*)buffer, messageLength);
    tcflush(fd, TCOFLUSH);

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "webot_ros_serial");

  static GOptionEntry entries[] = { {"portname",'p', 0, G_OPTION_ARG_STRING, &port, "Serial port name", port.c_str()},
                                   {"baudrate", 'b', 0, G_OPTION_ARG_INT,    &baud, "Baudrate", "115200"}, {
                                     "verbose", 'v', 0, G_OPTION_ARG_NONE,   &verbose, "Be verbose", NULL},
                                   {   "debug", 'd', 0, G_OPTION_ARG_NONE,   &debug, "Debug output", NULL},
								   {NULL, 0, 0, G_OPTION_ARG_NONE, NULL, NULL, 0},
								   {NULL, 0, 0, G_OPTION_ARG_NONE, NULL, NULL, 0},
                                   {NULL, 0, 0, G_OPTION_ARG_NONE, NULL, NULL, 0}};
  GError *error = NULL;
  GOptionContext *context;
  context = g_option_context_new("- translate MAVLink messages between ROS to serial port");
  g_option_context_add_main_entries(context, entries, NULL);
  if (!g_option_context_parse(context, &argc, &argv, &error))
    {
      g_print("Option parsing failed: %s\n", error->message);
      exit(1);
    }

// SETUP SERIAL PORT
  if (!silent)
     printf("Trying to connect to %s...", port.c_str());
  fd = open_port(port);
  if (fd == -1)
      {
         if (!silent)
        	printf("\n Open port failure....\n");// fprintf(stderr, "failure, could not open port.\n");
         exit(EXIT_FAILURE);
      }
      else
      {
        if (!silent)
          printf("OK!\n");
      }
      if (!silent)
        printf("Trying to configure %s... ", port.c_str());
      bool setup = setup_port(fd, baud, 8, 1, false, false);
      if (!setup)
      {
        if (!silent)
          fprintf(stderr, "failure, could not configure port.\n");
        exit(EXIT_FAILURE);
      }
      else
      {
        if (!silent)
          printf("Done.\n");
      }
      int* fd_ptr = &fd;
      msg.sysid = 1;
      msg.msgid = 69;
      msg.compid = 0;
      msg.len    = 11;
      // SETUP ROS

      ros::NodeHandle nh;


      nh.param("odom_frame_id", odom_frame_id, std::string("odom"));
      nh.param("base_frame_id", base_frame_id, std::string("base_link"));
      nh.param("imu_frame_id", imu_frame_id, std::string("imu_link"));

      pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("barrier_pcl", 1);
      imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 10);
      bridge_odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
      ros::Subscriber sub = nh.subscribe("/cmd_vel", 1000, callback);
      GThread* serial_thread;
      GError* err;
      if (!g_thread_supported())
         {
           g_thread_init(NULL);
            // Only initialize g thread if not already done
         }
      if (!silent)
          printf("\nREADY, waiting for serial/ROS data.\n");
      if ((serial_thread = g_thread_create((GThreadFunc)serial_wait, (void *)fd_ptr, TRUE, &err)) == NULL)
          {
             printf("Failed to create serial handling thread: %s!!\n", err->message);
             g_error_free(err);
          }
      int noErrors = 0;
      if (fd == -1 || fd == 0)
          {
             if (!silent)
                fprintf(stderr, "Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n", port.c_str(), baud);
             exit(EXIT_FAILURE);
          }
          else
          {

              if (!silent)
              {
            	  printf("\n   ********************************************************\n");
            	  printf("   *                                                      *\n");
            	  printf("   *                WEBOT's BRIDGE IS RUNNING...          *\n");
            	  printf("   *                                                      *\n");
            	  printf("   *   https://github.com/ROS-Robotics/Webot-ROS-bridge   *\n");
            	  printf("   *                                                      *\n");
            	  printf("   ********************************************************\n");
                fprintf(stderr, "\nConnected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", port.c_str(), baud);
              }
          }
      if (fd == -1 || fd == 0)
         {
            exit(noErrors);
         }
         // Ready to go .

      ros::spin();
      close_port(fd);
      return 0;
}


