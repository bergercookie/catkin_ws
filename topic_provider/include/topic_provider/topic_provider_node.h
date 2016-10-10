/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef TOPIC_PROVIDER_NODE_H
#define TOPIC_PROVIDER_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <mrpt_msgs/Pose2DStamped.h>
#include <nav_msgs/Odometry.h>

#include <mrpt_bridge/mrpt_bridge.h>
#include <mrpt/utils/COutputLogger.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/system/threads.h>
#include <mrpt/utils/CMessage.h>
#include <mrpt/utils/CServerTCPSocket.h>
#include <mrpt/utils/CClientTCPSocket.h>

#include <iostream>
#include <string>
#include <sstream>

// supplementary functions - TODO - where to put these?
template<class T>
std::string getVectorAsString(const T& t) {
	using namespace std;
	stringstream ss;
	for (typename T::const_iterator it = t.begin(); it != t.end(); ++it) {
		ss << *it << ", ";
	}
	return ss.str();
}
template<class T>
void printVector(const T& t) {
	std::cout << getVectorAsString(t) << std::endl;
}
template<class T>
void printVectorOfVectors(const T& t) {
	int i = 0;
	for (typename T::const_iterator it = t.begin(); it  != t.end(); ++i, ++it) {
		printf("Vector %d/%lu:\n\t", i, t.size());
		printVector(*it);
	}
}

//
// CTopicSniffer class
//
class CTopicSniffer : public mrpt::utils::COutputLogger
{
public:
	typedef CTopicSniffer sniffer_t;

	CTopicSniffer(ros::NodeHandle* nh);
	~CTopicSniffer();

	/**\brief Callback method for handling incoming LaserScans objects in a ROS
	 * topic.
	 *
	 */
	void sniffLaserScan(const sensor_msgs::LaserScan::ConstPtr& ros_laser_scan);
	/**\brief Callback method for handling incoming odometry measurements in a ROS
	 * topic.
	 *
	 */
	void sniffOdom(const mrpt_msgs::Pose2DStamped::ConstPtr& ros_odom);
	/**\brief Establish a TCP socket connection with the first incoming (most
	 * probably an MRPT) node.
	 *
	 */
	void initServer();

private:
	std::string m_class_name;

	ros::NodeHandle* nh;

	/** Top namespace in which we fetch the parameters for the graphslam-engine
	 */
	std::string top_param_ns;

	/** server instance - to be initialized in the constructor */
	mrpt::utils::CServerTCPSocket* m_server;
	/**\brief Socket to send the topic measurements to
	 *
	 * Whenever we have enough data to compose a CObservation object or a
	 * (CActionCollection + CSensoryFrame) we send the aforementioned to the
	 * client in a MRPT serialized form.
	 *
	 * \note Currently supports only one (MRPT) client socket.
	 */
	mrpt::utils::CClientTCPSocket* m_client;

	std::map<std::string, int> msg_types;
	std::string server_addr;
	int server_port_no;


};

#endif /* end of include guard: TOPIC_PROVIDER_NODE_H */
