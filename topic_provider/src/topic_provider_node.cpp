/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

#include <mrpt_bridge/mrpt_bridge.h>
#include <mrpt/utils/COutputLogger.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/system/threads.h>

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



// TODO - split the class - header & implementation
//
// TopicSniffer class
//
class TopicSniffer : mrpt::utils::COutputLogger
{
public:
	TopicSniffer();
	~TopicSniffer();

	void sniffLaserScan(const sensor_msgs::LaserScan::ConstPtr& ros_laser_scan);
	void sniffOdom();

private:
	/* data */
};

// Ctors, dtors
TopicSniffer::TopicSniffer() {
	this->setLoggerName("TopicSniffer");
	this->setMinLoggingLevel(mrpt::utils::LVL_DEBUG);
}
TopicSniffer::~TopicSniffer() { }

// Method Implementations
void TopicSniffer::sniffLaserScan(const sensor_msgs::LaserScan::ConstPtr& ros_laser_scan) {
	using namespace std;
	using namespace mrpt::utils;
	using namespace mrpt::obs;

	MRPT_LOG_DEBUG_STREAM << "sniffLaserScan: Received a LaserScan msg. Converting it to MRPT format...";

	//cout << "Laser Scan: " << *ros_laser_scan << endl;
	// build the CObservation2DRangeScan
	CObservation2DRangeScanPtr mrpt_laser_scan = CObservation2DRangeScan::Create();
	mrpt::poses::CPose3D rel_pose;
	mrpt_bridge::convert(*ros_laser_scan, rel_pose, *mrpt_laser_scan);

	printVector(mrpt_laser_scan->scan);

	
}
void TopicSniffer::sniffOdom() {
	using namespace std;
	using namespace mrpt::utils;

	cout << "In sniffOdom method" << endl;
	MRPT_LOG_DEBUG_STREAM << "sniffOdom: Received an odometry msg. Converting it to MRPT format...";
}

//
// Main
//
int main(int argc, char **argv)
{
	using namespace std;
	using namespace mrpt::poses;

	std::string node_name="topic_provider";

	// ros-related initialization actions
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);

	// subscribe to the topics that we want to read
	TopicSniffer sniffer;

	// Odometry
	//ros::Subscriber odom_sub = nh.subscribe("odom/", 1000, &TopicSniffer::sniffOdom, &sniffer);
	// LaserScans
	std::string base_scan(""); nh.getParam("/base_scan", base_scan);
	ros::Subscriber laserscans_sub = nh.subscribe<sensor_msgs::LaserScan>(
			base_scan,
			1000,
			&TopicSniffer::sniffLaserScan, &sniffer);

	//CPose2D p(15, 0, 0.2);

	while (ros::ok()) {
		//cout << p << endl;

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

