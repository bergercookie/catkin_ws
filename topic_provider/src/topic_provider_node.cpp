/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

//#include "../include/topic_provider/topic_provider_node.h"
#include <topic_provider/topic_provider_node.h>

/**\brief Persistant checking and fetching of a ROS parameter of type
 * std::string.
 *
 * Method tries to fetch the parameter given a certain time threshold
 * \return True if the fetching operation is successful
 */
template<class T>
bool getParamWithTimeout(
		const ros::NodeHandle& nh,
		const std::string& param_name,
		T* param_content,
		size_t timeout_ms=10000 /* ms */) {
	MRPT_START;
	using namespace std;

	bool found = nh.hasParam(param_name);
	int tries_thresh = 10; // how many times to try and fetch the parameter
	int timeout_ms_per = static_cast<int>(timeout_ms / tries_thresh);

	int current_try = 1;
	while ((!found) && (tries_thresh >= current_try++)) {
		mrpt::system::sleep(timeout_ms_per);
		found = nh.hasParam(param_name);
	}
	if (found) {
		nh.getParam(param_name, *param_content);
	}

	return found;

	MRPT_END;
}
/**\brief Wrapper around the getParamWithTimeout function that provides
 * textual feedback on the outcome of the getParam operation
 *
 * \return True if the fetching operation is successful
 */
template<class T>
bool getParamWithTimeout(
		const ros::NodeHandle& nh,
		const std::string& param_name,
		T* param_content,
		mrpt::utils::COutputLogger* logger,
		size_t timeout_ms=10000 /* ms */) {
	MRPT_START;
	using namespace mrpt::utils;

	ASSERT_(logger);

	logger->logFmt(
			LVL_INFO,
			"Fetching parameter \"%s\" from the ROS parameter server... timeout = %lu ms",
			param_name.c_str(),
			timeout_ms);

	// actual call to the getParamWithTimeout method
	bool found = getParamWithTimeout(nh, param_name, param_content, timeout_ms);

	if (found) {
		std::stringstream msg("");
		msg << "Successfully fetched parameter: " << param_name << " ==> "
			<< *param_content;
		logger->logStr(LVL_INFO, msg.str().c_str());
	}
	else {
		logger->logFmt(
				LVL_ERROR,
				"Failed to fetch parameter: \"%s\"",
				param_name.c_str());
	}
	return found;

	MRPT_END;
}

//
// CTopicSniffer class - Method Implementations
//
// Ctors, dtors
CTopicSniffer::CTopicSniffer(ros::NodeHandle* nh) {
	m_class_name = "CTopicSniffer";
	this->setLoggerName(m_class_name);

	// TODO - find a more robust solution - read it off the .ini file?
	this->setMinLoggingLevel(mrpt::utils::LVL_DEBUG);

	// Variables initialization
	m_client = NULL; // for safe deletion in the end

	// keep a pointer to the ROS node handler
	this->nh = nh;

	top_param_ns = "/graphslam_engine/";
	
}
CTopicSniffer::~CTopicSniffer() {
	MRPT_LOG_DEBUG_STREAM << "Releasing the client object..";
	delete m_client;
}

//
// Method Implementations
//
void CTopicSniffer::sniffLaserScan(const sensor_msgs::LaserScan::ConstPtr& ros_laser_scan) {
	using namespace std;
	using namespace mrpt::utils;
	using namespace mrpt::obs;

	MRPT_LOG_DEBUG_STREAM << "sniffLaserScan: Received a LaserScan msg. Converting it to MRPT format...";

	// build the CObservation2DRangeScan
	CObservation2DRangeScanPtr mrpt_laser_scan = CObservation2DRangeScan::Create();
	mrpt::poses::CPose3D rel_pose;
	mrpt_bridge::convert(*ros_laser_scan, rel_pose, *mrpt_laser_scan);

	// assemble and send the message
	ASSERTMSG_(m_client,
			"\nsniffLaserScan: Communication with server isn't established yet.\nExiting...\n");

	MRPT_LOG_DEBUG_STREAM << "sniffLaserScan: Sending LaserScan to MRPT node...";
	CMessage msg;
	msg.type = msg_types["FORMAT_2"];
	msg.serializeObject(mrpt_laser_scan.pointer());
	m_client->sendMessage(msg);

}

void CTopicSniffer::sniffOdom(const arduino_mr::Pose2DStamped::ConstPtr& ros_odom) {
	using namespace std;
	using namespace mrpt::utils;
	using namespace mrpt::obs;
	using namespace mrpt::poses;

	MRPT_LOG_DEBUG_STREAM << "sniffOdom: Received an odometry msg. Converting it to MRPT format...";
	MRPT_LOG_DEBUG_STREAM << *ros_odom << endl;

	// build and fill an CObservationOdometry instance

	CObservationOdometryPtr mrpt_odom = CObservationOdometry::Create();
	mrpt_bridge::convert(
			/* src = */ ros_odom->header.stamp,
			/* dst = */mrpt_odom->timestamp);
	mrpt_odom->hasEncodersInfo = false;
	mrpt_odom->hasVelocities = false;

	mrpt_odom->odometry.x(ros_odom->pose.x);
	mrpt_odom->odometry.y(ros_odom->pose.y);
	mrpt_odom->odometry.phi(ros_odom->pose.theta);

	MRPT_LOG_DEBUG_STREAM << "Odometry - MRPT format:\t" << mrpt_odom->odometry << endl;

	// serialize and send it to the TCP stream
	MRPT_LOG_DEBUG_STREAM << "sniffOdom: Sending odometry measurement to MRPT node...";
	CMessage msg;
	msg.type = msg_types["FORMAT_2"];
	msg.serializeObject(mrpt_odom.pointer());
	m_client->sendMessage(msg);

}

void CTopicSniffer::initServer() {
	using namespace std;
	using namespace mrpt::utils;

	// fetch the server parameters from the ROS parameter server
	ASSERT_(nh);

	MRPT_LOG_DEBUG_STREAM << "Fetching the server configuration from the ROS parameter server..." << endl;
	bool found_server = getParamWithTimeout(
			*nh,
			top_param_ns + "tcp/server_addr",
			&server_addr);
	bool found_port = getParamWithTimeout(
			*nh,
			top_param_ns + "tcp/server_port_no",
			&server_port_no);
	ASSERTMSG_(found_server,
			"TCP Server configuration was not found in the parameter server")
	ASSERTMSG_(found_port,
			"TCP Port configuration was not found in the parameter server")

	// fetch the message codes for the data tranmission from the ROS parameter
	// server
	bool found_format_1_code = getParamWithTimeout(
			*nh,
			top_param_ns + "msg_types/format_1",
			&msg_types["FORMAT_1"]);
	bool found_format_2_code = getParamWithTimeout(
			*nh,
			top_param_ns + "msg_types/format_2",
			&msg_types["FORMAT_2"]);
	bool found_exit_code = getParamWithTimeout(
			*nh,
			top_param_ns + "msg_types/exit",
			&msg_types["EXIT"]);

	ASSERTMSG_(found_format_1_code, "Code FORMAT #1 was not found");
	ASSERTMSG_(found_format_2_code, "Code FORMAT #2 was not found");
	ASSERTMSG_(found_exit_code, "Code EXIT was not found");

	// setup the TCP Socket Server
	MRPT_LOG_INFO_STREAM << "Setting up TCP Socket server." << endl
		<< "\tIP Address: " << server_addr << endl
		<< "\tPort: " << server_port_no << endl;
	m_server = new CServerTCPSocket(server_port_no, server_addr);
	ASSERT_(m_server->isListening());

	MRPT_LOG_INFO_STREAM << "Waiting for incoming connections..." << endl;
	m_client = m_server->accept(/* timeout_ms = */ -1);
	ASSERTMSG_(m_client,
			"Connection with the client could not be established...\nExiting...");

	// Reached here? Connection was successful.
	MRPT_LOG_INFO_STREAM << "Connection with client was successfully established"
		<< endl;
}



//
// Main
//
int main(int argc, char **argv)
{
	using namespace std;
	using namespace mrpt::poses;
	using namespace mrpt::utils;

	std::string node_name="topic_provider_node";

	// ros-related initialization actions
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);

	COutputLogger logger;
	logger.setLoggerName(node_name);
	// TODO - set the verbosity from a ros topic
	logger.setMinLoggingLevel(LVL_INFO);
	logger.logFmt(LVL_INFO, "Initializing node...");

	// subscribe to the topics that we want to read
	CTopicSniffer sniffer(&nh);
	sniffer.initServer();

	/**\brief Names of the parameters that point to the correct ROS topics from which to
	 * read the corresponding measurements.
	 */
	/**\{*/
	std::string scan_param_name="/topics/scan";
	std::string odom_param_name="/topics/odom";
	/**\}*/

	bool found;

	// Odometry
	// make sure that I know the topic off which I will be fetching the odometry
	std::string odom_topic("");
	found = getParamWithTimeout(nh, odom_param_name, &odom_topic, &logger);
	ASSERT_(found);

	ros::Subscriber odom_sub = nh.subscribe<arduino_mr::Pose2DStamped>(
			odom_topic,
			1000,
			&CTopicSniffer::sniffOdom, &sniffer);

	// LaserScans
	// make sure that I know the topic off which I will be fetching laser scans
	std::string scan_topic("");
	found = getParamWithTimeout(nh, scan_param_name, &scan_topic, &logger);
	ASSERT_(found);

	ros::Subscriber laserscans_sub = nh.subscribe<sensor_msgs::LaserScan>(
			scan_topic,
			1000,
			&CTopicSniffer::sniffLaserScan, &sniffer);


	while (ros::ok()) {

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

