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
bool getParamWithTimeout(
		const ros::NodeHandle& nh,
		const std::string& param_name,
		std::string* param_content,
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
bool getParamWithTimeout(
		const ros::NodeHandle& nh,
		const std::string& param_name,
		std::string* param_content,
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
		logger->logFmt(
				LVL_INFO,
				"Successfully fetched parameter: \"%s\" ==> \"%s\"",
				param_name.c_str(),
				param_content->c_str());
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
	msg.type = msg_types["FORMAT 2"];
	msg.serializeObject(mrpt_laser_scan.pointer());
	m_client->sendMessage(msg);
	MRPT_LOG_DEBUG_STREAM << "LaserScan was sent.";

}

void CTopicSniffer::sniffOdom() {
	using namespace std;
	using namespace mrpt::utils;
	using namespace mrpt::obs;

	MRPT_LOG_DEBUG_STREAM << "sniffOdom: Received an odometry msg. Converting it to MRPT format...";

	// build and fill an CObservationOdometry instance

	CObservationOdometryPtr mrpt_odom = CObservationOdometry::Create();

	// serialize and send it to the TCP stream
	//
}

void CTopicSniffer::initServer() {
	using namespace std;
	using namespace mrpt::utils;

	// initialization...
	// TODO - define these in a seperate class - .ini file?
	msg_types["FORMAT 1"] = 1; /**< Transmit data in the 1st rawlog MRPT format */
	msg_types["FORMAT 2"] = 2; /**< Transmit data in the 2nd rawlog MRPT format */
	msg_types["EXIT"] = 99; /**< Code is returned when no more data is to be transmitted */

	// fetch the server parameters from the ROS parameter server
	ASSERT_(nh);

	MRPT_LOG_DEBUG_STREAM << "Fetching the server configuration from the ROS parameter server..." << endl;
	bool found_server = getParamWithTimeout(
			*nh,
			"/graphslam_engine/tcp/server_addr",
			&server_addr);

	// TODO - Change the implementation on this one as well...
	bool found_port = nh->getParam(
			"/graphslam_engine/tcp/server_port_no", server_port_no);
	ASSERTMSG_(found_port,
			"TCP Port was not found in the parameter server")

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

	// Odometry
	//ros::Subscriber odom_sub = nh.subscribe("odom/", 1000, &CTopicSniffer::sniffOdom, &sniffer);

	// LaserScans
	// make sure that I know the topic off which I will be fetching laser scans
	std::string scan_topic("");
	bool found = getParamWithTimeout(nh, scan_param_name, &scan_topic, &logger);
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

