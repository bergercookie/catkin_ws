/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "ros/ros.h"
#include <mrpt_bridge/mrpt_bridge.h>
#include <mrpt/poses/CPose2D.h>

#include <iostream>
#include <string>

int main(int argc, char **argv)
{
	using namespace std;
	using namespace mrpt::poses;

	std::string node_name="topic_provider";

	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;

	CPose2D p(15, 0, 0.2);
	ros::Rate loop_rate(10);

	while (ros::ok()) {
		cout << p << endl;

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

