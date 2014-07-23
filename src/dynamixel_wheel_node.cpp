/*
 * dynamixel_wheel_node.cpp
 *
 *  Created on: Jul 16, 2014
 *      Author: sshuh
 */

#include "dynamixel_node.h"

int main( int argc, char **argv )
{
	printf("change mode: /dev/ttyUSB0\n");
	system("echo 3503764 | sudo -S chmod 666 /dev/ttyUSB0");

	printf("dynamixel_wheel_node started.\n");

	ros::init( argc, argv, "dynamixel_wheel_node" );
	ros::NodeHandle hNode;

	ClassDynamixelNode CDynamixel;

	int nVel = 0;
	if( argc == 1 )
		CDynamixel.Initiate( 30 );
	else
	{
		nVel = atoi(argv[1]);
		CDynamixel.Initiate( nVel );
	}

	while( ros::ok() )
	{
		CDynamixel.PublishTopic();

	}

	CDynamixel.Terminate();

	printf("dynamixel_wheel_node terminated.\n");

	return 0;
}
