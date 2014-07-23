/*
 * dynamixel_node.h
 *
 *  Created on: Feb 7, 2014
 *      Author: sshuh
 */

#ifndef DYNAMIXEL_NODE_H_
#define DYNAMIXEL_NODE_H_

#endif /* DYNAMIXEL_NODE_H_ */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include "rover/msg_dynamixel.h" // automatically generated by CMakeLists.txt

#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <termio.h>
#include "dynamixel/dynamixel.h"

#ifndef PI
	#define PI		3.14159265358979
#endif
#ifndef PIf
	#define PIf		3.14159265358979f
#endif


#define P_CW_ANGLE_LIMIT	6
#define P_CCW_ANGLE_LIMIT	8

#define P_TORQUE_ENABLE	24
#define P_LED_CONTROL	25

#define P_GOAL_POSITION_L	30

#define P_GOAL_SPEED_L	32
#define P_GOAL_SPEED_H	33

#define P_PRESENT_POSITION_L	36

#define P_MOVING		46

#define DEFAULT_ID		1


class ClassDynamixelNode
{
public:
	ros::NodeHandle m_hNode;
	ros::Subscriber m_Subscriber;
	ros::Publisher m_Publisher;

	int iTimer;
	rover::msg_dynamixel m_msgPub;


	ClassDynamixelNode()
	{
	}


	~ClassDynamixelNode()
	{
	}

	void Initiate( int nVel )
	{
		m_Publisher = m_hNode.advertise<rover::msg_dynamixel>("msg_dynamixel", 10);
		//m_Subscriber = m_hNode.subscribe<std_msgs::Float64>("/tilt_controller/state/current_pos", 1, &ClassDynamixelNode::ScanCallback, this );
		iTimer = 0;

		if( dxl_initialize(0,34) == 0 )
		{
			printf("Failed:Open USB2Dynamixel\n");
		}
		else
		{
			printf("Initiate:Open USB2Dynamixel\n");
		}

		usleep(100000);
		dxl_write_byte(1, P_LED_CONTROL, 0x01);
		usleep(100000);
		short nPresentPos = dxl_read_word(1, P_PRESENT_POSITION_L);
		usleep(100000);

		printf("PresentPos=%d\n", nPresentPos);


		//dxl_write_byte(1, P_TORQUE_ENABLE, 0x01);

		usleep(100000);
		dxl_write_word(1, P_CW_ANGLE_LIMIT, 0);
		dxl_write_word(1, P_CCW_ANGLE_LIMIT, 0);

		usleep(10000);


		int nMaxVelLimit = 200;
		if( nVel > 0 )
		{
			if( nVel>nMaxVelLimit )
			{
				nVel = nMaxVelLimit;
				printf("Cmd exceeds MaxVelLimit=%d  ", nMaxVelLimit);
			}
			printf("PresentVel=%d\n", nVel);
		}
		if( nVel < 0 )
		{
			if( nVel<-nMaxVelLimit )
			{
				nVel = -nMaxVelLimit;
				printf("Cmd exceeds MaxVelLimit=%d  ", -nMaxVelLimit);
			}

			printf("PresentVel=%d\n", nVel);
			nVel = 1024 - nVel;
		}

		if( nVel != 0 )
			dxl_write_word(1, P_GOAL_SPEED_L, nVel);
		else
			printf("PresentVel=%d  No Torque.\n", nVel);
	}


	void PublishTopic()
	{
		usleep(10000);

		m_msgPub.header.stamp = ros::Time::now();
		m_msgPub.nStep = dxl_read_word(1, P_PRESENT_POSITION_L);
		m_msgPub.dAngle = 0.001533980788 * (double)(m_msgPub.nStep);
		if(m_msgPub.dAngle > PI)
			m_msgPub.dAngle = m_msgPub.dAngle -2.0*PI;

		m_Publisher.publish(m_msgPub);
	}


	void Terminate()
	{
		usleep(100000);
		dxl_write_byte(1, P_LED_CONTROL, 0x00);
		usleep(100000);

		dxl_write_byte(1, P_TORQUE_ENABLE, 0x00);
		usleep(100000);
		dxl_terminate();
	}


	// Print communication result
	void PrintCommStatus(int CommStatus)
	{
		switch(CommStatus)
		{
		case COMM_TXFAIL:
			printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
			break;

		case COMM_TXERROR:
			printf("COMM_TXERROR: Incorrect instruction packet!\n");
			break;

		case COMM_RXFAIL:
			printf("COMM_RXFAIL: Failed get status packet from device!\n");
			break;

		case COMM_RXWAITING:
			printf("COMM_RXWAITING: Now recieving status packet!\n");
			break;

		case COMM_RXTIMEOUT:
			printf("COMM_RXTIMEOUT: There is no status packet!\n");
			break;

		case COMM_RXCORRUPT:
			printf("COMM_RXCORRUPT: Incorrect status packet!\n");
			break;

		default:
			printf("This is unknown error code!\n");
			break;
		}
	}

	// Print error bit of status packet
	void PrintErrorCode()
	{
		if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
			printf("Input voltage error!\n");

		if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
			printf("Angle limit error!\n");

		if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
			printf("Overheat error!\n");

		if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
			printf("Out of range error!\n");

		if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
			printf("Checksum error!\n");

		if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
			printf("Overload error!\n");

		if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
			printf("Instruction code error!\n");
	}



};



