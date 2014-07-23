/*
 * sick_lidar_node.cpp
 *
 *  Created on: Jul 16, 2014
 *      Author: sshuh
 */

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include "rover/msg_dynamixel.h"

#include "HS2_LaserLMS151.h"

double g_dDynamixelAngle = 0.0;

void Callback_Dynamixel( const rover::msg_dynamixel::ConstPtr& pData )
{
	//if( (iTimer++ %4) == 0 )
	//	printf("Size=%d  Range=%.3f\n", pScan->ranges.size(), pScan->ranges[540]);

	//printf("dAngle=%.3f  nStep=%d\n", pData->dAngle, pData->nStep);
	g_dDynamixelAngle = pData->dAngle;

}


int main( int argc, char **argv )
{
	printf("Initiate:sick_lidar_node\n");

	ros::init( argc, argv, "sick_lidar_node" );
	ros::NodeHandle hNode;


	ros::NodeHandle hNodeDynamixel;
	ros::Subscriber m_Subscriber;
	m_Subscriber = hNodeDynamixel.subscribe<rover::msg_dynamixel>("msg_dynamixel", 1, &Callback_Dynamixel);


	unsigned short usLaserData1[1081];

	HS2_LaserLMS151 CLaser;

	// ASCII = 2111, binary = 2112
	if( CLaser.OpenLaserScanner( "192.168.0.151", 2112 ) > 0 )
	{
		printf("LMS151 Connected\n");
		CLaser.InitializeLaserParams();
	}
	else
		return -1;


	ros::Publisher Publisher_Laser;
	Publisher_Laser = hNode.advertise<sensor_msgs::LaserScan>("msg_lms151", 10);

	sensor_msgs::LaserScanPtr msg_lms151(new sensor_msgs::LaserScan());
	msg_lms151->header.frame_id = "world";
	msg_lms151->header.stamp = ros::Time::now();

	msg_lms151->angle_min = -2.35619449615; //-135.0 deg
	msg_lms151->angle_max = 2.35619449615; //135.0 deg
	msg_lms151->angle_increment = 0.00436332309619; //0.25 deg
	msg_lms151->time_increment = 0.00002777777778; // 0.04/1440
	msg_lms151->scan_time = 0.04f; // 1/25
	msg_lms151->range_min = 0.05f;
	msg_lms151->range_max = 80.0f;

	msg_lms151->ranges.resize(1081);

	//msg_lms151->intensities.resize(1081);


	//ros::spin();
/*
	tf::TransformBroadcaster tf_TfBc;
	tf::Transform tf_Tf;
*/

	unsigned int nNumPoints = 1081;

	ros::NodeHandle hNodePointCloud2;
	ros::Publisher m_PublisherPointCloud2;
	m_PublisherPointCloud2 = hNodePointCloud2.advertise<sensor_msgs::PointCloud2>("msg_PointCloud2", 10);

	sensor_msgs::PointCloud2Ptr msg_PointCloud2(new sensor_msgs::PointCloud2());

	msg_PointCloud2->header.stamp = ros::Time::now();
	msg_PointCloud2->header.frame_id = "world";

	msg_PointCloud2->height = 1;
	msg_PointCloud2->width = nNumPoints;

	msg_PointCloud2->fields.resize(3);

	msg_PointCloud2->fields[0].name = "x";
	msg_PointCloud2->fields[0].offset = 0;
	msg_PointCloud2->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
	msg_PointCloud2->fields[0].count = 1;

	msg_PointCloud2->fields[1].name = "y";
	msg_PointCloud2->fields[1].offset = 4;
	msg_PointCloud2->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
	msg_PointCloud2->fields[1].count = 1;

	msg_PointCloud2->fields[2].name = "z";
	msg_PointCloud2->fields[2].offset = 8;
	msg_PointCloud2->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
	msg_PointCloud2->fields[2].count = 1;

	msg_PointCloud2->is_bigendian = false;

	msg_PointCloud2->point_step = 12;

	msg_PointCloud2->row_step = 12*nNumPoints;

	msg_PointCloud2->data.resize(msg_PointCloud2->row_step);

	float arrPoint[3] = {1.f, 2.f, 3.f};
	memcpy( (void*)(&msg_PointCloud2->data[0]), (void*)arrPoint, 12 );


	msg_PointCloud2->is_dense = true;


////////

	std::vector <float> fLookup_Cos;
	std::vector <float> fLookup_Sin;

//	std::vector <double> dLookup_CosElevation;
//	std::vector <double> dLookup_SinElevation;

	fLookup_Cos.resize(1081);
	fLookup_Sin.resize(1081);

//	dLookup_CosElevation.resize(1081);
//	dLookup_SinElevation.resize(1081);

	for( int i=0; i<1081; i++ )
	{
		fLookup_Cos[i] = (cos((-135.0+(float)(i-0)*0.25f)*D2RF));
		fLookup_Sin[i] = (sin((-135.0+(float)(i-0)*0.25f)*D2RF));

//		dLookup_CosElevation[i] = (cos((90.0-0.25*(double)abs(i-540))*D2R));
//		dLookup_SinElevation[i] = (sin((90.0-0.25*(double)abs(i-540))*D2R));

	}

////////////

	CLaser.ReadScanData_ContinuousStart_Binary();
	while( ros::ok() )
	{
		msg_PointCloud2->header.stamp = ros::Time::now();

		msg_lms151->header.stamp = ros::Time::now();

		//lms151_L.ReadScanData_Single_Ascii( g_TcpSensorData.LRFdata );

//		CLaser.ReadScanData_Single_Binary( usLaserData1 );
		CLaser.ReadScanData_Answer_Binary( usLaserData1 );


		for( int i=0; i<1081; i++ )
		{
			msg_lms151->ranges[i] = 0.001f * (float)(usLaserData1[i]);
		}

		//printf("L %.3f  C %.3f  R %.3f\n", usLaserData1[180], usLaserData1[540], usLaserData1[900] );
		printf("L %d  C %d  R %d\n", usLaserData1[180], usLaserData1[540], usLaserData1[900] );


		Publisher_Laser.publish( msg_lms151 );


		float fCosSpin = (float)cos(g_dDynamixelAngle);
		float fSinSpin = (float)sin(g_dDynamixelAngle);

		for( int i=0; i<1081; i++ )
		{
			arrPoint[0] = msg_lms151->ranges[i] * fLookup_Sin[i] * fCosSpin;
			arrPoint[1] = msg_lms151->ranges[i] * fLookup_Sin[i] * fSinSpin;
			arrPoint[2] = msg_lms151->ranges[i] * fLookup_Cos[i];
			memcpy( (void*)(&msg_PointCloud2->data[i*12]), (void*)arrPoint, 12 );
		}




		m_PublisherPointCloud2.publish( msg_PointCloud2 );

		ros::spinOnce();

		usleep(33000);

	}
	CLaser.ReadScanData_ContinuousStop_Binary();

	CLaser.CloseLaserScanner();

	printf("Terminate:sick_lidar_node\n");

	return 0;
}



