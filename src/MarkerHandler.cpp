/*
		Copyright 2015 Tyler Sorey, ARL, University of Nevada, Reno, USA

    This file is part of cortex_bridge.

    cortex_bridge is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    cortex_bridge is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "cortex.h"
#include "cortex_bridge/Markers.h"
#include "cortex_bridge/Marker.h"

visualization_msgs::MarkerArray CreateMarkerArray_vis ( sFrameOfData* FrameOfData )
{
	// initialize variables
	std::stringstream convert;
	visualization_msgs::MarkerArray markers_msg;
	visualization_msgs::Marker marker_tmp;
	int numMarkers = FrameOfData->nUnidentifiedMarkers;
	tMarkerData* UMarkers = FrameOfData->UnidentifiedMarkers;
	if ( UMarkers == NULL )
		return markers_msg;

	// initialize markers message
	marker_tmp.type = visualization_msgs::Marker::SPHERE;
	marker_tmp.header.stamp = ros::Time::now();
	marker_tmp.header.frame_id = "/world";
	marker_tmp.action = visualization_msgs::Marker::ADD;

	// add the markers of known bodies
	for ( int i = 0; i < FrameOfData->nBodies; ++i )
	{
		for ( int j = 0; j < FrameOfData->BodyData[i].nMarkers; ++j )
		{
			// name marker
			convert << "m" << (i+1) << "-" << j;
			marker_tmp.ns = convert.str();
			convert.str( std::string() );
			convert.clear();

			// add marker translation
			marker_tmp.pose.position.x = 0.001*(double)FrameOfData->BodyData[i].Markers[j][0]; // X
			marker_tmp.pose.position.y = 0.001*(double)FrameOfData->BodyData[i].Markers[j][1]; // Y
			marker_tmp.pose.position.z = 0.001*(double)FrameOfData->BodyData[i].Markers[j][2]; // Z
	
			marker_tmp.pose.orientation.x = 0.0;
			marker_tmp.pose.orientation.y = 0.0;
			marker_tmp.pose.orientation.z = 0.0;
			marker_tmp.pose.orientation.w = 1.0;

			// add marker scale
			marker_tmp.scale.x = 0.05;
			marker_tmp.scale.y = 0.05;
			marker_tmp.scale.z = 0.05;
	
			// set color
			marker_tmp.color.r = 255.0;
			marker_tmp.color.g = 0.0;
			marker_tmp.color.b = 0.0;
			marker_tmp.color.a = 1.0;

			// set lifetime
			marker_tmp.lifetime = ros::Duration();

			// add marker to msg
			markers_msg.markers.push_back ( marker_tmp );
		}
	}

	// iterate through all markers adding them to marker vector
	for ( int i = 0; i < numMarkers; ++i )
	{
		// name marker
		convert << "u" << (i+1);
		marker_tmp.ns = convert.str();
		convert.str( std::string() );
		convert.clear();

		// add marker translation
		marker_tmp.pose.position.x = 0.001*(double)UMarkers[i][0]; // X
		marker_tmp.pose.position.y = 0.001*(double)UMarkers[i][1]; // Y
		marker_tmp.pose.position.z = 0.001*(double)UMarkers[i][2]; // Z

		marker_tmp.pose.orientation.x = 0.0;
		marker_tmp.pose.orientation.y = 0.0;
		marker_tmp.pose.orientation.z = 0.0;
		marker_tmp.pose.orientation.w = 1.0;

		// add marker scale
		marker_tmp.scale.x = 0.05;
		marker_tmp.scale.y = 0.05;
		marker_tmp.scale.z = 0.05;

		// set color 
		marker_tmp.color.r = 255.0;
		marker_tmp.color.g = 255.0;
		marker_tmp.color.b = 255.0;
		marker_tmp.color.a = 1.0;

		// set lifetime
		marker_tmp.lifetime = ros::Duration();

		// add marker to msg
		markers_msg.markers.push_back ( marker_tmp );
	}
	// return marker array
	return markers_msg;
}

cortex_bridge::Markers CreateMarkerArray_novis ( sFrameOfData* FrameOfData )
{
	// initialize variables
	std::stringstream convert;
	int numMarkers = FrameOfData->nUnidentifiedMarkers;
	cortex_bridge::Markers markers_msg;
	cortex_bridge::Marker marker_tmp;
	tMarkerData* UMarkers = FrameOfData->UnidentifiedMarkers;
	if ( UMarkers == NULL )
		return markers_msg;

	// initialize markers message
	markers_msg.header.stamp = ros::Time::now();
	markers_msg.header.frame_id = "/world";
	markers_msg.frame_number = FrameOfData->iFrame;

	// add the markers of known bodies
	for ( int i = 0; i < FrameOfData->nBodies; ++i )
	{
		for ( int j = 0; j < FrameOfData->BodyData[i].nMarkers; ++j )
		{
			// name marker
			convert << "m" << (i+1) << "-" << j;
			marker_tmp.marker_name = convert.str();
			convert.str( std::string() );
			convert.clear();

			// subject name
			convert << FrameOfData->BodyData[i].szName;
			marker_tmp.subject_name = convert.str();
			convert.str( std::string() );
			convert.clear();

			// add marker translation
			marker_tmp.translation.x = 0.001*(double)FrameOfData->BodyData[i].Markers[j][0]; // X
			marker_tmp.translation.y = 0.001*(double)FrameOfData->BodyData[i].Markers[j][1]; // Y
			marker_tmp.translation.z = 0.001*(double)FrameOfData->BodyData[i].Markers[j][2]; // Z

			// is marker occluded?
			marker_tmp.occluded = false; // for now assume if cameras can see it then we are 
																	// not occluded...but look into this more later

			// add marker to msg
			markers_msg.markers.push_back ( marker_tmp );
		}
	}

	// iterate through all markers adding them to marker vector
	for ( int i = 0; i < numMarkers; ++i )
	{
		// name marker
		convert << "u" << (i+1);
		marker_tmp.marker_name = convert.str();
		convert.str( std::string() );
		convert.clear();

		// add marker translation
		marker_tmp.translation.x = 0.001*(double)UMarkers[i][0]; // X
		marker_tmp.translation.y = 0.001*(double)UMarkers[i][1]; // Y
		marker_tmp.translation.z = 0.001*(double)UMarkers[i][2]; // Z

		// is marker occluded?
		marker_tmp.occluded = false; // for now assume if cameras can see it then we are 
																	// not occluded...but look into this more later

		// add marker to msg
		markers_msg.markers.push_back ( marker_tmp );
	}
	
	// return array
	return markers_msg;
}
