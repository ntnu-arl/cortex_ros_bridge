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

#include <tf/transform_datatypes.h>
#include <tuple>
#include "cortex.h"

using namespace tf;

#define piDiv180 0.01745329251

std::tuple<bool, StampedTransform> CreateTransform ( sFrameOfData* FrameOfData, int frame_index, const char* body, float* origin_offset )
{
	// initialize variables
	double pos_x, pos_y, pos_z, roll, pitch, yaw;
  int numSegments;
	bool valid_data = true;
	ros::Time tf_stamp;
	StampedTransform transform;
	tSegmentData* Segments = FrameOfData->BodyData[frame_index].Segments;
	numSegments = FrameOfData->BodyData[frame_index].nSegments;

	if ( Segments != NULL )
	{
		// Publish the central marker and data for each segment
		for ( int i = 0; i < numSegments; ++i )
		{
			// get body data
			pos_x = 0.001*(double)Segments[i][0]; // x
			pos_y = 0.001*(double)Segments[i][1]; // y
			pos_z = 0.001*(double)Segments[i][2]; // z
			roll = (piDiv180)*Segments[i][3]; // roll (in rad)
			pitch = (piDiv180)*Segments[i][4]; // pitch (in rad)
			yaw = (piDiv180)*Segments[i][5]; // yaw (in rad)

			// assure body data is valid
			if ( ( pos_x < -2 || pos_x > 2 ) ||
						( pos_y < -1.25 || pos_y > 1.25 ) ||
						 ( pos_z < 0 || pos_z > 1.75 ) )
			{
				valid_data = false;
			}

			// subtract any offset for the body
			pos_x -= origin_offset[0];
			pos_y -= origin_offset[1];
			pos_z -= origin_offset[2];
			roll = ( roll - origin_offset[3] );
			pitch = ( pitch - origin_offset[4] );
			yaw = ( yaw - origin_offset[5] );

			// create transform
			Vector3 base_trans = Vector3(pos_x, pos_y, pos_z);
			Quaternion base_quat = createQuaternionFromRPY(roll, pitch, yaw);
			Transform bodyTransform = Transform(base_quat, base_trans);

			// create time stamped tranform to broadcast
			tf_stamp = ros::Time::now();
			transform = StampedTransform(bodyTransform, tf_stamp, "world", body);

			// return created transform
			return std::make_tuple ( valid_data, transform );
		}
	}
	Vector3 base_trans = Vector3(0,0,0);
	Quaternion base_quat = createQuaternionFromRPY(0,0,0);
	Transform badTransform = Transform(base_quat, base_trans);
	StampedTransform bad = StampedTransform(badTransform, ros::Time::now(), "world", "bad");
	return std::make_tuple ( false, bad );
}
