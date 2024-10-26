/****************************************************************************
 * Date: July 5, 2024
 * Author: Shubhanshu Gupta
 *
 * Outer loop trajectory control for transition of tailsitter vehicle
****************************************************************************/

#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>

uORB::Subscription	_vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
uORB::Subscription 	_local_pos_sub{ORB_ID(vehicle_local_position)};			// sensor subscription

vehicle_attitude_s		_v_att{};
vehicle_local_position_s	_local_pos{};
