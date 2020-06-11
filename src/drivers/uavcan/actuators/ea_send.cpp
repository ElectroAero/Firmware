#include "ea_send.hpp"
#include <systemlib/err.h>

UavcanEASend::UavcanEASend(uavcan::INode &node): 
	_node(node), 
	_uavcan_pub_ea_send(node),
	_timer(node)
{

}

UavcanEASend::~UavcanEASend() {

}

int UavcanEASend::init() {
	memset(&battStatus, 0, sizeof(battStatus));
	memset(&vehPos, 0, sizeof(vehPos));

	batt_sub = orb_subscribe(ORB_ID(battery_status));
	vehPos_sub = orb_subscribe(ORB_ID(vehicle_gps_position));

	/*
	 * Setup timer and call back function for periodic updates
	 */
	_timer.setCallback(TimerCbBinder(this, &UavcanEASend::periodic_update));
	return 0;
}

void UavcanEASend::sendTelemetry(int value) {

	count = 0;

	/*
	 * Start the periodic update timer after a command is set
	 */
	if (!_timer.isRunning()) {
		_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(value));
	}
}

void UavcanEASend::periodic_update(const uavcan::TimerEvent &)
{

	//check for any new update
	bool battStatusUpdated;
	orb_check(batt_sub, &battStatusUpdated);

	bool vehPosUpdated;
	orb_check(vehPos_sub, &vehPosUpdated);

	//copy a local copy, can also check for any change with above boolean
	orb_copy(ORB_ID(battery_status), batt_sub, &battStatus);

 	//printf("BattVoltage=%.2f , ", (double)battStatus.voltage_v);
 	//printf("BattCurrent=%.2f , ", (double)battStatus.current_a);

 	orb_copy(ORB_ID(vehicle_gps_position), vehPos_sub, &vehPos);

 	//printf("Speed:%.2f \n", (double)vehPos.vel_m_s);

	uavcan::equipment::EASend msg;

	//use this as counter for datatype sending
	msg.count = count;

	//in decavolts
	msg.voltage = (int)(battStatus.voltage_v*100);

	//current in milliamps
	msg.current = (long)(battStatus.current_a*1000 + 8388608);

	msg.speed = (int)(vehPos.vel_m_s);

	// Broadcast command at MAX_RATE_HZ
	(void)_uavcan_pub_ea_send.broadcast(msg);

	count++;
}

