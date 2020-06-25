#pragma once

#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/EASend.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_gps_position.h>

#include <perf/perf_counter.h>

class UavcanEASend
{
	public:
		UavcanEASend(uavcan::INode &node);
		~UavcanEASend();

		int init();

		void sendTelemetry(int output);

	private:
		int count;

		struct battery_status_s battStatus;
		struct vehicle_gps_position_s vehPos;

		int batt_sub;
		int vehPos_sub;

		void periodic_update(const uavcan::TimerEvent &);
		typedef uavcan::MethodBinder<UavcanEASend *, 
			void (UavcanEASend::*)(const uavcan::TimerEvent &)> TimerCbBinder;

		pthread_mutex_t	_node_mutex;

		uavcan::INode 		&_node;
		uavcan::Publisher<uavcan::equipment::EASend> _uavcan_pub_ea_send;
		uavcan::TimerEventForwarder<TimerCbBinder> _timer;
};