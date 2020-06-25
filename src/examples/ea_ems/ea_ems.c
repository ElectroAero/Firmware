/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

 /**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_log.h>
#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
#include <uORB/topics/power_monitor.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <nuttx/sched.h>
//#include <systemlib/systemlib.h>
#include <systemlib/err.h>


static bool thread_should_exit = false; /**< daemon exit flag */
static bool thread_running = false; /**< daemon status flag */
static int daemon_task; /**< Handle of daemon task / thread */


__EXPORT int ea_ems_main(int argc, char *argv[]);

/**
* Mainloop of daemon.
*/
int ea_ems_daemon_app_main(int argc, char *argv[]);

/**
* Print the correct usage.
*/
static void usage(const char *reason);

static void usage(const char *reason) {
	if (reason) {
	 	warnx("%s\n", reason);
	}
 	warnx("usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}


int ea_ems_main(int argc, char *argv[]) {

	if (argc < 2) {
 		usage("Missing command");
 		return 1;
 	}

 	else if (!strcmp(argv[1], "start")) {
 		if (thread_running) {
 			warnx("daemon already running\n");
 			return 0;
 		}

 		thread_should_exit = false;
 		
 		daemon_task = px4_task_spawn_cmd("ea_ems", 
 			SCHED_DEFAULT, 
 			SCHED_PRIORITY_DEFAULT, 
 			2000, 
 			ea_ems_daemon_app_main, 
 			(argv) ? (char * const *)&argv[2] : (char * const*)NULL);
 			
		return 0;
 	}

 	else if (!strcmp(argv[1], "stop")) {
 		thread_should_exit = true;
 		return 0;
 	}

 	else if (!strcmp(argv[1], "status")) {
 		if (thread_running) {
 			warnx("\trunning\n");
 		} 
 		else {
 			warnx("\tnot started\n");
 		}
 		return 0;
 	}
 	else {
 		usage("unrecognized command");
 		return 1;
 	}
 	
 	return 1;
}

int ea_ems_daemon_app_main(int argc, char *argv[]) {

	warnx("[Daemon] starting\n");
	thread_running = true;

	printf("Started Thread! Subscribing to battery UOrb...\n");

	//Structs to contain the system state
	struct power_monitor_s battStatus;
	memset(&battStatus, 0, sizeof(battStatus));
	struct vehicle_gps_position_s vehPos;
	memset(&vehPos, 0, sizeof(vehPos));

	//Subscribe to topics
	int batt_sub = orb_subscribe(ORB_ID(power_monitor));
	int vehPos_sub = orb_subscribe(ORB_ID(vehicle_gps_position));

	while(!thread_should_exit) {

		//check for any new update
		bool battStatusUpdated;
		orb_check(batt_sub, &battStatusUpdated);

		bool vehPosUpdated;
		orb_check(vehPos_sub, &vehPosUpdated);

		//copy a local copy, can also check for any change with above boolean
		orb_copy(ORB_ID(power_monitor), batt_sub, &battStatus);

 		printf("BattVoltage=%.2f , ", (double)battStatus.voltage_v);
 		printf("BattCurrent=%.2f , ", (double)battStatus.current_a);

 		orb_copy(ORB_ID(vehicle_gps_position), vehPos_sub, &vehPos);

 		printf("Speed:%.2f \n", (double)vehPos.vel_m_s);

 		sleep(1);
	}

	warnx("[Daemon] exiting.\n");
 	thread_running = false;
 	return OK;
 }
/*
	bool updated_battery;
	struct battery_status_s battStatus;

	static int topic_handle_battery;
	topic_handle_battery = orb_subscribe(ORB_ID(battery_status));

 	if(topic_handle_battery==ERROR){
 		printf("Error orb_subscribe (ERROR)=%d\n",errno);
 		sleep(10);
 		topic_handle_battery = orb_subscribe(ORB_ID(battery_status));
 	}
 	else if(topic_handle_battery==-1){
 		printf("Error orb_subscribe (-1)=%d\n",errno);
 		sleep(10);
 		topic_handle_battery = orb_subscribe(ORB_ID(battery_status));
 	}

 	if(orb_set_interval(topic_handle_battery,1000)==ERROR){
 		printf("Error orb_set_interval =%d\n",errno);
 		return ERROR;
 	}

	if(orb_copy(ORB_ID(battery_status),topic_handle_battery,&battStatus)==ERROR){
 		printf("Before Looop Battery Error orb_copy =%d\n",errno);
 		return ERROR;
 	}


	bool updated_position;
	struct vehicle_gps_position_s positionStatus;

	static int topic_handle_position;
	topic_handle_position = orb_subscribe(ORB_ID(vehicle_gps_position));

 	 if(topic_handle_position==ERROR){
 		printf("Error orb_subscribe (ERROR)=%d\n",errno);
 		sleep(10);
 		topic_handle_position = orb_subscribe(ORB_ID(vehicle_gps_position));
 	}
 	else if(topic_handle_position==-1){
 		printf("Error orb_subscribe (-1)=%d\n",errno);
 		sleep(10);
 		topic_handle_position = orb_subscribe(ORB_ID(vehicle_gps_position));
 	}

 	if(orb_set_interval(topic_handle_position,1000)==ERROR){
 		printf("Error orb_set_interval =%d\n",errno);
 		return ERROR;
 	}

 	 if(orb_copy(ORB_ID(vehicle_gps_position),topic_handle_position,&positionStatus)==ERROR){
 		printf("Before Loop Position Error orb_copy =%d\n",errno);
 		return ERROR;
 	}

 	while (!thread_should_exit) {
 		printf("Thread looping... :");

 		if(orb_check(topic_handle_battery, &updated_battery)==ERROR){
 			printf("Battery Error orb_check =%d\n",errno);
 			if(orb_check(topic_handle_battery, &updated_battery)==ERROR){
 				printf("Battery Error orb_check =%d\n",errno);
 				return ERROR;
 			}
 		}
 		if(updated_battery){
 			if(orb_copy(ORB_ID(battery_status),topic_handle_battery,&battStatus)==ERROR){
 				printf("Battery Error orb_copy =%d\n",errno);
 				if(orb_copy(ORB_ID(battery_status),topic_handle_battery,&battStatus)==ERROR){
 					printf("Battery Second Error orb_copy =%d\n",errno);
 					return ERROR;
 				}
 			}
 			printf("BattVoltage=%.2f , ", (double)battStatus.voltage_v);
 			printf("BattCurrent=%.2f\n", (double)battStatus.current_a);
 		}
 		else {
 			printf("Battery Not updated");
 		}

 		if(orb_check(topic_handle_position, &updated_position)==ERROR){
 			printf("Position Error orb_check =%d\n",errno);
 			if(orb_check(topic_handle_position, &updated_position)==ERROR){
 				printf("Position Error orb_check =%d\n",errno);
 				return ERROR;
 			}
 		}

		if(updated_position){
 			if(orb_copy(ORB_ID(vehicle_gps_position),topic_handle_position,&positionStatus)==ERROR){
 				printf("Postiion Error orb_copy =%d\n",errno);
 				if(orb_copy(ORB_ID(vehicle_gps_position),topic_handle_position,&positionStatus)==ERROR){
 					printf("Position Second Error orb_copy =%d\n",errno);
 					return ERROR;
 				}
 			}
 			printf("Velocity=%.2f , ", (double)positionStatus.vel_m_s);
 		}
 		else {
 			printf("Position Not updated");
 		}
 		
 		sleep(1);
 	}
 	
 	warnx("[Daemon] exiting.\n");
 	thread_running = false;
 	return OK;
}

*/