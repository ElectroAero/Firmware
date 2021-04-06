/****************************************************************************
 *
<<<<<<< HEAD:src/drivers/distance_sensor/a02yyuw/A02YYUW.hpp
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
=======
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
>>>>>>> 997abf382da830451d10a77575594d4427551637:boards/cubepilot/cubeorange/src/led.c
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
 * @file modified from TFMINI.hpp
 * @author Alex Thatcher <thatcher@electro.aero>
 *
<<<<<<< HEAD:src/drivers/distance_sensor/a02yyuw/A02YYUW.hpp
 * Declarations of driver for the DFRobot A02YYUW Ultrasonic Sensor
=======
 * LED backend.
>>>>>>> 997abf382da830451d10a77575594d4427551637:boards/cubepilot/cubeorange/src/led.c
 */

#pragma once

#include <termios.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <uORB/topics/distance_sensor.h>

#include "a02yyuw_parser.h"

<<<<<<< HEAD:src/drivers/distance_sensor/a02yyuw/A02YYUW.hpp
#define A02YYUW_DEFAULT_PORT	"/dev/ttyS2"

using namespace time_literals;
=======
/*
 * Ideally we'd be able to get these from arm_internal.h,
 * but since we want to be able to disable the NuttX use
 * of leds for system indication at will and there is no
 * separate switch, we need to build independent of the
 * CONFIG_ARCH_LEDS configuration switch.
 */
__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
extern void led_toggle(int led);
__END_DECLS

#  define xlat(p) (p)
static uint32_t g_ledmap[] = {
	GPIO_nLED_AMBER,
};
>>>>>>> 997abf382da830451d10a77575594d4427551637:boards/cubepilot/cubeorange/src/led.c

class A02YYUW : public px4::ScheduledWorkItem
{
<<<<<<< HEAD:src/drivers/distance_sensor/a02yyuw/A02YYUW.hpp
public:
	A02YYUW(const char *port, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	virtual ~A02YYUW();

	int init();

	void print_info();

private:
=======
	for (size_t l = 0; l < (sizeof(g_ledmap) / sizeof(g_ledmap[0])); l++) {
		if (g_ledmap[l] != 0) {
			stm32_configgpio(g_ledmap[l]);
		}
	}
}

static void phy_set_led(int led, bool state)
{
	/* Drive Low to switch on */
	if (g_ledmap[led] != 0) {
		stm32_gpiowrite(g_ledmap[led], !state);
	}
}

static bool phy_get_led(int led)
{
	/* If Low it is on */
	if (g_ledmap[led] != 0) {
		return !stm32_gpioread(g_ledmap[led]);
	}

	return false;
}
>>>>>>> 997abf382da830451d10a77575594d4427551637:boards/cubepilot/cubeorange/src/led.c

	int collect();

	void Run() override;

	void start();
	void stop();

	PX4Rangefinder	_px4_rangefinder;

	A02YYUW_PARSE_STATE _parse_state {A02YYUW_PARSE_STATE::STATE0_UNSYNC};

	char _linebuf[10] {};
	char _port[20] {};

	static constexpr int kCONVERSIONINTERVAL{9_ms};

	int _fd{-1};

	unsigned int _linebuf_index{0};

	hrt_abstime _last_read{0};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};

};
