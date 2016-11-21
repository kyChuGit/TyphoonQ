/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file led.cpp
 *
 * LED driver.
 */

#include <px4_config.h>
#include <drivers/device/device.h>
#include <drivers/drv_led.h>
#include <stdio.h>
#include <fcntl.h>

/*
 * Ideally we'd be able to get these from up_internal.h,
 * but since we want to be able to disable the NuttX use
 * of leds for system indication at will and there is no
 * separate switch, we need to build independent of the
 * CONFIG_ARCH_LEDS configuration switch.
 */
__BEGIN_DECLS
extern void led_init();
extern void led_on(int led);
extern void led_off(int led);
extern void led_toggle(int led);
__END_DECLS

#ifdef __PX4_NUTTX
class LED : device::CDev
#else
class LED : device::VDev
#endif
{
public:
	LED();
	virtual ~LED();

	virtual int		init();
	virtual int		ioctl(device::file_t *filp, int cmd, unsigned long arg);
};

extern "C" __EXPORT int led_test_main(int argc, char *argv[]);

LED::LED() :
#ifdef __PX4_NUTTX
	CDev("led", LED0_DEVICE_PATH)
#else
	VDev("led", LED0_DEVICE_PATH)
#endif
{
	// force immediate init/device registration
	init();
}

LED::~LED()
{
}

int
LED::init()
{
	DEVICE_DEBUG("LED::init");
#ifdef __PX4_NUTTX
	CDev::init();
#else
	VDev::init();
#endif
	led_init();

	return 0;
}

int
LED::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	int result = OK;

	switch (cmd) {
	case LED_ON:
		led_on(arg);
		break;

	case LED_OFF:
		led_off(arg);
		break;

	case LED_TOGGLE:
		led_toggle(arg);
		break;


	default:
#ifdef __PX4_NUTTX
		result = CDev::ioctl(filp, cmd, arg);
#else
		result = VDev::ioctl(filp, cmd, arg);
#endif
	}

	return result;
}

namespace
{
LED	*gLED;
}

void
drv_led_start(void)
{
	if (gLED == nullptr) {
		gLED = new LED;

		if (gLED != nullptr) {
			gLED->init();
		}
	}
}

namespace led_test
{
	int _led_fd = -1;
	int led_start(void);
	bool led_run(void);

	int led_start(void)
	{
		drv_led_start();
		if (gLED != nullptr)
			_led_fd = open(LED0_DEVICE_PATH, O_RDWR);
		return _led_fd;
	}
	bool led_run(void)
	{
		int ret;
		uint8_t i = 10;
		while(i --)
		{
			ioctl(_led_fd, LED_TOGGLE, 2);
			ret = ioctl(_led_fd, LED_TOGGLE, 0);
			if(ret < 0)
				return false;
			usleep(200000);
		}
		return true;
	}
}

int led_test_main(int argc, char *argv[])
{
	if(led_test::led_start())
	{
		if(!led_test::led_run())
			errx(1, "run err");
		else
			errx(0, "run ok");
	}
	else
		errx(1, "open err");
	exit(0);
}
