/* Edge Impulse ingestion SDK
 * Copyright (c) 2023 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Include ----------------------------------------------------------------- */
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(accelerometer, CONFIG_EI_LOG_LEVEL);
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include "ei_accelerometer.h"

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2    9.80665f

const struct device *iis2dlpc;

static void iis2dlpc_config(const struct device *iis2dlpc)
{
	struct sensor_value odr_attr, fs_attr;

	/* set IIS2DLPC accel/gyro sampling frequency to 1600 Hz */
	odr_attr.val1 = 1600;
	odr_attr.val2 = 0;

	if (sensor_attr_set(iis2dlpc, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		LOG_ERR("Cannot set sampling frequency for IIS2DLPC accel");
		return;
	}

	sensor_g_to_ms2(2, &fs_attr);

	if (sensor_attr_set(iis2dlpc, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_FULL_SCALE, &fs_attr) < 0) {
		LOG_ERR("Cannot set sampling frequency for IIS2DLPC gyro");
		return;
	}
}

bool ei_accelerometer_init(void)
{
    iis2dlpc = DEVICE_DT_GET_ONE(st_iis2dlpc);
    if (!device_is_ready(iis2dlpc))
    {
        LOG_ERR("%s: device not ready.", iis2dlpc->name);
        return false;
    }

    iis2dlpc_config(iis2dlpc);

    if(ei_add_sensor_to_fusion_list(accelerometer_sensor) == false) {
        LOG_ERR("ERR: failed to register accelerometer sensor!");
        return false;
    }

    return true;
}


float *ei_fusion_accelerometer_read_data(int n_samples)
{
    struct sensor_value accel2[ACCEL_AXIS_SAMPLED];
    static float acceleration_g[ACCEL_AXIS_SAMPLED];

    memset(acceleration_g, 0, ACCEL_AXIS_SAMPLED * sizeof(float));

    if (sensor_sample_fetch(iis2dlpc) < 0) {
        LOG_ERR("IIS2DLPC Sensor sample update error");
    }
    else {
        sensor_channel_get(iis2dlpc, SENSOR_CHAN_ACCEL_XYZ, accel2);
        acceleration_g[0] = sensor_value_to_double(&accel2[0]);
        acceleration_g[1] = sensor_value_to_double(&accel2[1]);
        acceleration_g[2] = sensor_value_to_double(&accel2[2]);
    }

    return acceleration_g;
}

