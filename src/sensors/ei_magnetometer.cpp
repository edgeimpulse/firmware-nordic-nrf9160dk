/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(magnetometer, CONFIG_EI_LOG_LEVEL);
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include "ei_magnetometer.h"

const struct device *const iis2mdc = DEVICE_DT_GET_ONE(st_iis2mdc);

static void iis2mdc_config(const struct device *iis2mdc)
{
	struct sensor_value odr_attr;

	/* set IIS2MDC sampling frequency to 100 Hz */
	odr_attr.val1 = 100;
	odr_attr.val2 = 0;

	if (sensor_attr_set(iis2mdc, SENSOR_CHAN_ALL,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		LOG_ERR("Cannot set sampling frequency for IIS2MDC");
		return;
	}
}

bool ei_magnetometer_init(void)
{
	if (!device_is_ready(iis2mdc)) {
		LOG_ERR("%s: device not ready.", iis2mdc->name);
		return false;
	}

    iis2mdc_config(iis2mdc);

    if(ei_add_sensor_to_fusion_list(magnetometer_sensor) == false) {
        LOG_ERR("ERR: failed to register magnetometer sensor!");
        return false;
    }

    return true;
}

float *ei_fusion_magnetometer_read_data(int n_samples)
{
    struct sensor_value magn[MAG_AXIS_SAMPLED];
    static float magn_mt[MAG_AXIS_SAMPLED];

    memset(magn_mt, 0, MAG_AXIS_SAMPLED * sizeof(float));

    if (sensor_sample_fetch(iis2mdc) < 0) {
        LOG_ERR("IIS2MDC Magn Sensor sample update error");
    }
    else {
        sensor_channel_get(iis2mdc, SENSOR_CHAN_MAGN_XYZ, magn);
        // convert gauss to militesla
        magn_mt[0] = sensor_value_to_double(&magn[0]) / 10.0f;
        magn_mt[1] = sensor_value_to_double(&magn[1]) / 10.0f;
        magn_mt[2] = sensor_value_to_double(&magn[2]) / 10.0f;
    }

    return magn_mt;
}