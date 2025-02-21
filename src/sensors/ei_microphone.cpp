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

/* Include ----------------------------------------------------------------- */
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(microphone, CONFIG_EI_LOG_LEVEL);
#include <zephyr/kernel.h>
#include <zephyr/audio/dmic.h>
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"
#include "firmware-sdk/ei_device_memory.h"
#include "firmware-sdk/ei_device_info_lib.h"
#include "firmware-sdk/sensor-aq/sensor_aq_none.h"
#include "ei_microphone.h"

#define AUDIO_FREQ          16000
#define BYTES_PER_SAMPLE    sizeof(int16_t)
#define SAMPLE_BIT_WIDTH    (BYTES_PER_SAMPLE * 8)
#define BLOCK_COUNT         4
#define BLOCK_LENGTH_MS     100
#define BLOCKS_PER_SECOND   (1000 / BLOCK_LENGTH_MS)
#define READ_TIMEOUT        5 // Milliseconds to wait for a block to be read
/* Size of a block for 100 ms of audio data. */
#define BLOCK_SIZE(_sample_rate, _number_of_channels) \
	(BYTES_PER_SAMPLE * (_sample_rate / BLOCKS_PER_SECOND) * _number_of_channels)
#define MAX_BLOCK_SIZE   BLOCK_SIZE(AUDIO_FREQ, 1)
// Allocate memory slab, aligned to 4 bytes
K_MEM_SLAB_DEFINE_STATIC(rx_mem_slab, MAX_BLOCK_SIZE, BLOCK_COUNT, 4);

static size_t ei_write(const void*, size_t size, size_t count, EI_SENSOR_AQ_STREAM*);
static int ei_seek(EI_SENSOR_AQ_STREAM*, long int offset, int origin);
static void inference_sampling_timer_handler(struct k_timer *dummy);
static void inference_sampling_work_handler(struct k_work *work);
K_MUTEX_DEFINE(inference_mutex);
K_TIMER_DEFINE(inference_sampling_timer, inference_sampling_timer_handler, NULL);
K_WORK_DEFINE(inference_sampling_work, inference_sampling_work_handler);

/** Status and control struct for inferencing struct */
typedef struct {
    microphone_sample_t *buffers[2];
    uint8_t buf_select;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

const struct device *mic_dev;
static uint32_t headerOffset;
static uint32_t required_samples;
static uint32_t collected_samples;
static volatile inference_t inference;
static EiDeviceMemory* memory = EiDeviceInfo::get_device()->get_memory();
static unsigned char ei_mic_ctx_buffer[1024];
static sensor_aq_signing_ctx_t ei_mic_signing_ctx;
static sensor_aq_ctx ei_mic_ctx = {
    { ei_mic_ctx_buffer, 1024 },
    &ei_mic_signing_ctx,
    &ei_write,
    &ei_seek,
    NULL,
};

/* Dummy functions for sensor_aq_ctx type */
static size_t ei_write(const void*, size_t size, size_t count, EI_SENSOR_AQ_STREAM*)
{
    return count;
}

static int ei_seek(EI_SENSOR_AQ_STREAM*, long int offset, int origin)
{
    return 0;
}

static int insert_ref(char *buffer, int hdrLength)
{
    #define EXTRA_BYTES(a)  ((a & 0x3) ? 4 - (a & 0x3) : (a & 0x03))
    const char *ref = "Ref-BINARY-i16";
    int addLength = 0;
    int padding = EXTRA_BYTES(hdrLength);

    buffer[addLength++] = 0x60 + 14 + padding;
    for(size_t i = 0; i < strlen(ref); i++) {
        buffer[addLength++] = *(ref + i);
    }
    for(int i = 0; i < padding; i++) {
        buffer[addLength++] = ' ';
    }

    buffer[addLength++] = 0xFF;

    return addLength;
}

static bool create_header(sensor_aq_payload_info *payload)
{
    int ret;
    sensor_aq_init_none_context(&ei_mic_signing_ctx);

    ret = sensor_aq_init(&ei_mic_ctx, payload, NULL, true);

    if (ret != AQ_OK) {
        LOG_ERR("sensor_aq_init failed (%d)", ret);
        return false;
    }

    // then we're gonna find the last byte that is not 0x00 in the CBOR buffer.
    // That should give us the whole header
    size_t end_of_header_ix = 0;
    for (size_t ix = ei_mic_ctx.cbor_buffer.len - 1; ix >= 0; ix--) {
        if (((uint8_t*)ei_mic_ctx.cbor_buffer.ptr)[ix] != 0x0) {
            end_of_header_ix = ix;
            break;
        }
    }

    if (end_of_header_ix == 0) {
        LOG_ERR("Failed to find end of header");
        return false;
    }

    int ref_size = insert_ref(((char*)ei_mic_ctx.cbor_buffer.ptr + end_of_header_ix), end_of_header_ix);

    end_of_header_ix += ref_size;

    // Write to blockdevice
    ret = memory->write_sample_data((uint8_t*)ei_mic_ctx.cbor_buffer.ptr, 0, end_of_header_ix);
    if ((size_t)ret != end_of_header_ix) {
        LOG_ERR("Failed to write to header blockdevice (%d)", ret);
        return false;
    }

    headerOffset = end_of_header_ix;

    return true;
}

bool ei_microphone_init(void)
{
    mic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));
    if (!device_is_ready(mic_dev)) {
		LOG_ERR("%s: device not ready", mic_dev->name);
        return false;
	}

    return true;
}

bool ei_microphone_sample_start(void)
{
    EiDeviceInfo* dev = EiDeviceInfo::get_device();
    int ret;
    void *buffer;
    uint32_t size;
    bool first_block_discarded = false;

    ei_printf("Sampling settings:\n");
    ei_printf("\tInterval: %.5f ms.\n", dev->get_sample_interval_ms());
    ei_printf("\tLength: %u ms.\n", dev->get_sample_length_ms());
    ei_printf("\tName: %s\n", dev->get_sample_label().c_str());
    ei_printf("\tHMAC Key: %s\n", dev->get_sample_hmac_key().c_str());
    ei_printf("\tFile name: %s\n", dev->get_sample_label().c_str());

    sensor_aq_payload_info payload = {
        dev->get_device_id().c_str(),
        dev->get_device_type().c_str(),
        dev->get_sample_interval_ms(),
        { { "audio", "wav" } }
    };

    /* Audio sample rate sanity check */
    if ((uint32_t)(1000.f / dev->get_sample_interval_ms()) != AUDIO_FREQ) {
        LOG_ERR("Only 16 kHz sampling rate is supported!");
        return false;
    }

    required_samples = (uint32_t)((dev->get_sample_length_ms()) / dev->get_sample_interval_ms());

    /* Round to even number of samples for word align flash write */
    if(required_samples & 1) {
        required_samples++;
    }

    collected_samples = 0;

    if(required_samples * sizeof(microphone_sample_t) > memory->get_available_sample_bytes()) {
        LOG_ERR("Sample length is too long. Maximum allowed is %u ms at 16000 Hz.",
            ((memory->get_available_sample_bytes() / (16000 * sizeof(microphone_sample_t))) * 1000));
        return false;
    }

    dev->set_state(eiStateErasingFlash);

    // Minimum delay of 2000 ms for daemon
    uint32_t delay_time_ms = ((required_samples * sizeof(microphone_sample_t) / memory->block_size) + 1) * memory->block_erase_time;
    ei_printf("Starting in %u ms... (or until all flash was erased)\n", delay_time_ms < 2000 ? 2000 : delay_time_ms);

    if(memory->erase_sample_data(0, required_samples * sizeof(microphone_sample_t)) != (required_samples * sizeof(microphone_sample_t))) {
        return false;
    }

    // if erasing took less than 2 seconds, wait additional time
    if(delay_time_ms < 2000) {
        ei_sleep(2000 - delay_time_ms);
    }

    if (create_header(&payload) == false) {
        return false;
    }

	struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_BIT_WIDTH,
		.mem_slab  = &rx_mem_slab,
	};
	struct dmic_cfg cfg = {
		.io = {
			/* These fields can be used to limit the PDM clock
			 * configurations that the driver is allowed to use
			 * to those supported by the microphone.
			 */
			.min_pdm_clk_freq = 1000000,
			.max_pdm_clk_freq = 3500000,
			.min_pdm_clk_dc   = 40,
			.max_pdm_clk_dc   = 60,
		},
		.streams = &stream,
		.channel = {
			.req_num_streams = 1,
		},
	};

	cfg.channel.req_num_chan = 1;
	cfg.channel.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
	cfg.streams[0].pcm_rate = AUDIO_FREQ;
	cfg.streams[0].block_size = BLOCK_SIZE(cfg.streams[0].pcm_rate, cfg.channel.req_num_chan);

	ret = dmic_configure(mic_dev, &cfg);
	if (ret) {
		LOG_ERR("DMIC configure error (%d)", ret);
        return false;
	}

    ei_printf("Sampling...\n");

    dev->set_state(eiStateSampling);

    ret = dmic_trigger(mic_dev, DMIC_TRIGGER_START);
    if (ret < 0) {
        LOG_ERR("START trigger failed: %d", ret);
        return false;
    }

    dev->set_state(eiStateSampling);

    while(collected_samples < required_samples) {
        ret = dmic_read(mic_dev, 0, &buffer, &size, READ_TIMEOUT);
        if (ret < 0) {
            LOG_WRN("DMIC read failed: %d", ret);
			continue;
		}

        if(first_block_discarded == false) {
            first_block_discarded = true;
            k_mem_slab_free(&rx_mem_slab, &buffer);
            continue;
        }

        memory->write_sample_data((uint8_t*)buffer, headerOffset + collected_samples * sizeof(microphone_sample_t), size);
        k_mem_slab_free(&rx_mem_slab, &buffer);
        collected_samples += size / sizeof(microphone_sample_t);
        k_yield();
    }

	ret = dmic_trigger(mic_dev, DMIC_TRIGGER_STOP);
	if (ret < 0) {
		LOG_ERR("STOP trigger failed: %d", ret);
		return false;
	}

    ei_printf("Done sampling, total bytes collected: %u\n", collected_samples * sizeof(microphone_sample_t));
    ei_printf("[1/1] Uploading file to Edge Impulse...\n");
    ei_printf("Not uploading file, not connected to WiFi. Used buffer, from=0, to=%u.\n", collected_samples * sizeof(microphone_sample_t) + headerOffset);
    ei_printf("OK\n");

    return true;
}

int ei_microphone_inference_get_data(size_t offset, size_t length, float *out_ptr)
{
    int ret;

    k_mutex_lock(&inference_mutex, K_FOREVER);
    ret = ei::numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);
    k_mutex_unlock(&inference_mutex);

    return ret;
}

bool ei_microphone_inference_start(uint32_t n_samples, float interval_ms)
{
    int ret;

    k_mutex_lock(&inference_mutex, K_FOREVER);
    inference.buffers[0] = (microphone_sample_t*)ei_malloc(n_samples * sizeof(microphone_sample_t));
    if(inference.buffers[0] == nullptr) {
        LOG_ERR("Can't allocate audio buffer[0]");
        return false;
    }

    inference.buffers[1] = (microphone_sample_t*)ei_malloc(n_samples * sizeof(microphone_sample_t));
    if(inference.buffers[0] == nullptr) {
        LOG_ERR("Can't allocate audio buffer[1]");
        ei_free(inference.buffers[0]);
        return false;
    }

    inference.buf_select = 0;
    inference.buf_count  = 0;
    inference.n_samples  = n_samples;
    inference.buf_ready  = 0;
    k_mutex_unlock(&inference_mutex);

	struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_BIT_WIDTH,
		.mem_slab  = &rx_mem_slab,
	};

	struct dmic_cfg cfg = {
		.io = {
			/* These fields can be used to limit the PDM clock
			 * configurations that the driver is allowed to use
			 * to those supported by the microphone.
			 */
			.min_pdm_clk_freq = 1000000,
			.max_pdm_clk_freq = 3500000,
			.min_pdm_clk_dc   = 40,
			.max_pdm_clk_dc   = 60,
		},
		.streams = &stream,
		.channel = {
			.req_num_streams = 1,
		},
	};

	cfg.channel.req_num_chan = 1;
	cfg.channel.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
	cfg.streams[0].pcm_rate = AUDIO_FREQ;
	cfg.streams[0].block_size = BLOCK_SIZE(cfg.streams[0].pcm_rate, cfg.channel.req_num_chan);

	ret = dmic_configure(mic_dev, &cfg);
	if (ret) {
		LOG_ERR("DMIC configure error (%d)", ret);
        ei_free(inference.buffers[0]);
        ei_free(inference.buffers[1]);
        return false;
	}

    ret = dmic_trigger(mic_dev, DMIC_TRIGGER_START);
    if (ret < 0) {
        LOG_ERR("START trigger failed: %d", ret);
        ei_free(inference.buffers[0]);
        ei_free(inference.buffers[1]);
        return false;
    }

    k_timer_start(&inference_sampling_timer, K_MSEC(BLOCK_LENGTH_MS), K_MSEC(BLOCK_LENGTH_MS));

    return true;
}

bool ei_microphone_inference_is_recording(void)
{
    return inference.buf_ready == 0;
}

void ei_microphone_inference_reset_buffers(void)
{
    k_mutex_lock(&inference_mutex, K_FOREVER);
    inference.buf_select = 0;
    inference.buf_ready = 0;
    inference.buf_count = 0;
    k_mutex_unlock(&inference_mutex);
}

bool ei_microphone_inference_end(void)
{
    int ret;

    k_timer_stop(&inference_sampling_timer);

    ret = dmic_trigger(mic_dev, DMIC_TRIGGER_STOP);
	if (ret < 0) {
		LOG_ERR("STOP trigger failed: %d", ret);
		return false;
	}

    k_mutex_lock(&inference_mutex, K_FOREVER);
    ei_free(inference.buffers[0]);
    ei_free(inference.buffers[1]);
    k_mutex_unlock(&inference_mutex);

    return true;
}

static void inference_sampling_work_handler(struct k_work *work)
{
    int ret;
    void *buffer;
    uint32_t size;

    ret = dmic_read(mic_dev, 0, &buffer, &size, READ_TIMEOUT);
    if (ret < 0) {
        LOG_WRN("DMIC read failed: %d", ret);
        return;
    }

    // prevent buffer overflow
    if(inference.buf_count + (size / sizeof(microphone_sample_t)) > inference.n_samples) {
        size = (inference.n_samples - inference.buf_count) * sizeof(microphone_sample_t);
    }

    k_mutex_lock(&inference_mutex, K_FOREVER);
    memcpy(&inference.buffers[inference.buf_select][inference.buf_count], buffer, size);

    k_mem_slab_free(&rx_mem_slab, &buffer);

    inference.buf_count += (size / sizeof(microphone_sample_t));

    if(inference.buf_count >= inference.n_samples) {
        inference.buf_select ^= 1;
        inference.buf_count = 0;
        inference.buf_ready = 1;
    }
    k_mutex_unlock(&inference_mutex);
}

static void inference_sampling_timer_handler(struct k_timer *dummy)
{
    k_work_submit(&inference_sampling_work);
}
