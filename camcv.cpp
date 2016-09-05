#include <stdio.h>

#include "bcm_host.h"
#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_default_components.h"

#include "camcv.hpp"

// Standard port setting for the camera component
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

/// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM 3

#define CAM_ALIGN_UP(x, round) (x + round - 1) & ~(round - 1)

static MMAL_POOL_T *pool;
static MMAL_COMPONENT_T *camera = 0;
static MMAL_BUFFER_HEADER_T * volatile latestFrame = 0;
static int width, height, useRGB;
static volatile int copyingFrame = 0;

clock_t start;

#define startChron() do start = clock(); while(0)
#define stopChron(text) do printf("%s %lius\n", text, (clock()-start)*1000000/CLOCKS_PER_SEC); while(0)


// useless but required
static void camera_control_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    mmal_buffer_header_release(buffer);
}

/*
*  buffer header callback function for camera
*/
static void camera_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    MMAL_BUFFER_HEADER_T *new_buffer, *old_buffer;

    old_buffer = latestFrame;

    // wait if latest frame is being copied
    while(copyingFrame);
    mmal_buffer_header_mem_lock(buffer);
    latestFrame = buffer;

    // free last buffer
    if(old_buffer) {
        // release buffer back to the pool
        mmal_buffer_header_mem_unlock(old_buffer);
        mmal_buffer_header_release(old_buffer);

        // and send one back to the port (if still open)
        if (port->is_enabled) {
            MMAL_STATUS_T status;

            new_buffer = mmal_queue_get(pool->queue);

            if (new_buffer)
                status = mmal_port_send_buffer(port, new_buffer);

            if (!new_buffer || status != MMAL_SUCCESS)
                printf("Unable to return a buffer to the camera port\n");
        }
    }
}


/**
* Create the camera component, set up its ports
* returns MMAL_SUCCESS if all OK, something else otherwise
*/
static MMAL_STATUS_T create_camera_component(int framerate) {
    MMAL_ES_FORMAT_T *format;
    MMAL_PORT_T *video_port = NULL;
    MMAL_STATUS_T status;
    MMAL_PARAMETER_INT32_T camera_num = {{MMAL_PARAMETER_CAMERA_NUM, sizeof(camera_num)}, 0};

    /* Create the component */
    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);
    if (status != MMAL_SUCCESS) {
        printf("Failed to create camera component\n");
        goto err;
    }

    // use camera #0 (physically connected to raspberry pi camera header)
    status = mmal_port_parameter_set(camera->control, &camera_num.hdr);
    if (status != MMAL_SUCCESS) {
        printf("Could not select camera : error %d", status);
        goto err;
    }

    if (!camera->output_num) {
        status = MMAL_ENOSYS;
        printf("Camera doesn't have output ports");
        goto err;
    }

    status = mmal_port_parameter_set_uint32(camera->control, MMAL_PARAMETER_CAMERA_CUSTOM_SENSOR_CONFIG, 0);
    if (status != MMAL_SUCCESS) {
        printf("Could not set sensor mode : error %d", status);
        goto err;
    }

    video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];

    // Enable the camera, and tell it its control callback function
    status = mmal_port_enable(camera->control, camera_control_callback);
    if (status != MMAL_SUCCESS) {
        printf("Unable to enable control port : error %d", status);
        goto err;
    }

    //  set up the camera configuration
    {
        MMAL_PARAMETER_CAMERA_CONFIG_T cam_config = {
            { MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) },
            .max_stills_w = (uint32_t) width,
            .max_stills_h = (uint32_t) height,
            .stills_yuv422 = 0,
            .one_shot_stills = 0,
            .max_preview_video_w = (uint32_t) width,
            .max_preview_video_h = (uint32_t) height,
            .num_preview_video_frames = 3,
            .stills_capture_circular_buffer_height = 0,
            .fast_preview_resume = 0,
            .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
        };
        mmal_port_parameter_set(camera->control, &cam_config.hdr);
    }

    // Set the encode format on the video  port
    format = video_port->format;

    if (useRGB) {
        format->encoding = MMAL_ENCODING_BGR24;
        format->encoding_variant = 0;  //Irrelevant when not in opaque mode
    } else {
        format->encoding = MMAL_ENCODING_I420;
        format->encoding_variant = MMAL_ENCODING_I420;
    }

    format->es->video.width = CAM_ALIGN_UP(width, 32);
    format->es->video.height = CAM_ALIGN_UP(height, 16);
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = width;
    format->es->video.crop.height = height;
    format->es->video.frame_rate.num = framerate;
    format->es->video.frame_rate.den = 1;

    status = mmal_port_format_commit(video_port);

    if (status != MMAL_SUCCESS) {
        printf("camera video format couldn't be set");
        goto err;
    }

    // Ensure there are enough buffers to avoid dropping frames
    if (video_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
        video_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

    /* Enable component */
    status = mmal_component_enable(camera);
    if (status != MMAL_SUCCESS) {
        printf("camera component couldn't be enabled");
        goto err;
    }

    /* Create pool of buffer headers for the output port to consume */
    pool = mmal_port_pool_create(video_port, video_port->buffer_num, video_port->buffer_size);
    if (!pool) {
        printf("Failed to create buffer header pool for camera still port");
    }

    return status;
err:
    if (camera)
        mmal_component_destroy(camera);

    return status;
}

void destroyCamera() {
    if (camera) {
        if (camera->output[MMAL_CAMERA_CAPTURE_PORT] && camera->output[MMAL_CAMERA_CAPTURE_PORT]->is_enabled)
            mmal_port_disable(camera->output[MMAL_CAMERA_CAPTURE_PORT]);
        mmal_component_disable(camera);
        mmal_component_destroy(camera);
        camera = 0;
    }
}

int initCamera(int _width, int _height, int _framerate, int _useRGB) {
    int exit_code = 0;
    MMAL_STATUS_T status = MMAL_SUCCESS;
    MMAL_PORT_T *camera_video_port = NULL;

    bcm_host_init();

    width = _width;
    height = _height;
    useRGB = _useRGB;

    if ((status = create_camera_component(_framerate)) != MMAL_SUCCESS) {
        printf("Failed to create camera component\n");
        return -1;
    }

    camera_video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];

    // Enable the camera video port and tell it its callback function
    status = mmal_port_enable(camera_video_port, camera_buffer_callback);
    if (status != MMAL_SUCCESS) {
        printf("Failed to setup camera output\n");
        goto err2;
    }

    // Send all the buffers to the camera video port
    {
        int num = mmal_queue_length(pool->queue);
        int q;
        for (q=0;q<num;q++)
        {
            MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(pool->queue);

            if (!buffer)
                printf("Unable to get a required buffer %d from pool queue\n", q);

            if (mmal_port_send_buffer(camera_video_port, buffer)!= MMAL_SUCCESS)
                printf("Unable to send a buffer to camera video port (%d)\n", q);
        }
    }

    if (mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS) {
        printf("Failed to start capture\n");
        goto err2;
    }
    return MMAL_SUCCESS;

err2:
    destroyCamera();

    return exit_code;
}

cv::Mat getFrame() {
    int frameSize = width*height;
    uint8_t *frameBuffer = (uint8_t*) malloc(frameSize*3);
    uint8_t *latestData;
    int i=0;

    while(latestFrame == 0);

    // make sure latestFrame won't be modified during copy
    copyingFrame = 1;
    latestData = latestFrame->data;
    if(useRGB) {
        memcpy(frameBuffer, latestData, frameSize*3);
    } else {
        // translate I420 into a YUV matrix buffer
        for(int y=0; y<height; y++) {
            for(int x=0; x<width; x++) {
                frameBuffer[i] = latestData[i/3];
                frameBuffer[i+1] = latestData[frameSize + (x>>1) + (y >> 1)*(width >> 1)];
                frameBuffer[i+2] = latestData[frameSize*5/4 + (x>>1) + (y >> 1)*(width >> 1)];
                i+=3;
            }
        }
    }
    copyingFrame = 0;

    cv::Mat frame(height, width, CV_8UC3, frameBuffer);
    return frame;
}
