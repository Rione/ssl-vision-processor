/*
     Copyright 2024 Felix Weinmann

     Licensed under the Apache License, Version 2.0 (the "License");
     you may not use this file except in compliance with the License.
     You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

     Unless required by applicable law or agreed to in writing, software
     distributed under the License is distributed on an "AS IS" BASIS,
     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
     See the License for the specific language governing permissions and
     limitations under the License.
 */
#include "opencvdriver.h"

#include <linux/v4l2-controls.h>

OpenCVDriver::OpenCVDriver(const CameraConfig& config): capture(config.path, cv::CAP_ANY, {cv::CAP_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_ANY}), name(config.path) {
	std::replace(name.begin(), name.end(), '/', '_');

	// Use compressed data stream to unlock the highest resolution - framerate combination on USB2 cameras
	capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

	if(config.autoResolution()) {
		capture.set(cv::CAP_PROP_FRAME_WIDTH, INT_MAX);
		capture.set(cv::CAP_PROP_FRAME_HEIGHT, INT_MAX);
	} else {
		capture.set(cv::CAP_PROP_FRAME_WIDTH, config.width);
		capture.set(cv::CAP_PROP_FRAME_HEIGHT, config.height);
	}

	// The V4L2 backend passes values unconverted to the camera controls
	const bool v4l2 = (int)capture.get(cv::CAP_PROP_BACKEND) == cv::CAP_V4L2;

	if(config.autoExposure()) {
		if(!v4l2) {
			capture.set(cv::CAP_PROP_AUTO_EXPOSURE, 1.0);
		} else if(!capture.set(cv::CAP_PROP_AUTO_EXPOSURE, V4L2_EXPOSURE_APERTURE_PRIORITY)) {
			// Most UVC cameras only offer manual and aperture priority mode
			capture.set(cv::CAP_PROP_AUTO_EXPOSURE, V4L2_EXPOSURE_AUTO);
		}
	} else {
		capture.set(cv::CAP_PROP_AUTO_EXPOSURE, v4l2 ? (double)V4L2_EXPOSURE_MANUAL : 0.0);
		// V4L2 exposure unit is 100 µs
		capture.set(cv::CAP_PROP_EXPOSURE, config.exposure * (v4l2 ? 10.0 : 1000.0));
	}

	if(!config.autoGain()) {
		capture.set(cv::CAP_PROP_GAIN, config.gain);
	}

	if(v4l2) {
		// V4L2 has no gamma switch: send 1.0 (no gamma) explicitly, UVC unit is gamma * 100
		capture.set(cv::CAP_PROP_GAMMA, config.gamma * 100.0);
	} else if(!config.autoGamma()) {
		capture.set(cv::CAP_PROP_GAMMA, config.gamma);
	}

	if(config.whiteBalanceType != WhiteBalanceType_Manual) {
		capture.set(cv::CAP_PROP_AUTO_WB, 1.0);
	} else {
		capture.set(cv::CAP_PROP_AUTO_WB, 0.0);
		capture.set(cv::CAP_PROP_WHITE_BALANCE_BLUE_U, config.whiteBalanceBlue);
		capture.set(cv::CAP_PROP_WHITE_BALANCE_RED_V, config.whiteBalanceRed);
	}
}

std::shared_ptr<RawImage> OpenCVDriver::readImage() {
	if(image == nullptr || !image.unique())
		image = std::make_shared<RawImage>(&PixelFormat::BGR8, capture.get(cv::CAP_PROP_FRAME_WIDTH), capture.get(cv::CAP_PROP_FRAME_HEIGHT), name);

	CLMap<uint8_t> map = image->write<uint8_t>();
	cv::Mat mat(cv::Size(image->width, image->height), CV_8UC3, (void*)*map);
	if(!capture.read(mat))
		return nullptr;

	return image;
}

const PixelFormat OpenCVDriver::format() {
	return PixelFormat::BGR8;
}

double OpenCVDriver::expectedFrametime() {
	double fps = capture.get(cv::CAP_PROP_FPS);

	if(fps == 0.0) // Unavailable for cameras, estimate 30 FPS
		fps = 30.0;

	return 1 / fps;
}


double OpenCVDriver::getTime() {
	double pos = capture.get(cv::CAP_PROP_POS_FRAMES);

	if(pos == -1) // Not a video file, use real time
		return getRealTime();

	return pos / capture.get(cv::CAP_PROP_FPS);
}
