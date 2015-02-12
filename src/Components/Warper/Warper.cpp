/*!
 * \file
 * \brief
 * \author Maciej Stefanczyk
 */

#include <memory>
#include <string>

#include "Warper.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>


#include <opencv2/stitching/stitcher.hpp>

namespace Processors {
namespace Warper {

Warper::Warper(const std::string & name) :
		Base::Component(name)  {

}

Warper::~Warper() {
}

void Warper::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_img", &in_img);
	registerStream("in_transform", &in_transform);
	registerStream("in_camerainfo", &in_camerainfo);
	registerStream("out_img", &out_img);
	// Register handlers
	registerHandler("onNewImage", boost::bind(&Warper::onNewImage, this));
	addDependency("onNewImage", &in_img);
	addDependency("onNewImage", &in_transform);
	addDependency("onNewImage", &in_camerainfo);

}

bool Warper::onInit() {

	return true;
}

bool Warper::onFinish() {
	return true;
}

bool Warper::onStop() {
	return true;
}

bool Warper::onStart() {
	return true;
}

void Warper::onNewImage() {
	cv::detail::PlaneWarper * warper = new cv::detail::PlaneWarper(640);
	cv::Mat img = in_img.read();
	cv::Mat tf = in_transform.read();
	cv::Mat R = tf(cv::Rect(0, 0, 3, 3));
	cv::Mat R2 = cv::Mat::zeros(3, 3, CV_32FC1);
	R2.at<float>(0, 1) = -1;
	R2.at<float>(1, 2) = -1;
	R2.at<float>(2, 0) =  1;
	R.convertTo(R, CV_32F);
	R = R * R2;
	Types::CameraInfo ci = in_camerainfo.read();
	cv::Mat out;
	CLOG(LNOTICE) << "K: " << ci.cameraMatrix();
	//cv::transpose(R, R);
	CLOG(LNOTICE) << "R: " << R;
	warper->warp(img, ci.cameraMatrix(), R, cv::Mat::zeros(3, 1, CV_32FC1), cv::INTER_LINEAR, cv::BORDER_REFLECT, out);
	out_img.write(out);
}



} //: namespace Warper
} //: namespace Processors
