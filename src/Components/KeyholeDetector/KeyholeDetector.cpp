/*!
 * \file
 * \brief
 * \author Maciej Stefanczyk
 */

#include <memory>
#include <string>

#include "KeyholeDetector.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace KeyholeDetector {

KeyholeDetector::KeyholeDetector(const std::string & name) :
		Base::Component(name)  {

}

KeyholeDetector::~KeyholeDetector() {
}

void KeyholeDetector::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_homography", &in_homography);
	registerStream("in_point", &in_point);
	registerStream("in_plane", &in_plane);
	registerStream("in_transform", &in_transform);
	registerStream("in_camera_info", &in_camera_info);
	registerStream("out_point", &out_point);
	
	// Register handlers
	registerHandler("onNewPoint", boost::bind(&KeyholeDetector::onNewPoint, this));
	addDependency("onNewPoint", &in_point);
	
	registerHandler("onNewData", boost::bind(&KeyholeDetector::onNewData, this));
	addDependency("onNewData", &in_homography);
	addDependency("onNewData", &in_transform);
	addDependency("onNewData", &in_camera_info);
	
	registerHandler("readPlane", boost::bind(&KeyholeDetector::readPlane, this));
}

bool KeyholeDetector::onInit() {

	return true;
}

bool KeyholeDetector::onFinish() {
	return true;
}

bool KeyholeDetector::onStop() {
	return true;
}

bool KeyholeDetector::onStart() {
	return true;
}

void KeyholeDetector::readPlane() {
	if (!in_plane.empty()) {
		/*std::vector<float> plane = in_plane.read();
		
		n[0] = plane[0];
		n[1] = plane[1];
		n[2] = plane[2];
		
		n0 = n * fabs(plane[3]);*/
		
		cv::Vec6f plane = in_plane.read();
		n0[0] = plane[0];
		n0[1] = plane[1];
		n0[2] = plane[2];
		n[0] = plane[3];
		n[1] = plane[4];
		n[2] = plane[5];
		
		CLOG(LNOTICE) << "Detector plane: " << n << ", " << n0;
	}
}

void KeyholeDetector::onNewPoint() {
	point = in_point.read();
}

void KeyholeDetector::onNewData() {
	cv::Mat H = in_homography.read();
	cv::Mat tf = in_transform.read();
	tf.convertTo(tf, CV_32F);
	Types::CameraInfo ci = in_camera_info.read();
	
	std::vector<cv::Point2f> points;
	std::vector<cv::Point2f> out;
	points.push_back(point);
	
	// transform keyhole coordinates from model image to camera image
	cv::perspectiveTransform(points, out, H);
	
	
	
	
	float u = out[0].x;
	float v = out[0].y;
	float fx = ci.fx();
	float fy = ci.fy();
	float cx = ci.cx();
	float cy = ci.cy();
	
	// first point of line - optical center of camera
	cv::Mat p0 = cv::Mat::zeros(4, 1, CV_32FC1);
	p0.at<float>(3,0) = 1;
	
	// second point of line - 1 meter away through selected pixel
	cv::Mat p1(4, 1, CV_32FC1);
	float x = (u - cx) / fx;
	float y = (v - cy) / fy;
	float z = 1;
		
	p1.at<float>(0,0) = x;
	p1.at<float>(1,0) = y;
	p1.at<float>(2,0) = z;
	p1.at<float>(3,0) = 1;
	
	// transform both points to global frame
	p0 = tf * p0;
	p1 = tf * p1;
	
	// point on line - first point
	cv::Vec3f l0;
	l0[0] = p0.at<float>(0, 0);
	l0[1] = p0.at<float>(1, 0);
	l0[2] = p0.at<float>(2, 0);
	
	// point on line - second point
	cv::Vec3f l1;
	l1[0] = p1.at<float>(0, 0);
	l1[1] = p1.at<float>(1, 0);
	l1[2] = p1.at<float>(2, 0);
	
	// line direction
	cv::Vec3f d = l1 - l0;
	
	// intersection of line and plane
	cv::Vec3f diff = n0 - l0;
	float a = diff.dot(n);
	float b = d.dot(n);
	float c = a / b;
	
	cv::Vec3f pos = l0 + c * d;
	CLOG(LNOTICE) << "Calculated pos: " << pos;
	
	cv::Point3f result;
	result.x = pos[0];
	result.y = pos[1];
	result.z = pos[2];
	
	out_point.write(result);
}



} //: namespace KeyholeDetector
} //: namespace Processors
