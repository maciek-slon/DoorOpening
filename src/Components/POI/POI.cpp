/*!
 * \file
 * \brief
 * \author Maciej Stefanczyk
 */

#include <memory>
#include <string>

#include "POI.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace POI {

POI::POI(const std::string & name) :
		Base::Component(name)  {

	n[0] = -2;

}

POI::~POI() {
}

void POI::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_contours", &in_contours);
	registerStream("in_moments", &in_moments);
	registerStream("in_camera_info", &in_camera_info);
	registerStream("in_transform", &in_transform);
	registerStream("in_plane", &in_plane);
	registerStream("out_points", &out_points);
	// Register handlers
	registerHandler("readPlane", boost::bind(&POI::readPlane, this));
	registerHandler("sendPoints", boost::bind(&POI::sendPoints, this));
	registerHandler("readPoints", boost::bind(&POI::readPoints, this));
	addDependency("readPoints", &in_contours);
	addDependency("readPoints", &in_moments);
	addDependency("readPoints", &in_camera_info);
	addDependency("readPoints", &in_transform);

}

bool POI::onInit() {

	return true;
}

bool POI::onFinish() {
	return true;
}

bool POI::onStop() {
	return true;
}

bool POI::onStart() {
	return true;
}

void POI::readPlane() {
	if (!in_plane.empty()) {
		std::vector<float> plane = in_plane.read();
		
		n[0] = plane[0];
		n[1] = plane[1];
		n[2] = plane[2];
		
		p0 = n * fabs(plane[3]);
		
		CLOG(LINFO) << "Plane: " << n << ", " << p0;
	}
}

void POI::readPoints() {
	cv::Mat tf = in_transform.read();
	tf.convertTo(tf, CV_32F);
	std::vector<cv::Moments> moments = in_moments.read();
	std::vector<std::vector<cv::Point> > contours = in_contours.read();
	
	// no plane read yet => no calculations
	if (n[0] == -2) return;
	
	Types::CameraInfo camera_info = in_camera_info.read();
	
	for (int i = 0; i < moments.size(); ++i) {
		float u = moments[i].m10/moments[i].m00;
		float v = moments[i].m01/moments[i].m00;
		float fx = camera_info.fx();
		float fy = camera_info.fy();
		float cx = camera_info.cx();
		float cy = camera_info.cy();
		
		cv::Mat p0 = cv::Mat::zeros(4, 1, CV_32FC1);
		p0.at<float>(3,0) = 1;
		
		cv::Mat p1(4, 1, CV_32FC1);
		float x = (u - cx) / fx;
		float y = (v - cy) / fy;
		float z = 1;
		
		CLOG(LINFO) << "Contour " << i;
		CLOG(LINFO) << "\tp0: " << p0;
		CLOG(LINFO) << "\tp1: " << p1;
		
		p1.at<float>(0,0) = x;
		p1.at<float>(1,0) = y;
		p1.at<float>(2,0) = z;
		p1.at<float>(3,0) = 1;
		
		p0 = tf * p0;
		p1 = tf * p1;
		
		CLOG(LINFO) << "Transform";
		CLOG(LINFO) << "\tp0: " << p0;
		CLOG(LINFO) << "\tp1: " << p1;
		
		cv::Vec3f l0;
		l0[0] = p0.at<float>(0, 0);
		l0[1] = p0.at<float>(1, 0);
		l0[2] = p0.at<float>(2, 0);
		
		cv::Vec3f l1;
		l1[0] = p1.at<float>(0, 0);
		l1[1] = p1.at<float>(1, 0);
		l1[2] = p1.at<float>(2, 0);
		
		cv::Vec3f d = l1 - l0;
		
		CLOG(LINFO) << "Line: " << l0 << "," << d;
		
		cv::Vec3f diff = this->p0 - l0;
		float a = diff.dot(n);
		float b = d.dot(n);
		float c = a / b;
		
		cv::Vec3f pos = l0 + c * d;
		CLOG(LINFO) << "Calculated pos: " << pos;
		
		points.push_back(pos);
	}
}

void POI::sendPoints() {
	std::vector<cv::Vec6f> vec_points;
	for (int i = 0; i < points.size(); ++i) {
		cv::Vec6f pt;
		pt[0] = points[i][0];
		pt[1] = points[i][1];
		pt[2] = points[i][2];
		
		/*
		cv::Vec3f z;
		z[0] = 0;
		z[1] = 0;
		z[2] = 1;
		
		y = z.cross(n);*/
		
		pt[3] = n[0];
		pt[4] = n[1];
		pt[5] = n[2];
		
		vec_points.push_back(pt);
	}
	
	out_points.write(vec_points);
}



} //: namespace POI
} //: namespace Processors
