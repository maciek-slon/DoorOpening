/*!
 * \file
 * \brief
 * \author Maciej Stefanczyk
 */

#include <memory>
#include <string>

#include "MatTransform.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace MatTransform {

MatTransform::MatTransform(const std::string & name) :
		Base::Component(name)  {

}

MatTransform::~MatTransform() {
}

void MatTransform::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_img", &in_img);
	registerStream("in_transform", &in_transform);
	registerStream("in_point", &in_point);
	registerStream("out_img", &out_img);
	registerStream("lockPosition", &lockPosition);
	// Register handlers
	registerHandler("onNewImage", boost::bind(&MatTransform::onNewImage, this));
	addDependency("onNewImage", &in_img);
	addDependency("onNewImage", &in_transform);

}

bool MatTransform::onInit() {

	return true;
}

bool MatTransform::onFinish() {
	return true;
}

bool MatTransform::onStop() {
	return true;
}

bool MatTransform::onStart() {
	return true;
}

void MatTransform::onNewImage() {
	cv::Mat img = in_img.read();
	cv::Mat tf = in_transform.read();
	cv::Point target_pos(320, 240);
	if (!in_point.empty()) target_pos = in_point.read();
	
	float z_fix = 0.02;
	
	// check, if image has proper number of channels
	if (img.channels() != 3) {
		CLOG(LERROR) << "MatTransform: Wrong number of channels";
		return;
	}
	
	// check image depth, allowed is only 32F and 64F
	int img_type = img.depth();
	if ( (img_type != CV_32F) && (img_type != CV_64F) ) {
		CLOG(LERROR) << "MatTransform: Wrong depth";
		return;
	}
	
	// temporary point
	cv::Mat pt(4, 1, img_type);
	
	// clone image - operation will change its contents
	img = img.clone();
	
	// clone transform and change its type to match image type
	tf.clone();
	
	tf.convertTo(tf, img_type);
	
	//
	// iterate through all image pixels
	//
	
	int rows = img.rows;
	int cols = img.cols;
	
	if (img.isContinuous()) {
		cols *= rows;
		rows = 1;
	}
	
	// float variant
	if (img_type == CV_32F) {
		CLOG(LINFO) << "Transflorming CV_32F";
		int i,j;
		float* p;
		
		cv::Vec3f ptt = img.at<cv::Vec3f>(target_pos.y, target_pos.x);
		CLOG(LNOTICE) << ptt[0];
		CLOG(LNOTICE) << ptt[1];
		CLOG(LNOTICE) << ptt[2];
		
		for( i = 0; i < rows; ++i) {
			p = img.ptr<float>(i);
			for ( j = 0; j < cols; ++j) {
				// read point coordinates 
				pt.at<float>(0, 0) = p[3*j];
				pt.at<float>(1, 0) = p[3*j + 1];
				pt.at<float>(2, 0) = p[3*j + 2] + z_fix;
				pt.at<float>(3, 0) = 1;

				// transform point
				pt = tf * pt;

				// write back result
				p[3*j]   = pt.at<float>(0, 0);
				p[3*j+1] = pt.at<float>(1, 0);
				p[3*j+2] = pt.at<float>(2, 0);
				
			}
		}
		
		img.at<cv::Vec3f>(target_pos.y, target_pos.x);
		CLOG(LNOTICE) << ptt[0];
		CLOG(LNOTICE) << ptt[1];
		CLOG(LNOTICE) << ptt[2];
		
		lockPosition.write(ptt);
	} else { // double variant
		CLOG(LINFO) << "Transflorming CV_64F";
		int i,j;
		double* p;
		cv::Vec3d ptt = img.at<cv::Vec3d>(target_pos.y, target_pos.x);
		CLOG(LNOTICE) << ptt[0];
		CLOG(LNOTICE) << ptt[1];
		CLOG(LNOTICE) << ptt[2];
		
		for( i = 0; i < rows; ++i) {
			p = img.ptr<double>(i);
			for ( j = 0; j < cols; ++j) {
				pt.at<double>(0, 0) = p[3*j];
				pt.at<double>(1, 0) = p[3*j + 1];
				pt.at<double>(2, 0) = p[3*j + 2] + z_fix;
				pt.at<double>(3, 0) = 1;

				pt = tf * pt;

				p[3*j]   = pt.at<double>(0, 0);
				p[3*j+1] = pt.at<double>(1, 0);
				p[3*j+2] = pt.at<double>(2, 0);
			}
		}
		
		ptt = img.at<cv::Vec3d>(target_pos.y, target_pos.x);
		CLOG(LNOTICE) << ptt[0];
		CLOG(LNOTICE) << ptt[1];
		CLOG(LNOTICE) << ptt[2];
		
		lockPosition.write(ptt);
	}
	
	out_img.write(img);
}



} //: namespace MatTransform
} //: namespace Processors
