/*!
 * \file
 * \brief
 * \author Maciej Stefanczyk
 */

#include <memory>
#include <string>

#include "TransformGenerator.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace TransformGenerator {

TransformGenerator::TransformGenerator(const std::string & name) :
		Base::Component(name) , 
		x("x", 0), 
		y("y", 0), 
		z("z", 0) {
	registerProperty(x);
	registerProperty(y);
	registerProperty(z);

}

TransformGenerator::~TransformGenerator() {
}

void TransformGenerator::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("transform", &out_transform);
	registerStream("trigger", &trigger);
	// Register handlers
	//registerHandler("step", boost::bind(&TransformGenerator::step, this));
	//addDependency("step", NULL);
	
	registerHandler("onTrigger", boost::bind(&TransformGenerator::step, this));
	addDependency("onTrigger", &trigger);
}

bool TransformGenerator::onInit() {

	return true;
}

bool TransformGenerator::onFinish() {
	return true;
}

bool TransformGenerator::onStop() {
	return true;
}

bool TransformGenerator::onStart() {
	return true;
}

void TransformGenerator::step() {
	trigger.read();
	
	cv::Mat tf = cv::Mat::eye(4, 4, CV_64FC1);
	tf.at<double>(0,3) = x;
	tf.at<double>(1,3) = y;
	tf.at<double>(2,3) = z;
	out_transform.write(tf);
}



} //: namespace TransformGenerator
} //: namespace Processors
