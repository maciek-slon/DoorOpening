/*!
 * \file
 * \brief
 * \author Maciej Stefanczyk
 */

#include <memory>
#include <string>

#include "ROSProxy.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace ROSProxy {

ROSProxy::ROSProxy(const std::string & name) :
		Base::Component(name)  {

}

ROSProxy::~ROSProxy() {
}

void ROSProxy::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("lockPosition", &lockPosition);
	registerStream("transform", &transform);
	// Register handlers
	registerHandler("spin", boost::bind(&ROSProxy::spin, this));
	addDependency("spin", NULL);
	
	registerHandler("onNewData", boost::bind(&ROSProxy::onNewData, this));
	addDependency("onNewData", &lockPosition);

}

bool ROSProxy::onInit() {
	static char * tmp = NULL;
	static int tmpi;
	ros::init(tmpi, &tmp, std::string("discode"), ros::init_options::NoSigintHandler);
	nh = new ros::NodeHandle;
	listener = new tf::TransformListener;
	pub = nh->advertise<std_msgs::Int32>("from_discode", 1000);
	sub = nh->subscribe("my_topic", 1, &ROSProxy::callback, this);
	return true;
}

bool ROSProxy::onFinish() {
	delete nh;
	return true;
}

bool ROSProxy::onStop() {
	return true;
}

bool ROSProxy::onStart() {
	return true;
}

void ROSProxy::spin() {
	ros::spinOnce();
	
	try {
		tf::StampedTransform transform;
		listener->lookupTransform("/head_kinect_rgb_optical_frame", "/world", ros::Time(0), transform);
	
		CLOG(LNOTICE) << "Transform: " << transform.getOrigin()[0];
	} catch(...) {}
}

void ROSProxy::onNewData() {
	std_msgs::Int32 msg;
//	msg.data = in_data.read();
	msg.data = 5;
	pub.publish(msg);
}

void ROSProxy::callback(const std_msgs::Int32ConstPtr& msg) {
	CLOG(LNOTICE) << "Received from ROS: " << msg->data;
	
	
} 



} //: namespace ROSProxy
} //: namespace Processors
