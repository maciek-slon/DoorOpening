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
	registerStream("trigger", &trigger);
	// Register handlers
	registerHandler("spin", boost::bind(&ROSProxy::spin, this));
	addDependency("spin", NULL);
	
	registerHandler("onNewData", boost::bind(&ROSProxy::onNewData, this));
	addDependency("onNewData", &lockPosition);
	
	registerHandler("onTrigger", boost::bind(&ROSProxy::onTrigger, this));
	addDependency("onTrigger", &trigger);
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
}

void ROSProxy::onNewData() {
//	std_msgs::Int32 msg;
//	msg.data = in_data.read();
//	msg.data = 5;
//	pub.publish(msg);
	
	cv::Vec3d pos = lockPosition.read();
	
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(pos[0], pos[1], pos[2]) );
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/torso_base", "/tmp_lock"));
}

void ROSProxy::callback(const std_msgs::Int32ConstPtr& msg) {
	CLOG(LNOTICE) << "Received from ROS: " << msg->data;
} 

void ROSProxy::onTrigger() {
	trigger.read();
	
	cv::Mat tf_mat = cv::Mat::eye(4,4,CV_64FC1);
	
	try {
		tf::StampedTransform transform;
		listener->lookupTransform("/torso_base", "/head_kinect_rgb_optical_frame", ros::Time(0), transform);
	
		for (int i = 0; i < 3; ++i) {
			tf_mat.at<double>(i, 0) = transform.getBasis()[i][0];
			tf_mat.at<double>(i, 1) = transform.getBasis()[i][1];
			tf_mat.at<double>(i, 2) = transform.getBasis()[i][2];
			tf_mat.at<double>(i, 3) = transform.getOrigin()[i];
		}
		
		CLOG(LINFO) << tf_mat;
	} catch(...) {
		CLOG(LWARNING) << "No tf!";
	}
	
	transform.write(tf_mat);
}



} //: namespace ROSProxy
} //: namespace Processors
