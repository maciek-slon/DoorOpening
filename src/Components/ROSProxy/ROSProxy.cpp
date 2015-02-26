/*!
 * \file
 * \brief
 * \author Maciej Stefanczyk
 */

#include <memory>
#include <string>

#include "ROSProxy.hpp"
#include "Common/Logger.hpp"

#include "geometry_msgs/PoseStamped.h"

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
	registerStream("transform2", &transform2);
	registerStream("trigger", &trigger);
	registerStream("trigger2", &trigger2);
	// Register handlers
	registerHandler("spin", boost::bind(&ROSProxy::spin, this));
	addDependency("spin", NULL);
	
	registerHandler("onNewData", boost::bind(&ROSProxy::onNewData, this));
	addDependency("onNewData", &lockPosition);
	
	registerHandler("onTrigger", boost::bind(&ROSProxy::onTrigger, this));
	addDependency("onTrigger", &trigger);
	
	registerHandler("onTrigger2", boost::bind(&ROSProxy::onTrigger2, this));
	addDependency("onTrigger2", &trigger2);
}

bool ROSProxy::onInit() {
	static char * tmp = NULL;
	static int tmpi;
	ros::init(tmpi, &tmp, std::string("discode"), ros::init_options::NoSigintHandler);
	nh = new ros::NodeHandle;
	listener = new tf::TransformListener;
	pub = nh->advertise<geometry_msgs::PoseStamped>("poi_poses", 1000);
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
	std::vector<cv::Vec6f> points = lockPosition.read();
	
	static tf::TransformBroadcaster br;
	for (int i = 0; i < points.size(); ++i) {
		tf::Transform transform, transform2, transform3;
		transform.setOrigin( tf::Vector3(points[i][0], points[i][1], points[i][2]) );
		
		cv::Vec3f n;
		n[0] = points[i][3];
		n[1] = points[i][4];
		n[2] = points[i][5];
		
		transform2.setOrigin( tf::Vector3(-0.7, 0, 0) );
		
		cv::Vec3f x;
		x[0] = 1;
		x[1] = 0;
		x[2] = 0;
		
		cv::Vec3f w = x.cross(n);
		
		tf::Quaternion q(w[0], w[1], w[2], 1.f + x.dot(n));
		q.normalize();
		
		transform.setRotation(q);
		transform2.setRotation(tf::Quaternion(tf::Vector3(0, 0, 1), 3.1415));
		std::string name = "/tmp_lock_";
		name += ('0'+i);
		std::string name2 = "/look_lock_";
		name2 += ('0'+i);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/torso_base", name));
		//br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), name, name2));
		
		transform3 = transform * transform2;
		br.sendTransform(tf::StampedTransform(transform3, ros::Time::now(), "/torso_base", name2));
		
		geometry_msgs::PoseStamped pose;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "/torso_base";
		tf::Vector3 origin = transform3.getOrigin();
		tf::Quaternion quaternion = transform3.getRotation();
		pose.pose.position.x = origin.x();
		pose.pose.position.y = origin.y();
		pose.pose.position.z = origin.z() + 0.05;
		pose.pose.orientation.x = quaternion.x();
		pose.pose.orientation.y = quaternion.y();
		pose.pose.orientation.z = quaternion.z();
		pose.pose.orientation.w = quaternion.w();
		
		pub.publish(pose);
		
		CLOG(LNOTICE) << "Sending transform " << points[i];
	}
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

void ROSProxy::onTrigger2() {
	trigger2.read();
	
	cv::Mat tf_mat = cv::Mat::eye(4,4,CV_64FC1);
	
	try {
		tf::StampedTransform transform;
		listener->lookupTransform("/torso_base", "/left_hand_camera_optical_frame", ros::Time(0), transform);
	
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
	
	transform2.write(tf_mat);
}



} //: namespace ROSProxy
} //: namespace Processors
