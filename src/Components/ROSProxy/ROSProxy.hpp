/*!
 * \file
 * \brief 
 * \author Maciej Stefanczyk
 */

#ifndef ROSPROXY_HPP_
#define ROSPROXY_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

#include <opencv2/opencv.hpp>

namespace Processors {
namespace ROSProxy {

/*!
 * \class ROSProxy
 * \brief ROSProxy processor class.
 *
 * 
 */
class ROSProxy: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	ROSProxy(const std::string & name = "ROSProxy");

	/*!
	 * Destructor
	 */
	virtual ~ROSProxy();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();


	// Input data streams
	Base::DataStreamIn<Base::UnitType, Base::DataStreamBuffer::Newest> trigger;
	Base::DataStreamIn<Base::UnitType, Base::DataStreamBuffer::Newest> trigger2;
	Base::DataStreamIn<std::vector<cv::Vec6f> > lockPosition;

	// Output data streams
	Base::DataStreamOut<cv::Mat> transform;
	Base::DataStreamOut<cv::Mat> transform2;

	// Handlers

	// Properties

	
	// Handlers
	void spin();
	void onNewData();
	void onTrigger();
	void onTrigger2();
	
	ros::Publisher pub;
	ros::Subscriber sub;
	ros::NodeHandle * nh;
	
	tf::TransformListener * listener;

	void callback(const std_msgs::Int32ConstPtr& msg);

};

} //: namespace ROSProxy
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("ROSProxy", Processors::ROSProxy::ROSProxy)

#endif /* ROSPROXY_HPP_ */
