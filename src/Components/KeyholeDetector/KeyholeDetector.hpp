/*!
 * \file
 * \brief 
 * \author Maciej Stefanczyk
 */

#ifndef KEYHOLEDETECTOR_HPP_
#define KEYHOLEDETECTOR_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>

#include "Types/CameraInfo.hpp"


namespace Processors {
namespace KeyholeDetector {

/*!
 * \class KeyholeDetector
 * \brief KeyholeDetector processor class.
 *
 * 
 */
class KeyholeDetector: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	KeyholeDetector(const std::string & name = "KeyholeDetector");

	/*!
	 * Destructor
	 */
	virtual ~KeyholeDetector();

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
	Base::DataStreamIn<cv::Mat, Base::DataStreamBuffer::Newest> in_homography;
	Base::DataStreamIn<cv::Point2f> in_point;
	Base::DataStreamIn<cv::Vec6f, Base::DataStreamBuffer::Newest> in_plane;
	Base::DataStreamIn<cv::Mat, Base::DataStreamBuffer::Newest> in_transform;
	Base::DataStreamIn<Types::CameraInfo, Base::DataStreamBuffer::Newest> in_camera_info;

	// Output data streams
	Base::DataStreamOut<cv::Point3f> out_point;

	// Handlers

	// Properties

	
	// Handlers
	void onNewPoint();
	void onNewData();
	void readPlane();
	
	cv::Point2f point;
	cv::Vec3f n0, n;
	
};

} //: namespace KeyholeDetector
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("KeyholeDetector", Processors::KeyholeDetector::KeyholeDetector)

#endif /* KEYHOLEDETECTOR_HPP_ */
