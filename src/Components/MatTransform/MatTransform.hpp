/*!
 * \file
 * \brief 
 * \author Maciej Stefanczyk
 */

#ifndef MATTRANSFORM_HPP_
#define MATTRANSFORM_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>


namespace Processors {
namespace MatTransform {

/*!
 * \class MatTransform
 * \brief MatTransform processor class.
 *
 * 
 */
class MatTransform: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	MatTransform(const std::string & name = "MatTransform");

	/*!
	 * Destructor
	 */
	virtual ~MatTransform();

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
	Base::DataStreamIn<cv::Mat> in_img;
	Base::DataStreamIn<cv::Mat, Base::DataStreamBuffer::Newest> in_transform;
	Base::DataStreamIn<cv::Point2f, Base::DataStreamBuffer::Newest> in_point;

	// Output data streams
	Base::DataStreamOut<cv::Mat> out_img;
	Base::DataStreamOut<cv::Vec3d> lockPosition;

	// Handlers

	// Properties
	
	// Handlers
	void onNewImage();

};

} //: namespace MatTransform
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("MatTransform", Processors::MatTransform::MatTransform)

#endif /* MATTRANSFORM_HPP_ */
