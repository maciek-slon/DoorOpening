/*!
 * \file
 * \brief 
 * \author Maciej Stefanczyk
 */

#ifndef WARPER_HPP_
#define WARPER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include "Types/CameraInfo.hpp"

#include <opencv2/opencv.hpp>


namespace Processors {
namespace Warper {

/*!
 * \class Warper
 * \brief Warper processor class.
 *
 * 
 */
class Warper: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	Warper(const std::string & name = "Warper");

	/*!
	 * Destructor
	 */
	virtual ~Warper();

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
	Base::DataStreamIn<cv::Mat> in_transform;
	Base::DataStreamIn<Types::CameraInfo> in_camerainfo;

	// Output data streams
	Base::DataStreamOut<cv::Mat> out_img;

	// Handlers

	// Properties

	
	// Handlers
	void onNewImage();

};

} //: namespace Warper
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("Warper", Processors::Warper::Warper)

#endif /* WARPER_HPP_ */
