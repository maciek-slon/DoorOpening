/*!
 * \file
 * \brief 
 * \author maciej,,,
 */

#ifndef DOORANGLEDETECTOR_HPP_
#define DOORANGLEDETECTOR_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>

#include "Types/DrawableContainer.hpp"

namespace Processors {
namespace DoorAngleDetector {

/*!
 * \class DoorAngleDetector
 * \brief DoorAngleDetector processor class.
 *
 * DoorAngleDetector processor.
 */
class DoorAngleDetector: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	DoorAngleDetector(const std::string & name = "DoorAngleDetector");

	/*!
	 * Destructor
	 */
	virtual ~DoorAngleDetector();

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

	Base::DataStreamIn<Types::DrawableContainer> in_lines;

	// Output data streams

	Base::DataStreamOut<Types::DrawableContainer> out_lines;
	Base::DataStreamOut<Types::DrawableContainer> out_points;
	Base::DataStreamOut<cv::Mat> out_mask;
	// Handlers
	Base::EventHandler2 h_onNewLines;

	
	// Handlers
	void onNewLines();
	
	Base::Property<int> prop_width;
	Base::Property<int> prop_height;

};

} //: namespace DoorAngleDetector
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("DoorAngleDetector", Processors::DoorAngleDetector::DoorAngleDetector)

#endif /* DOORANGLEDETECTOR_HPP_ */
