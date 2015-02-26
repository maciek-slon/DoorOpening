/*!
 * \file
 * \brief 
 * \author Maciej Stefanczyk
 */

#ifndef PLANEGENERATOR_HPP_
#define PLANEGENERATOR_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>


namespace Processors {
namespace PlaneGenerator {

/*!
 * \class PlaneGenerator
 * \brief PlaneGenerator processor class.
 *
 * 
 */
class PlaneGenerator: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	PlaneGenerator(const std::string & name = "PlaneGenerator");

	/*!
	 * Destructor
	 */
	virtual ~PlaneGenerator();

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

	// Output data streams
	Base::DataStreamOut<cv::Vec6f> out_plane;

	// Handlers

	// Properties
	Base::Property<float> px;
	Base::Property<float> py;
	Base::Property<float> pz;
	Base::Property<float> nx;
	Base::Property<float> ny;
	Base::Property<float> nz;

	
	// Handlers
	void sendPlane();

};

} //: namespace PlaneGenerator
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("PlaneGenerator", Processors::PlaneGenerator::PlaneGenerator)

#endif /* PLANEGENERATOR_HPP_ */
