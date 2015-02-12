/*!
 * \file
 * \brief 
 * \author Maciej Stefanczyk
 */

#ifndef TRANSFORMGENERATOR_HPP_
#define TRANSFORMGENERATOR_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>


namespace Processors {
namespace TransformGenerator {

/*!
 * \class TransformGenerator
 * \brief TransformGenerator processor class.
 *
 * 
 */
class TransformGenerator: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	TransformGenerator(const std::string & name = "TransformGenerator");

	/*!
	 * Destructor
	 */
	virtual ~TransformGenerator();

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
	Base::DataStreamIn<Base::UnitType> trigger;

	// Output data streams
	Base::DataStreamOut<cv::Mat> out_transform;

	// Handlers

	// Properties
	Base::Property<float> x;
	Base::Property<float> y;
	Base::Property<float> z;

	
	// Handlers
	void step();

};

} //: namespace TransformGenerator
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("TransformGenerator", Processors::TransformGenerator::TransformGenerator)

#endif /* TRANSFORMGENERATOR_HPP_ */
