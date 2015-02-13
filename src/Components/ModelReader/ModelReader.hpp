/*!
 * \file
 * \brief 
 * \author Maciej Stefanczyk
 */

#ifndef MODELREADER_HPP_
#define MODELREADER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>

#include "Types/DrawableContainer.hpp"


namespace Processors {
namespace ModelReader {

/*!
 * \class ModelReader
 * \brief ModelReader processor class.
 *
 * 
 */
class ModelReader: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	ModelReader(const std::string & name = "ModelReader");

	/*!
	 * Destructor
	 */
	virtual ~ModelReader();

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
	Base::DataStreamOut<cv::Mat> out_img;
	Base::DataStreamOut<std::vector<cv::Point2f> > out_points_2;
	Base::DataStreamOut<std::vector<cv::Point3f> > out_points_3;
	Base::DataStreamOut<Types::DrawableContainer> out_model_info;

	// Handlers

	// Properties
	Base::Property<std::string> path;

	
	// Handlers
	void step();
	void readNextModel();
	void readPreviousModel();

	std::vector<std::string> files;
	
	int model_id;
};

} //: namespace ModelReader
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("ModelReader", Processors::ModelReader::ModelReader)

#endif /* MODELREADER_HPP_ */
