/*!
 * \file
 * \brief 
 * \author Maciej Stefanczyk
 */

#ifndef STITCHER_HPP_
#define STITCHER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/stitching/stitcher.hpp>


namespace Processors {
namespace Stitcher {

/*!
 * \class Stitcher
 * \brief Stitcher processor class.
 *
 * 
 */
class Stitcher: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	Stitcher(const std::string & name = "Stitcher");

	/*!
	 * Destructor
	 */
	virtual ~Stitcher();

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
	Base::DataStreamIn<cv::Mat, Base::DataStreamBuffer::Newest> in_img;
	Base::DataStreamIn<cv::Mat, Base::DataStreamBuffer::Newest> in_depth;

	// Output data streams
	Base::DataStreamOut<cv::Mat> out_img;
	Base::DataStreamOut<cv::Mat> out_depth;

	// Handlers

	// Properties

	
	// Handlers
	void onNewImage();
	void stitch();
	
private:
	std::vector<cv::Mat> m_imgs;

};

} //: namespace Stitcher
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("Stitcher", Processors::Stitcher::Stitcher)

#endif /* STITCHER_HPP_ */
