/*!
 * \file
 * \brief 
 * \author Maciej Stefanczyk
 */

#ifndef POI_HPP_
#define POI_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>

#include "Types/CameraInfo.hpp"


namespace Processors {
namespace POI {

/*!
 * \class POI
 * \brief POI processor class.
 *
 * 
 */
class POI: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	POI(const std::string & name = "POI");

	/*!
	 * Destructor
	 */
	virtual ~POI();

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
	Base::DataStreamIn<std::vector< std::vector<cv::Point> > > in_contours;
	Base::DataStreamIn<std::vector< cv::Moments > > in_moments;
	Base::DataStreamIn<Types::CameraInfo> in_camera_info;
	Base::DataStreamIn<cv::Mat> in_transform;
	Base::DataStreamIn<std::vector<float>, Base::DataStreamBuffer::Newest > in_plane;

	// Output data streams
	Base::DataStreamOut<std::vector<cv::Point3f> > out_points;

	// Handlers

	// Properties

	
	// Handlers
	void readPlane();
	void readPoints();
	
	// plane equation: normal and point
	cv::Vec3f n;
	cv::Vec3f p0;
	
	std::vector<cv::Point3f> points;

};

} //: namespace POI
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("POI", Processors::POI::POI)

#endif /* POI_HPP_ */
