/*!
 * \file
 * \brief
 * \author Maciej Stefanczyk
 */

#include <memory>
#include <string>

#include "PlaneGenerator.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace PlaneGenerator {

PlaneGenerator::PlaneGenerator(const std::string & name) :
		Base::Component(name) , 
		px("px", 0), 
		py("py", 0), 
		pz("pz", 0), 
		nx("nx", 0), 
		ny("ny", 0), 
		nz("nz", 1) {
	registerProperty(px);
	registerProperty(py);
	registerProperty(pz);
	registerProperty(nx);
	registerProperty(ny);
	registerProperty(nz);

}

PlaneGenerator::~PlaneGenerator() {
}

void PlaneGenerator::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("out_plane", &out_plane);
	// Register handlers
	registerHandler("sendPlane", boost::bind(&PlaneGenerator::sendPlane, this));

}

bool PlaneGenerator::onInit() {

	return true;
}

bool PlaneGenerator::onFinish() {
	return true;
}

bool PlaneGenerator::onStop() {
	return true;
}

bool PlaneGenerator::onStart() {
	return true;
}

void PlaneGenerator::sendPlane() {
	cv::Vec6f plane;
	plane[0] = px;
	plane[1] = py;
	plane[2] = pz;
	plane[3] = nx;
	plane[4] = ny;
	plane[5] = nz;
	out_plane.write(plane);
}



} //: namespace PlaneGenerator
} //: namespace Processors
