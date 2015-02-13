/*!
 * \file
 * \brief
 * \author Maciej Stefanczyk
 */

#include <memory>
#include <string>

#include "ModelReader.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>


#include "Utils.hpp"

#include <algorithm>
#include <boost/foreach.hpp>

namespace Processors {
namespace ModelReader {

ModelReader::ModelReader(const std::string & name) :
		Base::Component(name) , 
		path("path", std::string(".")) {
	registerProperty(path);
	
	model_id = 0;

}

ModelReader::~ModelReader() {
}

void ModelReader::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("out_img", &out_img);
	registerStream("out_points_2", &out_points_2);
	registerStream("out_points_3", &out_points_3);
	registerStream("out_model_info", &out_model_info);
	// Register handlers
	registerHandler("step", boost::bind(&ModelReader::step, this));
	addDependency("step", NULL);
	registerHandler("readNextModel", boost::bind(&ModelReader::readNextModel, this));
	registerHandler("readPreviousModel", boost::bind(&ModelReader::readPreviousModel, this));

}

bool ModelReader::onInit() {
	files.clear();

	files = Utils::searchFiles(path, ".*\\.yml");

	CLOG(LINFO) << "Sequence loaded.";
	BOOST_FOREACH(std::string fname, files)
		CLOG(LINFO) << fname;

	return !files.empty();
	return true;
}

bool ModelReader::onFinish() {
	return true;
}

bool ModelReader::onStop() {
	return true;
}

bool ModelReader::onStart() {
	return true;
}

void ModelReader::step() {
}

void ModelReader::readNextModel() {
}

void ModelReader::readPreviousModel() {
}



} //: namespace ModelReader
} //: namespace Processors
