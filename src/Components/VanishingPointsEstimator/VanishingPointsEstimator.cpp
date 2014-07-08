/*!
 * \file
 * \brief
 * \author maciej,,,
 */

#include <memory>
#include <string>

#include "VanishingPointsEstimator.hpp"
#include "Common/Logger.hpp"

#include "Types/Line.hpp"

#include <boost/bind.hpp>


namespace Processors {
namespace VanishingPointsEstimator {

VanishingPointsEstimator::VanishingPointsEstimator(const std::string & name) :
		Base::Component(name)  {

}

VanishingPointsEstimator::~VanishingPointsEstimator() {
}

void VanishingPointsEstimator::prepareInterface() {
	// Register data streams, events and event handlers HERE!
registerStream("in_lines", &in_lines);
registerStream("out_lines", &out_lines);
registerStream("out_points", &out_points);
	// Register handlers
	h_onNewLines.setup(boost::bind(&VanishingPointsEstimator::onNewLines, this));
	registerHandler("onNewLines", &h_onNewLines);
	addDependency("onNewLines", &in_lines);

}

bool VanishingPointsEstimator::onInit() {

	return true;
}

bool VanishingPointsEstimator::onFinish() {
	return true;
}

bool VanishingPointsEstimator::onStop() {
	return true;
}

bool VanishingPointsEstimator::onStart() {
	return true;
}

void VanishingPointsEstimator::onNewLines() {
	Types::DrawableContainer lines = in_lines.read();
	Types::Line * line;

	std::vector<Types::Line*> lines_v, lines_h;

	for (int i = 0; i < lines.size(); ++i) {
		line = static_cast<Types::Line*>(lines.get(i));
		if (abs(line->getAngle()) < M_PI/6) {
			line->setCol(cv::Scalar(0, 0, 255));
			lines_h.push_back(line);
		} else
		if (abs(line->getAngle()) > 2*M_PI/6) {
			line->setCol(cv::Scalar(0, 255, 0));
			lines_v.push_back(line);
		} else {
			line->setCol(cv::Scalar(255, 0, 0));
		}
	}

	std::vector<cv::Point2f> points_v, points_h;

	for (int i = 0; i < 20; ++i) {
		std::random_shuffle(lines_v.begin(), lines_v.end());

	}

	out_lines.write(lines);
}



} //: namespace VanishingPointsEstimator
} //: namespace Processors
