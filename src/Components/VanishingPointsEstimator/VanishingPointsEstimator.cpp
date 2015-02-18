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
	registerStream("out_mask", &out_mask);
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
			line->setCol(cv::Scalar(0, 0, 128));
			lines_h.push_back(line);
		} else
		if (abs(line->getAngle()) > 2*M_PI/6) {
			line->setCol(cv::Scalar(0, 128, 0));
			lines_v.push_back(line);
		} else {
			line->setCol(cv::Scalar(128, 0, 0));
		}
	}

	std::vector<cv::Point2f> points_v, points_h;

	for (int i = 0; i < 20; ++i) {
		std::random_shuffle(lines_v.begin(), lines_v.end());

	}

	Types::Line * ll = NULL, * rr = NULL;
	for (int i = 0; i < lines_v.size(); ++i) {
		// if line is on the left side of the image
		if (lines_v[i]->getP1().x < 320) {
			if (!ll) ll = lines_v[i];
			if (lines_v[i]->length() > ll->length()) ll = lines_v[i];
		}
		
		// if line is on the right side of the image
		if (lines_v[i]->getP1().x > 320) {
			if (!rr) rr = lines_v[i];
			if (lines_v[i]->length() > rr->length()) rr = lines_v[i];
		}
	}
	
	Types::DrawableContainer mask;
	cv::Mat mask_img = cv::Mat::zeros(480, 640, CV_8UC1);
	if (ll && rr) {
		ll->setCol(cv::Scalar(255, 0, 255));
		rr->setCol(cv::Scalar(255, 0, 255));
		Types::Line top(cv::Point(0, 50), cv::Point(100, 50));
		Types::Line bot(cv::Point(0, 430), cv::Point(100, 430));
		
		cv::Point p1 = top.intersect(ll);
		cv::Point p2 = top.intersect(rr);
		cv::Point p3 = bot.intersect(rr);
		cv::Point p4 = bot.intersect(ll);
		
		mask.add(new Types::Line(p1, p2));
		mask.add(new Types::Line(p2, p3));
		mask.add(new Types::Line(p3, p4));
		mask.add(new Types::Line(p4, p1));
		
		cv::Point mask_points[4];
		mask_points[0] = p1;
		mask_points[1] = p2;
		mask_points[2] = p3;
		mask_points[3] = p4;
		
		const cv::Point * mask_points_2[1] = { mask_points };
		
		int npts[1];
		npts[0] = 4;

		cv::fillPoly(mask_img, mask_points_2, npts, 1, cv::Scalar(255));
		
		out_lines.write(mask);
		out_mask.write(mask_img);
	} else {
		out_lines.write(lines);
		out_mask.write(mask_img);
	}

	
	
}



} //: namespace VanishingPointsEstimator
} //: namespace Processors
