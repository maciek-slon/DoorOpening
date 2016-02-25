/*!
 * \file
 * \brief
 * \author maciej,,,
 */

#include <memory>
#include <string>

#include "DoorAngleDetector.hpp"
#include "Common/Logger.hpp"

#include "Types/Line.hpp"

#include <boost/bind.hpp>


namespace Processors {
namespace DoorAngleDetector {

DoorAngleDetector::DoorAngleDetector(const std::string & name) :
		Base::Component(name),
		prop_width("width", 640),
		prop_height("height", 480) {
			registerProperty(prop_width);
			registerProperty(prop_height);

}

DoorAngleDetector::~DoorAngleDetector() {
}

void DoorAngleDetector::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_lines", &in_lines);
	registerStream("out_lines", &out_lines);
	registerStream("out_points", &out_points);
	registerStream("out_mask", &out_mask);
	
	// Register handlers
	registerHandler("onNewLines", boost::bind(&DoorAngleDetector::onNewLines, this));
	addDependency("onNewLines", &in_lines);

}

bool DoorAngleDetector::onInit() {

	return true;
}

bool DoorAngleDetector::onFinish() {
	return true;
}

bool DoorAngleDetector::onStop() {
	return true;
}

bool DoorAngleDetector::onStart() {
	return true;
}

void DoorAngleDetector::onNewLines() {
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

	/*for (int i = 0; i < 20; ++i) {
		std::random_shuffle(lines_v.begin(), lines_v.end());
	}*/

	// center line - vertical door frame
	Types::Line * cl = NULL;
	
	// score of center line
	float cl_score = 0;
	
	for (int i = 0; i < lines_v.size(); ++i) {
		Types::Line * tmp = lines_v[i];
		
		// angle score - 1 for perfectly vertical line
		float angle_score = abs(tmp->getAngle()) / (M_PI/2);
		
		// position score - 1 for centered line
		float mean_x = (tmp->getP1().x + tmp->getP2().x) / 2;
		float cx = 0.5 * prop_width;
		float position_score = 1.0 - fabs(cx - mean_x) / cx;
		
		// length score - 1 for line at least half of image height
		float length_score = 2 * tmp->length() / prop_height;
		if (length_score > 1) length_score = 1;
		
		float tmp_score = angle_score * position_score * length_score;
		if (tmp_score > cl_score) {
			cl = tmp;
			cl_score = tmp_score;
		}
	}
	
	if (cl)
		cl->setCol(cv::Scalar(255, 0, 255));
		
	// left line - left floor/wall crossing
	Types::Line * ll = NULL;
	
	// score of left line
	float ll_score = 0;
	
	for (int i = 0; i < lines_h.size(); ++i) {
		Types::Line * tmp = lines_h[i];
		
		// angle score - 1 for perfectly horizontal line
		float angle_score = (M_PI/2 - abs(tmp->getAngle())) / (M_PI/2);
		
		// position score - 1 for line centered on the left part
		float mean_x = (tmp->getP1().x + tmp->getP2().x) / 2;
		float cx = 0.25 * prop_width;
		float position_score = 1.0 - fabs(cx - mean_x) / cx;
		
		// length score - 1 for line at least half of image width
		float length_score = 2 * tmp->length() / prop_width;
		if (length_score > 1) length_score = 1;
		
		float tmp_score = angle_score * position_score * length_score;
		if (tmp_score > ll_score) {
			ll = tmp;
			ll_score = tmp_score;
		}
	}
	
	if (ll)
		ll->setCol(cv::Scalar(0, 0, 255));
		
	// left line - left floor/wall crossing
	Types::Line * rl = NULL;
	
	// score of left line
	float rl_score = 0;
	
	for (int i = 0; i < lines_h.size(); ++i) {
		Types::Line * tmp = lines_h[i];
		
		// angle score - 1 for perfectly horizontal line
		float angle_score = (M_PI/2 - abs(tmp->getAngle())) / (M_PI/2);
		
		// position score - 1 for line centered on the left part
		float mean_x = (tmp->getP1().x + tmp->getP2().x) / 2;
		float cx = 0.75 * prop_width;
		float position_score = 1.0 - fabs(cx - mean_x) / cx;
		
		// length score - 1 for line at least half of image width
		float length_score = 2 * tmp->length() / prop_width;
		if (length_score > 1) length_score = 1;
		
		float tmp_score = angle_score * position_score * length_score;
		if (tmp_score > rl_score) {
			rl = tmp;
			rl_score = tmp_score;
		}
	}
	
	if (rl)
		rl->setCol(cv::Scalar(0, 0, 255));
		
	
	Types::DrawableContainer mask;
	cv::Mat mask_img = cv::Mat::zeros(480, 640, CV_8UC1);
	
	float angle = 0;
	if (ll && rl) {
		angle = fabs(ll->getAngle() - rl->getAngle());
	} else {
		angle = 90;
	}
	
	CLOG(LNOTICE) << "Angle: " << angle;
	
	out_lines.write(lines);
	out_mask.write(mask_img);

	
	
}



} //: namespace DoorAngleDetector
} //: namespace Processors
