/*!
 * \file
 * \brief
 * \author Maciej Stefanczyk
 */

#include <memory>
#include <string>

#include "Stitcher.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace Stitcher {

Stitcher::Stitcher(const std::string & name) :
		Base::Component(name)  {

}

Stitcher::~Stitcher() {
}

void Stitcher::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_img", &in_img);
	registerStream("in_depth", &in_depth);
	registerStream("out_img", &out_img);
	registerStream("out_depth", &out_depth);
	
	// Register handlers
	registerHandler("onNewImage", boost::bind(&Stitcher::onNewImage, this));
	addDependency("onNewImage", &in_img);
	addDependency("onNewImage", &in_depth);
	
	registerHandler("stitch", boost::bind(&Stitcher::stitch, this));
}

bool Stitcher::onInit() {

	return true;
}

bool Stitcher::onFinish() {
	return true;
}

bool Stitcher::onStop() {
	return true;
}

bool Stitcher::onStart() {
	return true;
}

void Stitcher::onNewImage() {
	cv::Mat img = in_img.read();
	cv::Mat depth = in_depth.read();
	
	m_imgs.push_back(img.clone());
	
	CLOG(LINFO) << "Got new pano frame: " << m_imgs.size();
}

void Stitcher::stitch() {
	cv::Stitcher st = cv::Stitcher::createDefault();
	st.setWaveCorrectKind(cv::detail::WAVE_CORRECT_VERT);
	st.setWaveCorrection(false);
	st.setWarper(cv::Ptr<cv::PlaneWarper>(new cv::PlaneWarper));
	
	CLOG(LINFO) << "Stitching " << m_imgs.size() << " images...";
	
	cv::Mat out;
	cv::Stitcher::Status ret = st.stitch(m_imgs, out);
	if (ret == cv::Stitcher::OK) {
		out_img.write(out);
		CLOG(LINFO) << "Stitching done.";
	}	else {
		CLOG(LERROR) << "Can't stitch images, error code = " << int(ret);
	}
	
	m_imgs.clear();
}



} //: namespace Stitcher
} //: namespace Processors
