/*
 * mapUpdate.hpp
 *
 *  Created on: Dec 12, 2014
 *      Author: fpatzelt <fpatzelt@techfak.uni-bielefeld.de>
 */

#ifndef MAPUPDATE_HPP_
#define MAPUPDATE_HPP_

#include <opencv2/core/core.hpp>

using namespace cv;

// simple wrapper class that contains a cv::Mat and a pose.
class MapUpdate: public Mat {
	// use constructors from cv::Mat
	//using Mat::Mat; compiler in twb is to old for this...

public:
	MapUpdate() :
			Mat(), x(0), y(0), theta(0) {
	}
	;
	MapUpdate(Size size, int type) :
			Mat(size, type), x(0), y(0), theta(0) {
	}
	;
	MapUpdate(Size size, int type, const Scalar& s) :
			Mat(size, type, s), x(0), y(0), theta(0) {
	}
	;
	MapUpdate(Size size, int type, void* data, size_t step = AUTO_STEP) :
			Mat(size, type, data, step), x(0), y(0), theta(0) {
	}
	;
	MapUpdate clone() {
		MapUpdate m;
		copyTo(m);
		m.x = x;
		m.y = y;
		m.theta = theta;
		return m;
	}
	float x;
	float y;
	float theta;

};

#endif /* MAPUPDATE_HPP_ */
