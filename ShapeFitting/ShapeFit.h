#pragma once

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <dlib/optimization.h>
#include "Util.h"

class ShapeFit {
		
	typedef dlib::matrix<double, 0, 1> column_vector;

	class BFGSSolver {
	private:
		std::vector<cv::Point2f> target_polygon;

	public:
		BFGSSolver(const std::vector<cv::Point2f>& target_polygon) {
			this->target_polygon = target_polygon;
		}

		double operator() (const column_vector& arg) const {
			std::vector<cv::Point2f> polygon;
			for (int i = 0; i < 4; i++) {
				polygon.push_back(cv::Point2f(arg(i * 2), arg(i * 2 + 1)));
			}

			try {
				return calculateIOU(polygon, target_polygon);
			}
			catch (...) {
				return 0;
			}
		}
	};

protected:
	ShapeFit();
	~ShapeFit();

public:
	static std::vector<cv::Point2f> fit(const std::vector<cv::Point2f>& target_polygon);
};

