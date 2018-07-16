#pragma once

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <dlib/optimization.h>
#include "Util.h"
#include <QString>

static int seq;

class ShapeFit {
		
	typedef dlib::matrix<double, 0, 1> column_vector;

	class BFGSSolver {
	private:
		std::vector<cv::Point2f> target_polygon;

	public:
		BFGSSolver(const std::vector<cv::Point2f>& target_polygon) {
			this->target_polygon = target_polygon;
			seq = 0;
		}

		double operator() (const column_vector& arg) const {
			std::vector<cv::Point2f> polygon;
			for (int i = 0; i < arg.size() / 2; i++) {
				polygon.push_back(cv::Point2f(arg(i * 2), arg(i * 2 + 1)));
			}

			///////////////// DEBUG //////////////////
			/*
			if (seq % 100 == 0) {
				cv::Mat img(600, 600, CV_8UC3, cv::Scalar(255, 255, 255));
				std::vector<std::vector<cv::Point>> polygons(1, std::vector<cv::Point>(target_polygon.size()));
				for (int i = 0; i < target_polygon.size(); i++) {
					polygons[0][i] = cv::Point(target_polygon[i].x * 540 + 30, target_polygon[i].y * 540 + 30);
				}
				cv::polylines(img, polygons, true, cv::Scalar(0, 0, 0), 1);
				std::vector<std::vector<cv::Point>> shapes(1, std::vector<cv::Point>(polygon.size()));
				for (int i = 0; i < polygon.size(); i++) {
					shapes[0][i] = cv::Point(polygon[i].x * 540 + 30, polygon[i].y * 540 + 30);
				}
				cv::polylines(img, shapes, true, cv::Scalar(255, 0, 0), 3);
				cv::imwrite(QString("result_%1.png").arg(seq).toUtf8().constData(), img);
			}
			seq++;
			*/

			try {
				//return calculateIOU(polygon, target_polygon);
				//return calculateIOUbyImage(polygon, target_polygon, 1000);
				return calculatePoLIS(polygon, target_polygon);
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
	static std::vector<cv::Point2f> fit(const std::vector<cv::Point2f>& target_polygon, const std::vector<cv::Point2f>& initial_polygon);
};

