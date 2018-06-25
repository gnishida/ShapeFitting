#include "Util.h"

/**
 * Find the contour polygons from the image.
 */
std::vector<std::vector<cv::Point2f>> findContours(const cv::Mat& image, int threshold, bool simplify) {
	std::vector<std::vector<cv::Point2f>> polygons;

	cv::Mat mat = image.clone();
	cv::threshold(mat, mat, threshold, 255, cv::THRESH_BINARY);

	// extract contours
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	if (simplify) {
		cv::findContours(mat, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	}
	else {
		cv::findContours(mat, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
	}

	for (int i = 0; i < hierarchy.size(); i++) {
		if (hierarchy[i][3] != -1) continue;
		if (contours[i].size() < 3) continue;

		std::vector<cv::Point2f> polygon;
		polygon.resize(contours[i].size());
		for (int j = 0; j < contours[i].size(); j++) {
			polygon[j] = cv::Point2f(contours[i][j].x, contours[i][j].y);
		}

		if (polygon.size() >= 3) {
			// obtain all the holes inside this contour
			int hole_id = hierarchy[i][2];
			while (hole_id != -1) {
				// ignore holes in this implementation for simplicity

				hole_id = hierarchy[hole_id][0];
			}

			polygons.push_back(polygon);
		}
	}

	return polygons;
}

/**
 * return angle difference between two angles in range [0, PI/2].
 */
float angle_difference(float x, float xi) {
	x = regularize_angle_PI(x);
	xi = regularize_angle_PI(xi);

	if (std::abs(x - xi) <= CV_PI * 0.5) return std::abs(x - xi);
	else return CV_PI - std::abs(x - xi);
}

/**
 * return the normalized angle in range [0, PI].
 */
float regularize_angle_PI(float x) {
	if (x < 0) {
		x += CV_PI * (int)(-x / CV_PI + 1);
	}
	else {
		x -= CV_PI * (int)(x / CV_PI);
	}

	return x;
}

/**
 * Calculate the intersection of line a-b and line c-d.
 *
 * @param a				the first end point of the first line
 * @param b				the second end point of the first line
 * @param c				the first end point of the second line
 * @param d				the second end point of the second line
 * @param tab			the position of the intersecion on the first line (0 means point a, whereas 1 means point b).
 * @param tcd			the position of the intersecion on the second line (0 means point c, whereas 1 means point d).
 * @param segment_only	if true, find the intersection only on the line segments.
 * @param int_pt		intersecion point
 */
bool lineLineIntersection(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c, const cv::Point2f& d, double *tab, double *tcd, bool segment_only, cv::Point2f& int_pt) {
	cv::Point2f u = b - a;
	cv::Point2f v = d - c;

	if (cv::norm(u) < 0.0000001 || cv::norm(v) < 0.0000001) {
		return false;
	}

	double numer = v.x * (c.y - a.y) + v.y * (a.x - c.x);
	double denom = u.y * v.x - u.x * v.y;

	if (denom == 0.0)  {
		// they are parallel
		return false;
	}

	double t0 = numer / denom;

	cv::Point2f ipt = a + t0*u;
	cv::Point2f tmp = ipt - c;
	double t1;
	if (tmp.dot(v) > 0.0) {
		t1 = cv::norm(tmp) / cv::norm(v);
	}
	else {
		t1 = -1.0 * cv::norm(tmp) / cv::norm(d - c);
	}

	// check if intersection is within the segments
	if (segment_only && !((t0 >= 0.0000001) && (t0 <= 1.0 - 0.0000001) && (t1 >= 0.0000001) && (t1 <= 1.0 - 0.0000001))) {
		return false;
	}

	*tab = t0;
	*tcd = t1;
	cv::Point2f dirVec = b - a;

	int_pt = a + t0 * dirVec;

	return true;
}

float calculateIOU(const std::vector<cv::Point2f>& polygon1, const std::vector<cv::Point2f>& polygon2) {
	CGAL::Polygon_2<Kernel> pol1;
	for (auto& pt : polygon1) {
		pol1.push_back(Kernel::Point_2(pt.x, pt.y));
	}
	if (!pol1.is_simple()) {
		throw "calculateIOU: polygon1 is not simple.";
	}
	if (pol1.is_clockwise_oriented()) pol1.reverse_orientation();

	CGAL::Polygon_2<Kernel> pol2;
	for (auto& pt : polygon2) {
		pol2.push_back(Kernel::Point_2(pt.x, pt.y));
	}
	if (!pol2.is_simple()) {
		throw "calculateIOU: polygon2 is not simple.";
	}
	if (pol2.is_clockwise_oriented()) pol2.reverse_orientation();

	if (CGAL::do_intersect(pol1, pol2)) {
		std::list<CGAL::Polygon_with_holes_2<Kernel>> inter;
		CGAL::intersection(pol1, pol2, std::back_inserter(inter));

		CGAL::Polygon_with_holes_2<Kernel> uni;
		CGAL::join(pol1, pol2, uni);

		float inter_area = 0;
		for (auto it = inter.begin(); it != inter.end(); it++) {
			inter_area += area(*it);
		}

		return inter_area / area(uni);
	}
	else {
		return 0;
	}
}

float area(const CGAL::Polygon_with_holes_2<Kernel>& pwh) {
	float ans = 0;

	ans += CGAL::to_double(pwh.outer_boundary().area());
	for (auto it = pwh.holes_begin(); it != pwh.holes_end(); it++) {
		ans -= CGAL::to_double(it->area());
	}

	return ans;
}

float calculateImageBasedIOU(const std::vector<cv::Point2f>& polygon1, const std::vector<cv::Point2f>& polygon2) {
	int min_x = INT_MAX;
	int min_y = INT_MAX;
	int max_x = INT_MIN;
	int max_y = INT_MIN;
	for (int i = 0; i < polygon1.size(); i++) {
		min_x = std::min(min_x, (int)polygon1[i].x);
		min_y = std::min(min_y, (int)polygon1[i].y);
		max_x = std::max(max_x, (int)(polygon1[i].x + 0.5));
		max_y = std::max(max_y, (int)(polygon1[i].y + 0.5));
	}
	for (int i = 0; i < polygon2.size(); i++) {
		min_x = std::min(min_x, (int)polygon2[i].x);
		min_y = std::min(min_y, (int)polygon2[i].y);
		max_x = std::max(max_x, (int)(polygon2[i].x + 0.5));
		max_y = std::max(max_y, (int)(polygon2[i].y + 0.5));
	}

	cv::Mat_<uchar> img1 = cv::Mat_<uchar>::zeros(max_y - min_y + 1, max_x - min_x + 1);

	std::vector<std::vector<cv::Point>> contour_points1(1);
	contour_points1[0].resize(polygon1.size());
	for (int i = 0; i < polygon1.size(); i++) {
		contour_points1[0][i] = cv::Point(polygon1[i].x - min_x, polygon1[i].y - min_y);
	}
	cv::fillPoly(img1, contour_points1, cv::Scalar(255), cv::LINE_4);

	cv::Mat_<uchar> img2 = cv::Mat_<uchar>::zeros(max_y - min_y + 1, max_x - min_x + 1);

	std::vector<std::vector<cv::Point>> contour_points2(1);
	contour_points2[0].resize(polygon2.size());
	for (int i = 0; i < polygon2.size(); i++) {
		contour_points2[0][i] = cv::Point(polygon2[i].x - min_x, polygon2[i].y - min_y);
	}
	cv::fillPoly(img2, contour_points2, cv::Scalar(255), cv::LINE_4);

	int inter_cnt = 0;
	int union_cnt = 0;
	for (int r = 0; r < img1.rows; r++) {
		for (int c = 0; c < img1.cols; c++) {
			if (img1(r, c) == 255 && img2(r, c) == 255) inter_cnt++;
			if (img1(r, c) == 255 || img2(r, c) == 255) union_cnt++;
		}
	}

	return (double)inter_cnt / union_cnt;

}