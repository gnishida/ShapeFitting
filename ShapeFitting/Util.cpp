#include "Util.h"

/**
 * Find the contour polygons from the image.
 */
std::vector<std::vector<cv::Point2f>> findContours(const cv::Mat& image, int threshold, bool simplify, bool allow_diagonal, bool dilate) {
	std::vector<std::vector<cv::Point2f>> polygons;

	cv::Mat mat = image.clone();
	cv::threshold(mat, mat, threshold, 255, cv::THRESH_BINARY);

	cv::Mat_<uchar> mat2 = mat.clone();

	// if diagonal is not allowwd, dilate the diagonal connection.
	if (!allow_diagonal) {
		while (true) {
			bool updated = false;
			for (int r = 0; r < mat.rows - 1; r++) {
				for (int c = 0; c < mat.cols - 1; c++) {
					if (mat2(r, c) == 255 && mat2(r + 1, c + 1) == 255 && mat2(r + 1, c) == 0 && mat2(r, c + 1) == 0) {
						updated = true;
						mat2(r + 1, c) = 255;
					}
					else if (mat2(r, c) == 0 && mat2(r + 1, c + 1) == 0 && mat2(r + 1, c) == 255 && mat2(r, c + 1) == 255) {
						updated = true;
						mat2(r, c) = 255;
					}
				}
			}
			if (!updated) break;
		}
	}

	// dilate the image
	if (dilate) {
		// resize x4
		cv::Mat_<uchar> img3;
		cv::resize(mat2, img3, cv::Size(mat2.cols * 4, mat2.rows * 4), 0, 0, cv::INTER_NEAREST);

		// add padding
		cv::Mat_<uchar> padded = cv::Mat_<uchar>::zeros(img3.rows + 1, img3.cols + 1);
		img3.copyTo(padded(cv::Rect(0, 0, img3.cols, img3.rows)));

		// dilate image
		cv::Mat_<uchar> kernel = (cv::Mat_<uchar>(3, 3) << 1, 1, 0, 1, 1, 0, 0, 0, 0);
		cv::dilate(padded, mat2, kernel);
	}

	// extract contours
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	if (simplify) {
		cv::findContours(mat2, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	}
	else {
		cv::findContours(mat2, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
	}

	for (int i = 0; i < hierarchy.size(); i++) {
		if (hierarchy[i][3] != -1) continue;
		if (contours[i].size() < 3) continue;

		std::vector<cv::Point2f> polygon;
		polygon.resize(contours[i].size());
		for (int j = 0; j < contours[i].size(); j++) {
			if (dilate) {
				polygon[j] = cv::Point2f(std::round(contours[i][j].x * 0.25), std::round(contours[i][j].y * 0.25));
			}
			else {
				polygon[j] = cv::Point2f(contours[i][j].x, contours[i][j].y);
			}
		}
		if (dilate) {
			polygon = removeRedundantPoint(polygon);
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

std::vector<cv::Point2f> removeRedundantPoint(const std::vector<cv::Point2f>& polygon) {
	std::vector<cv::Point2f> ans;
	if (polygon.size() == 0) return ans;

	ans.push_back(polygon[0]);
	for (int i = 1; i < polygon.size(); i++) {
		if (polygon[i] != polygon[i - 1]) {
			ans.push_back(polygon[i]);
		}
	}
	if (ans.size() > 1 && ans.back() == ans.front()) ans.pop_back();

	/*
	for (int i = 0; i < ans.size() && ans.size() >= 3;) {
		int prev = (i - 1 + ans.size()) % ans.size();
		int next = (i + 1) % ans.size();
		if (dotProduct(ans[i] - ans[prev], ans[next] - ans[i]) > 0 && std::abs(crossProduct(ans[i] - ans[prev], ans[next] - ans[i])) < 0.0001) {
			ans.erase(ans.begin() + i);
		}
		else {
			i++;
		}
	}
	*/

	return ans;
}

std::vector<cv::Point2f> simplifyContour(const std::vector<cv::Point2f>& polygon, float epsilon) {
	std::vector<cv::Point2f> ans;
	cv::approxPolyDP(polygon, ans, epsilon, true);
	if (isSimple(ans)) return ans;
	else return polygon;
}

bool isSimple(const std::vector<cv::Point2f>& polygon) {
	CGAL::Polygon_2<Kernel> pol;
	for (auto& pt : polygon) {
		pol.push_back(Kernel::Point_2(pt.x, pt.y));
	}
	return pol.is_simple();
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

float distance(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c) {
	float dist;

	float r_numerator = (c.x - a.x) * (b.x - a.x) + (c.y - a.y) * (b.y - a.y);
	float r_denomenator = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);

	// For the case that the denominator is 0.
	if (r_denomenator <= 0.0f) {
		return cv::norm(a - c);
	}

	float r = r_numerator / r_denomenator;

	float px = a.x + r * (b.x - a.x);
	float py = a.y + r * (b.y - a.y);
	float s = ((a.y - c.y) * (b.x - a.x) - (a.x - c.x) * (b.y - a.y)) / r_denomenator;

	float distanceLine = std::fabs(s) * std::sqrt(r_denomenator);

	if ((r >= 0) && (r <= 1)) {
		dist = distanceLine;
	}
	else {
		float dist1 = (c.x - a.x) * (c.x - a.x) + (c.y - a.y) * (c.y - a.y);
		float dist2 = (c.x - b.x) * (c.x - b.x) + (c.y - b.y) * (c.y - b.y);
		if (dist1 < dist2) {
			dist = std::sqrt(dist1);
		}
		else {
			dist = std::sqrt(dist2);
		}
	}

	return abs(dist);
}

double calculateIOU(const std::vector<cv::Point2f>& polygon1, const std::vector<cv::Point2f>& polygon2) {
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

		double inter_area = 0;
		for (auto it = inter.begin(); it != inter.end(); it++) {
			inter_area += area(*it);
		}

		return inter_area / area(uni);
	}
	else {
		return 0;
	}
}

double calculateIOUbyImage(const std::vector<cv::Point2f>& polygon1, const std::vector<cv::Point2f>& polygon2, int image_size) {
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

	double scale = (float)image_size / std::max(max_x - min_x, max_y - min_y);

	cv::Mat_<uchar> img1 = cv::Mat_<uchar>::zeros(image_size, image_size);
	if (polygon1.size() > 0) {
		std::vector<std::vector<cv::Point>> contour_points1(1);
		contour_points1[0].resize(polygon1.size());
		for (int i = 0; i < polygon1.size(); i++) {
			contour_points1[0][i] = cv::Point((polygon1[i].x - min_x) * scale, (polygon1[i].y - min_y) * scale);
		}
		cv::fillPoly(img1, contour_points1, cv::Scalar(255), cv::LINE_4);
	}

	cv::Mat_<uchar> img2 = cv::Mat_<uchar>::zeros(image_size, image_size);
	if (polygon2.size() > 0) {
		std::vector<std::vector<cv::Point>> contour_points2(1);
		contour_points2[0].resize(polygon2.size());
		for (int i = 0; i < polygon2.size(); i++) {
			contour_points2[0][i] = cv::Point((polygon2[i].x - min_x) * scale, (polygon2[i].y - min_y) * scale);
		}
		cv::fillPoly(img2, contour_points2, cv::Scalar(255), cv::LINE_4);
	}

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

double calculatePoLIS(const std::vector<cv::Point2f>& polygon1, const std::vector<cv::Point2f>& polygon2) {
	double dist1 = 0;
	for (auto& pt : polygon1) {
		double min_dist = std::numeric_limits<double>::max();
		for (int i = 0; i < polygon2.size(); i++) {
			int i2 = (i + 1) % polygon2.size();
			min_dist = std::min(min_dist, (double)distance(polygon2[i], polygon2[i2], pt));
		}

		dist1 += min_dist;
	}
	dist1 /= polygon1.size();

	double dist2 = 0;
	for (auto& pt : polygon2) {
		double min_dist = std::numeric_limits<double>::max();
		for (int i = 0; i < polygon1.size(); i++) {
			int i2 = (i + 1) % polygon1.size();
			min_dist = std::min(min_dist, (double)distance(polygon1[i], polygon1[i2], pt));
		}

		dist2 += min_dist;
	}
	dist2 /= polygon2.size();

	return std::max(dist1, dist2);
}

double area(const CGAL::Polygon_with_holes_2<Kernel>& pwh) {
	double ans = 0;

	ans += CGAL::to_double(pwh.outer_boundary().area());
	for (auto it = pwh.holes_begin(); it != pwh.holes_end(); it++) {
		ans -= CGAL::to_double(it->area());
	}

	return ans;
}
