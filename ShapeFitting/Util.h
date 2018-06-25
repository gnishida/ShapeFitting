#pragma once

#include <vector>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Boolean_set_operations_2.h>

//typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Simple_cartesian<double> Kernel;

std::vector<std::vector<cv::Point2f>> findContours(const cv::Mat& image, int threshold, bool simplify);
float angle_difference(float x, float xi);
float regularize_angle_PI(float x);
bool lineLineIntersection(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c, const cv::Point2f& d, double *tab, double *tcd, bool segment_only, cv::Point2f& int_pt);

float calculateIOU(const std::vector<cv::Point2f>& polygon1, const std::vector<cv::Point2f>& polygon2);
float area(const CGAL::Polygon_with_holes_2<Kernel>& pwh);

float calculateImageBasedIOU(const std::vector<cv::Point2f>& polygon1, const std::vector<cv::Point2f>& polygon2);