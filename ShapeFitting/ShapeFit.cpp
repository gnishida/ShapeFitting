#include "ShapeFit.h"

ShapeFit::ShapeFit() {
}

ShapeFit::~ShapeFit() {
}

std::vector<cv::Point2f> ShapeFit::fit(const std::vector<cv::Point2f>& polygon) {
	try {
		column_vector starting_point(8);
		starting_point(0) = 50;
		starting_point(1) = 50;
		starting_point(2) = 500;
		starting_point(3) = 50;
		starting_point(4) = 500;
		starting_point(5) = 500;
		starting_point(6) = 50;
		starting_point(7) = 500;

		BFGSSolver solver(polygon);
		find_max_using_approximate_derivatives(dlib::bfgs_search_strategy(), dlib::objective_delta_stop_strategy(1e-7),	solver, starting_point, 1, 1);

		std::vector<cv::Point2f> ans(4);
		for (int i = 0; i < 4; i++) {
			ans[i].x = starting_point(i * 2);
			ans[i].y = starting_point(i * 2 + 1);
		}

		return ans;
	}
	catch (char* ex) {
		std::cout << ex << std::endl;
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}
	catch (...) {
		std::cout << "BFGS optimization failure." << std::endl;
	}

	return{};
}