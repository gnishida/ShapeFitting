#ifndef CANVAS_H
#define CANVAS_H

#include <vector>
#include <QWidget>
#include <QKeyEvent>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

class Canvas : public QWidget {
private:
	QImage orig_image;
	QImage image;
	float image_scale;
	std::vector<std::vector<cv::Point2f>> polygons;
	std::vector<std::vector<cv::Point2f>> simplified_polygons;

	bool ctrlPressed;
	bool shiftPressed;
	
public:
	Canvas(QWidget *parent = NULL);

	void loadImage(const QString& filename);
	void saveImage(const QString& filename);
	void fitShape();
	void keyPressEvent(QKeyEvent* e);
	void keyReleaseEvent(QKeyEvent* e);

protected:
	void paintEvent(QPaintEvent *event);
	void mousePressEvent(QMouseEvent* e);
	void resizeEvent(QResizeEvent *e);
};

#endif // CANVAS_H
