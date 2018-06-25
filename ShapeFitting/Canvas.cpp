#include "Canvas.h"
#include <QPainter>
#include <iostream>
#include <QFileInfoList>
#include <QDir>
#include <QMessageBox>
#include <QTextStream>
#include <QResizeEvent>
#include "Util.h"
#include "ShapeFit.h"

Canvas::Canvas(QWidget *parent) : QWidget(parent) {
	ctrlPressed = false;
	shiftPressed = false;
}

void Canvas::loadImage(const QString& filename) {
	orig_image = QImage(filename).convertToFormat(QImage::Format_Grayscale8);
	image_scale = std::min((float)width() / orig_image.width(), (float)height() / orig_image.height());
	image = orig_image.scaled(orig_image.width() * image_scale, orig_image.height() * image_scale);

	cv::Mat mat = cv::Mat(orig_image.height(), orig_image.width(), CV_8UC1, orig_image.bits(), orig_image.bytesPerLine()).clone();
	polygons = findContours(mat, 40, true, false, true);

	for (auto& polygon : polygons) {
		polygon = simplifyContour(polygon, 4);
	}

	simplified_polygons.clear();

	update();
}

void Canvas::saveImage(const QString& filename) {
	grab().save(filename);
}

void Canvas::fitShape() {
	simplified_polygons.resize(polygons.size(), {});

	int max_count = 0;
	int max_index = -1;
	for (int i = 0; i < polygons.size(); i++) {
		if (polygons[i].size() > max_count) {
			max_count = polygons[i].size();
			max_index = i;
		}
	}

	if (max_index >= 0) {
		time_t start = clock();
		simplified_polygons[max_index] = ShapeFit::fit(polygons[max_index]);
		time_t end = clock();
		std::cout << "Time elapsed: " << (double)(end - start) / CLOCKS_PER_SEC << " sec." << std::endl;
	}
}

void Canvas::keyPressEvent(QKeyEvent* e) {
	ctrlPressed = false;
	shiftPressed = false;

	if (e->modifiers() & Qt::ControlModifier) {
		ctrlPressed = true;
	}
	if (e->modifiers() & Qt::ShiftModifier) {
		shiftPressed = true;
	}

	switch (e->key()) {
	case Qt::Key_Space:
		break;
	}

	update();
}

void Canvas::keyReleaseEvent(QKeyEvent* e) {
	switch (e->key()) {
	case Qt::Key_Control:
		ctrlPressed = false;
		break;
	case Qt::Key_Shift:
		shiftPressed = false;
		break;
	default:
		break;
	}
}

void Canvas::paintEvent(QPaintEvent *event) {
	QPainter painter(this);

	painter.fillRect(0, 0, width(), height(), QColor(255, 255, 255));

	painter.setPen(QPen(QColor(0, 0, 0), 1));
	for (auto& polygon : polygons) {
		QPolygon pgon;
		for (auto& p : polygon) {
			pgon.push_back(QPoint(p.x * image_scale, p.y * image_scale));
		}
		painter.drawPolygon(pgon);
	}

	painter.setPen(QPen(QColor(0, 0, 255), 3));
	for (auto& simplified_polygon : simplified_polygons) {
		QPolygon pgon;
		for (auto& p : simplified_polygon) {
			pgon.push_back(QPoint(p.x * image_scale, p.y * image_scale));
		}
		painter.drawPolygon(pgon);
	}
}

void Canvas::mousePressEvent(QMouseEvent* e) {
	update();
}

void Canvas::resizeEvent(QResizeEvent *e) {
	if (!orig_image.isNull()) {
		image_scale = std::min((float)width() / orig_image.width(), (float)height() / orig_image.height());
		image = orig_image.scaled(orig_image.width() * image_scale, orig_image.height() * image_scale);
	}
}

