#ifndef ASTAR_H
#define ASTAR_H
#include "quadtree.h"
#include <QPolygonF>
#include <qgeoshape.h>

void enum_neighbor(quadtree* root, quadtree* node, int max_depth, function<void(quadtree*)> callback);
quadtree* find_quadtree(quadtree* root, double x, double y, int* out_depth = nullptr);
QRectF get_bounding_box(const QGeoShape * const shape);
class astar
{
public:
    astar();
	int generate_quadtree(quadtree* tree, const QVector<QPolygonF>& polys, int max_depth);
	int generate_quadtree(quadtree* tree, const QVector<QGeoShape>& polys, int max_depth, int begini);
	QVector<QPointF> solve(QPointF src, QPointF dst, QVector<QPolygonF> polygons, QVector<QGeoShape> shapes);
	QVector<QPointF> optimize(QVector<QPointF> prev_result,bool is_strict = false);
	QVector<QPointF> evalute_search(const QVector<QPointF> pts, std::function<double(const double*, int)> eval_func);
	int max_level = 7;
	int dst_direction = 0;
	quadtree* result_tree;
private:
	QVector<QPointF> solve(quadtree* tree, int max_depth, int node_count, QPointF src, QPointF dst);
	
};

#endif // ASTAR_H 
