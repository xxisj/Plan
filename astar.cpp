#include "astar.h"
#include <queue>
#include <QLineF>
#include <stack>
#include <QGeoCircle>
#include <QGeoShape>
#include <QGeoRectangle>
#include <QVector2D>
#include <QGeoCoordinate>
#include <QtMath>

inline static double earthMeanRadius()
{
	return 6371007.2;
}

bool crossNorthPole(const QGeoCircle* const circle)
{
	auto&& m_center = circle->center();
	const QGeoCoordinate northPole(90.0, m_center.longitude());
	qreal distanceToPole = m_center.distanceTo(northPole);
	return distanceToPole < circle->radius();
}

bool crossSouthPole(const QGeoCircle* const circle)
{
	auto&& m_center = circle->center();
	const QGeoCoordinate southPole(-90.0, m_center.longitude());
	qreal distanceToPole = m_center.distanceTo(southPole);
	return distanceToPole < circle->radius();
}

inline static double radians(double degrees)
{
	return qDegreesToRadians(degrees);
}

inline static double degrees(double radians)
{
	return qRadiansToDegrees(radians);
}

inline static double clipLat(double lat, double clipValue = 90.0) {
	if (lat > clipValue)
		lat = clipValue;
	else if (lat < -clipValue)
		lat = -clipValue;
	return lat;
}

inline static double wrapLong(double lng) {
	if (lng > 180.0)
		lng -= 360.0;
	else if (lng < -180.0)
		lng += 360.0;
	return lng;
}

QRectF get_bounding_box(const QGeoCircle * const circle)
{
	auto&& m_center = circle->center();
	auto&& m_radius = circle->radius();
	QGeoRectangle m_bbox;
	bool crossNorth = crossNorthPole(circle);
	bool crossSouth = crossSouthPole(circle);

	if (crossNorth && crossSouth) {
		// Circle crossing both poles fills the whole map
		m_bbox = QGeoRectangle(QGeoCoordinate(90.0, -180.0), QGeoCoordinate(-90.0, 180.0));
	}
	else if (crossNorth) {
		// Circle crossing one pole fills the map in the longitudinal direction
		m_bbox = QGeoRectangle(QGeoCoordinate(90.0, -180.0), QGeoCoordinate(m_center.atDistanceAndAzimuth(m_radius, 180.0).latitude(), 180.0));
	}
	else if (crossSouth) {
		m_bbox = QGeoRectangle(QGeoCoordinate(m_center.atDistanceAndAzimuth(m_radius, 0.0).latitude(), -180.0), QGeoCoordinate(-90, 180.0));
	}
	else {
		// Regular circle not crossing anything

		// Calculate geo bounding box of the circle
		//
		// A circle tangential point with a meridian, together with pole and
		// the circle center create a spherical triangle.
		// Finding the tangential point with the spherical law of sines:
		//
		// * lon_delta_in_rad : delta between the circle center and a tangential
		//   point (absolute value).
		// * r_in_rad : angular radius of the circle
		// * lat_in_rad : latitude of the circle center
		// * alpha_in_rad : angle between meridian and radius of the circle.
		//   At the tangential point, sin(alpha_in_rad) == 1.
		// * lat_delta_in_rad - absolute delta of latitudes between the circle center and
		//   any of the two points where the great circle going through the circle
		//   center and the pole crosses the circle. In other words, the points
		//   on the circle with azimuth 0 or 180.
		//
		//  Using:
		//  sin(lon_delta_in_rad)/sin(r_in_rad) = sin(alpha_in_rad)/sin(pi/2 - lat_in_rad)

		double r_in_rad = m_radius / earthMeanRadius(); // angular r
		double lat_delta_in_deg = degrees(r_in_rad);
		double lon_delta_in_deg = degrees(std::asin(
			std::sin(r_in_rad) /
			std::cos(radians(m_center.latitude()))
		));

		QGeoCoordinate topLeft;
		topLeft.setLatitude(clipLat(m_center.latitude() + lat_delta_in_deg));
		topLeft.setLongitude(wrapLong(m_center.longitude() - lon_delta_in_deg));
		QGeoCoordinate bottomRight;
		bottomRight.setLatitude(clipLat(m_center.latitude() - lat_delta_in_deg));
		bottomRight.setLongitude(wrapLong(m_center.longitude() + lon_delta_in_deg));

		m_bbox = QGeoRectangle(topLeft, bottomRight);
	}
	//QRectF是在y轴向下的坐标系，所以取负号
	return QRectF(
		QPointF(m_bbox.topLeft().longitude(), -m_bbox.topLeft().latitude()),
		QPointF(m_bbox.bottomRight().longitude(), -m_bbox.bottomRight().latitude()));
}

QRectF get_bounding_box(const QGeoShape* const shape)
{
	switch (shape->type())
	{
	case QGeoShape::ShapeType::CircleType:
	{
		auto circle = (QGeoCircle*)shape;
		return get_bounding_box(circle);
	}
	case QGeoShape::ShapeType::RectangleType:
	{
		auto& m_bbox = *((QGeoRectangle*)shape);
		return QRectF(
			QPointF(m_bbox.topLeft().longitude(), -m_bbox.topLeft().latitude()),
			QPointF(m_bbox.bottomRight().longitude(), -m_bbox.bottomRight().latitude()));
	}
	default:
		return QRectF();
	}
}

void traverseTree(quadtree* root, int level, QPointF src, QPointF dir, function<void(quadtree*)> callback)
{
	double length = std::max(abs(dir.y() - src.y()), abs(dir.x() - src.x()));//sqrt((dir.y - src.y) * (dir.y - src.y) + (dir.x - src.x) * (dir.x - src.x));
	int step_length;
	if (length == 0)
		return;
	double dx = (double)(dir.x() - src.x()) / length;
	double dy = (double)(dir.y() - src.y()) / length;
	double min_step_y = root->area.width()  * pow(0.5, level + 1);
	double min_step_x = root->area.height() * pow(0.5, level + 1);
	min_step_x = min_step_y = std::max(min_step_x, min_step_y);
	dx = dx * min_step_x;
	dy = dy * min_step_y;
	step_length = (int)ceil(length / min_step_x);
	double x = src.x();// +0.5 * min_step_x;
	double y = src.y();// +0.5 * min_step_y;
					 //double m = (dir.y - src.y) / (dir.x - src.x);
					 //double e = m - 0.5f;
	quadtree* prevhit = find_quadtree(root, src.x(), src.y());
	for (int step = 0; step < step_length;step++)
	{
		if (root->area.contains(x, y) == false)
			break;
		auto hit = find_quadtree(root, x, y);
		if (hit != prevhit)
		{
			callback(hit);
			prevhit = hit;
		}
		x += dx;
		y += dy;
	}
}

astar::astar()
{
	result_tree = nullptr;
}

void delete_quadtree(quadtree* root)
{
	std::list<quadtree*> pending;
	pending.push_back(root);
	for (;;)
	{
		if(pending.empty())
			break;
		quadtree* current = pending.back();
		pending.pop_back();
		for (int i = 0; i < 4; i++)
		{
			if (current->child[i] != nullptr)
			{
				pending.push_back(current->child[i]);
			}
		}
		delete current;
	}
}

quadtree* find_quadtree(quadtree* root, double x, double y,int* out_depth)
{
	int depth = 0;
	quadtree* current = root;
	if (!root->area.contains(x, y))
		return nullptr;
	for (;;)
	{
		auto child = current->get_side(x, y);
		if (child == nullptr)
			break;
		current = child;
		depth++;
	}
	if(out_depth != nullptr)
	{
		*out_depth = depth;
	}
	return current;
}

int get_depth(quadtree* root, quadtree* node)
{
	quadtree* current = node;
	int level = 0;
	for (;;)
	{
		if (current == root)
			break;
		current = current->parent;
		level++;
	}
	return level;
}

void enum_neighbor(quadtree* root, quadtree* node,int max_depth, function<void(quadtree*)> callback)
{
	double minimal_x = root->area.width() / pow(2, max_depth + 1);
	double minimal_y = root->area.height() / pow(2, max_depth + 1);
	int depth = get_depth(root, node);
	auto max_x = node->area.right();
	auto max_y = node->area.bottom();
	auto tmp_y = node->area.top() - minimal_y;
	for (double x = node->area.left() + minimal_x; x < max_x;)
	{
		
		auto current = find_quadtree(root, x, tmp_y);
		if(current == nullptr)break;
		int cur_depth = get_depth(root, current);
		callback(current);
		x += current->area.width();
	}
	tmp_y = node->area.bottom() + minimal_y;
	for (double x = node->area.left() + minimal_x; x < max_x;)
	{

		auto current = find_quadtree(root, x, tmp_y);
		if (current == nullptr)break;
		int cur_depth = get_depth(root, current);
		callback(current);
		x += current->area.width();
	}

	auto tmp_x = node->area.left() - minimal_x;
	for(double y = node->area.top();y < max_y;)
	{
		auto current = find_quadtree(root, tmp_x, y);
		if (current == nullptr)break;
		int cur_depth = get_depth(root, current);
		callback(current);
		y += current->area.height();
	}
	tmp_x = node->area.right() + minimal_x;
	for (double y = node->area.top();y < max_y;)
	{
		auto current = find_quadtree(root, tmp_x, y);
		if (current == nullptr)break;
		int cur_depth = get_depth(root, current);
		callback(current);
		y += current->area.height();
	}
}

int astar::generate_quadtree(quadtree* tree, const QVector<QPolygonF>& polys, int max_depth)
{
	double sample_interval_x = tree->area.width() / pow(2, max_depth);
	double sample_interval_y = tree->area.height() / pow(2, max_depth);
	int id = 0;
	for(auto& poly : polys)
	{
		auto out_rect = poly.boundingRect();
		auto max_x = out_rect.right();
		for (double x = out_rect.left(); x < max_x; x += sample_interval_x)
		{
			auto max_y = out_rect.bottom();
			for (double y = out_rect.top(); y < max_y; y += sample_interval_y)
			{
				if(poly.containsPoint(QPointF(x,y),Qt::WindingFill))
					tree->insertblock(x, y, max_depth);
			}
		}
	}
	return id;
}

int astar::generate_quadtree(quadtree* tree, const QVector<QGeoShape>& polys, int max_depth, int begini = 0)
{
	double sample_interval_x = tree->area.width() / pow(2, max_depth);
	double sample_interval_y = tree->area.height() / pow(2, max_depth);
	int id = begini;
	for (auto& poly : polys)
	{
		auto out_rect = get_bounding_box(&poly);
		auto max_x = out_rect.right();
		for (double x = out_rect.left(); x < max_x; x += sample_interval_x)
		{
			auto max_y = out_rect.bottom();
			for (double y = out_rect.top(); y < max_y; y += sample_interval_y)
			{
				if(poly.contains(QGeoCoordinate(-y, x)))
					tree->insertblock(x, y, max_depth);
			}
		}
	}
	return id;
}

double fitness_func(double distance,QPointF pos, QPointF dst)
{
	auto target_dist = QLineF(pos, dst).length();
	return distance - target_dist;
}

struct node_data
{
	double distance{ std::numeric_limits<double>::max() };
	double fitness{ std::numeric_limits<double>::max() };
	double dir_factor{ 0 };
	int prev_node{ -1 };
	quadtree* ptr{ nullptr };
};

QVector<QPointF> astar::solve(quadtree* tree, int max_depth, int node_count, QPointF src, QPointF dst)
{

	struct order_data
	{
		int id{ -1 };
		double fitness{ 0 };
		order_data() = default;
		order_data(int id, double distance)
		{
			this->id = id;
			this->fitness = distance;
		}
	};
	auto cmp = [](const order_data& left, const order_data& right) {return left.fitness > right.fitness; };
	std::priority_queue<order_data, std::vector<order_data>, decltype(cmp)> queue(cmp);

	std::vector<node_data> data;
	data.resize(node_count);

	auto dst_node = find_quadtree(tree, dst.x(), dst.y());
	int dst_node_id = dst_node->custom_data;
	auto src_node = find_quadtree(tree, src.x(), src.y());
	int src_node_id = src_node->custom_data;
	double step = fmax(tree->area.width(), tree->area.height()) * pow(0.5, max_depth + 1);

	queue.push(order_data(src_node_id, 0));
	data[src_node_id].distance = 0;
	data[src_node_id].ptr = src_node;

	auto dir_factor_func = [&](quadtree* current, quadtree* next) -> double
	{
		if (data[current->custom_data].prev_node == -1)
			return 0;
		auto& prev = data[data[current->custom_data].prev_node];
		QVector2D vec_prev(current->area.center() - prev.ptr->area.center());
		QVector2D vec_next(next->area.center() - current->area.center());
		double distance = vec_next.length();
		double distance_factor = distance + vec_prev.length();
		vec_prev.normalize();
		vec_next.normalize();
		double delta_degree = acosf(QVector2D::dotProduct(vec_prev, vec_next));
		return data[current->custom_data].dir_factor + delta_degree * 400.0f * step / distance_factor;
	};

	auto fitnessfunc = [&](quadtree* current, quadtree* next)
	{
		QVector2D vec_next(next->area.center() - current->area.center());
		QVector2D vec_target(dst - next->area.center());
		QVector2D vec_dir(cosf(this->dst_direction / 180.0f * M_PI), sinf(this->dst_direction / 180.0f * M_PI));
		vec_target.normalize();
		double delta_degree = acosf(QVector2D::dotProduct(vec_target, vec_dir));
		double distance = vec_next.length();
		double dst_factor = QVector2D(dst - next->area.center()).length();
		return (data[current->custom_data].distance + distance + dst_factor) / step + dir_factor_func(current, next) + delta_degree * 100.0f;
	};

	//enumerate
	for (;;)
	{
		if (queue.empty())
		{
			return{ src,dst };
		}
		auto& node = queue.top();

		auto current = data[node.id].ptr;
		auto& current_node_data = data[node.id];
		if (node.id == dst_node_id)
			break;
		auto pos = current->area.center();
		queue.pop();
		auto check_node = [&](quadtree* nextnode)
		{
			if (nextnode->is_block)
				return;
			auto additional_distance = QLineF(pos, nextnode->area.center()).length();
			auto& next_node_data = data[nextnode->custom_data];
			auto fitness = fitnessfunc(current, nextnode);//(current_node_data.distance + additional_distance, nextnode->area.center(), dst);
			if (fitness < next_node_data.fitness)
			{
				if (nextnode->custom_data == dst_node_id)
				{
					QVector2D vec(nextnode->area.center() - current->area.center());
					QVector2D vec_dir(cosf(this->dst_direction / 180.0f * M_PI), sinf(this->dst_direction / 180.0f * M_PI));
					vec.normalize();
					double degree = acos(QVector2D::dotProduct(vec, vec_dir));
					if (degree > 30 * M_PI / 180.0f)
					{
						return;
					}
				}
				next_node_data.distance = current_node_data.distance + additional_distance;
				next_node_data.fitness = fitness;
				next_node_data.ptr = nextnode;
				next_node_data.prev_node = current_node_data.ptr->custom_data;
				next_node_data.dir_factor = dir_factor_func(current, nextnode);
				nextnode->custom_data2 = current->custom_data2 + 1;
				queue.push(order_data(nextnode->custom_data, fitness));
			}
		};
		current->enum_neighbor(check_node);
		if (QVector2D(dst - current->area.center()).length() < step * 16)
			check_node(dst_node);
	}

	//output result
	QVector<QPointF> result;
	{
		std::stack<QPointF> results;
		results.push(dst);
		int current_id = dst_node_id;
		for (;;)
		{
			if (current_id == src_node_id)
				break;
			results.push(data[current_id].ptr->area.center());
			current_id = data[current_id].prev_node;
		}
		results.push(src);
		for (;;)
		{
			if (results.empty())
				break;
			result.push_back(results.top());
			results.pop();
		}
	}
	return result;
}
QRectF merge_bounding_box(QRectF a, QRectF b)
{
	double max_x, min_x;
	double max_y, min_y;
	max_x = std::max({ a.left(),a.right(),b.left(),b.right() });
	min_x = std::min({ a.left(),a.right(),b.left(),b.right() });
	max_y = std::max({ a.top(),a.bottom(),b.top(),b.bottom() });
	min_y = std::min({ a.top(),a.bottom(),b.top(),b.bottom() });  
	return{ QPointF(min_x, min_y), QPointF(max_x, max_y) };
}

static int enum_quadtree_node(quadtree* root)
{
	std::list<quadtree*> pending;
	int cnt = 0;
	pending.push_back(root);
	for (;;)
	{
		if (pending.empty())
			break;
		quadtree* current = pending.back();
		pending.pop_back();
		if (current->is_block == false)
		{
			current->custom_data = cnt;
			cnt += 1;
		}

		for (int i = 0; i < 4; i++)
		{
			if (current->child[i] != nullptr)
			{
				pending.push_back(current->child[i]);
			}
		}
	}
	return cnt;
}




/*
 * 这里因为QRectF等采用的是y轴朝下的坐标系
 * 所以输入参数Polygons默认是y轴朝下的
 * 而输入的QGeoShape则由内部进行反转
 * 最后输出的结果也是y轴朝下的
 * 默认src，以及dst是转换到了y轴朝下的坐标系的
 */
QVector<QPointF> astar::solve(QPointF src, QPointF dst, QVector<QPolygonF> polygons, QVector<QGeoShape> shapes)
{
	if (result_tree != nullptr)
		delete_quadtree(result_tree);
	quadtree* tree = new quadtree();
	QRectF bounding_box = QRectF(src, src * 1.000001);
	bounding_box = merge_bounding_box(bounding_box, QRectF(dst, dst * 1.000001));
	for(auto& polygon : polygons)
	{
		bounding_box = merge_bounding_box(bounding_box, polygon.boundingRect());
	}
	tree->area = QRectF(QPointF(bounding_box.topLeft().x() - bounding_box.width() * 0.25, bounding_box.topLeft().y() - bounding_box.height()*0.25), QSizeF(bounding_box.width() * 1.5f, bounding_box.height() * 1.5f));

	tree->insertblock(src.x(), src.y(), max_level);
	auto fnd1 = find_quadtree(tree, src.x(), src.y());
	fnd1->is_block = false;
	tree->insertblock(dst.x(), dst.y(), max_level);
	auto fnd2 = find_quadtree(tree, dst.x(), dst.y());
	fnd2->is_block = false;

	fnd2->enum_neighbor([this](quadtree* node)
	{
		if (node->get_level()< max_level)
			node->init_inner();
	});
	int i = generate_quadtree(tree, polygons, max_level);
	generate_quadtree(tree, shapes, max_level, i);
	auto count = enum_quadtree_node(tree);
	result_tree = tree;
	
	auto result = solve(tree, max_level, count, src, dst);
	
	return result;

	//enum_neighbor(tree, fnd2, max_level, [](quadtree* node) {node->custom_data2 = 10; });
	return{ src,dst };
}

QVector<QPointF> astar::optimize(const QVector<QPointF> prev_result, bool is_strict)
{
	QVector<QPointF> result;

	if (prev_result.size() < 3)
		return prev_result;
	QVector<QPointF> result_fin;
	result_fin.append(prev_result[0]);
	for (int i = 1; i < prev_result.size() - 1; i++)
	{

		auto&& prev = result_fin.last();
		auto&& curr = prev_result[i];
		auto&& next = prev_result[i + 1];
		QVector2D vec_prev(curr - prev);
		QVector2D vec_curr(next - curr);
		vec_prev.normalize();
		vec_curr.normalize();
		double delta_degree = acosf(QVector2D::dotProduct(vec_prev, vec_curr));
		int prev_depth, cur_depth, next_depth;
		find_quadtree(result_tree, result_fin.last().x(), result_fin.last().y(), &prev_depth);
		find_quadtree(result_tree, prev_result[i].x(), prev_result[i].y(), &cur_depth);
		find_quadtree(result_tree, next.x(), next.y(), &next_depth);
		if (cur_depth < prev_depth && next_depth < prev_depth && is_strict && i < prev_result.size() - 3)
		{
			double step = -0.5;
			double succ_loc = 1;
			double currentloc = 0.5;
			for (int times = 0; times < 4; times++)
			{
				step *= 0.5;
				bool trav_result = false;//no intersect
				auto loc = prev * (1 - currentloc) + curr * (currentloc);
				traverseTree(result_tree, max_level, loc, prev_result[i + 1], [&](quadtree* tree)
				{
					trav_result = trav_result || tree->is_block;
				});
				if (trav_result)
				{
					step = -step;
				}
				else
				{
					succ_loc = currentloc;
					step = -fabs(step);
				}
				currentloc += step;
			}
			result_fin.append(prev * (1 - succ_loc) + curr * (succ_loc));
			continue;
		}
		else if (cur_depth < next_depth && prev_depth < next_depth && is_strict && i < prev_result.size() - 3)
		{
			double step = 0.5;
			double succ_loc = 1;
			double currentloc = 0.5;
			for (int times = 0; times < 4; times++)
			{
				step *= 0.5;
				bool trav_result = false;//no intersect
				auto loc = prev * (1 - currentloc) + curr * (currentloc);
				traverseTree(result_tree, max_level, loc, prev_result[i + 1], [&](quadtree* tree)
				{
					trav_result = trav_result || tree->is_block;
				});
				if (trav_result)
				{
					step = -step;
				}
				else
				{
					succ_loc = currentloc;
					step = -fabs(step);
				}
				currentloc += step;
			}
			result_fin.append(prev * (1 - succ_loc) + curr * (succ_loc));
			continue;
		}
		else if (!is_strict && delta_degree < M_PI / 18)
		{
			bool traversalResult = false;
			traverseTree(result_tree, max_level, result_fin.last(), prev_result[i + 1], [&](quadtree* tree)
			{
				traversalResult = traversalResult || tree->is_block;
			});
			if (traversalResult)
			{
				result_fin.append(prev_result[i]);
			}
		}
		else
		{
			result_fin.append(prev_result[i]);
		}
	}
	result_fin.append(prev_result.last());
	return result_fin;
}