
//#define VISUAL
#include "planning_problem.h"
#include <QFile>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QDomDocument>
#include <QFileInfo>
#include <cassert>
#include <QPointF>
//#include <opencv2/superres.hpp>
#include <QGeoCoordinate>
#include <QGeoCircle>
#include <QGeoRectangle>

#include <QXmlStreamWriter>
#include <QTextStream>
#if VISUAL
#include <opencv2/opencv.hpp>

void draw_polygon_x(cv::Mat& image, const QPolygonF& polygon, bool isFill, int percent, QPointF base)
{
	int cnt = polygon.size();
	std::vector<cv::Point> pts;
	for (auto& pt : polygon)
	{
		pts.push_back(cv::Point((pt.x() - base.x()) * 50, (pt.y() - base.y()) * 50));
	}
	cv::Point* polylist[1] = { pts.data() };
	int npt[] = { polygon.size() };
	if (isFill)
	{
		fillPoly(image, (const cv::Point**)polylist, npt, 1, cv::Scalar(255));
		cv::polylines(image, (const cv::Point**)polylist, npt, 1, 1, cv::Scalar(255), 1, 8, 0);
	}
	else
	{
		//fillPoly(image, (const cv::Point**)polylist, npt, 1, Scalar(percent));
		cv::polylines(image, (const cv::Point**)polylist, npt, 1, 1, cv::Scalar(percent), 1, 8, 0);
	}
}

void DrawQuadtree(quadtree* tree)
{
	cv::Mat image(tree->area.height() * 100, tree->area.width() * 100, CV_8U, cv::Scalar(0));
	auto base = tree->area.topLeft();
	auto draw_polygon = [&](const QPolygonF& polygon, bool isFill, int percent)
	{
		draw_polygon_x(image, polygon, isFill, percent, base);
	};
	{
		std::list<quadtree*> pending;
		pending.push_back(tree);
		for (;;)
		{
			if (pending.empty())
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
			draw_polygon(current->area, current->is_block && false, 125);
		}
	}
	cv::namedWindow("My Imagex");
	cv::imshow("My Imagex", image);
}
#endif

inline double zero_inf_to_zero_one(double value)
{
	return 1 - 1 / (1 + value * 0.1);
}

planning_problem::planning_problem() = default;

inline QGeoCoordinate coord2pt(const QPointF& pt)
{
	return QGeoCoordinate(pt.y(), pt.x());
}

inline QPointF pt2coord(const QGeoCoordinate& pt)
{
	return{ pt.longitude(),pt.latitude() };
}

template<class T>
auto lerp(const T& a, const T& b, double scale) -> decltype(a + b)
{
	return a * scale + b * (1 - scale);
}

std::vector<QGeoCoordinate> planning_problem::find_path(double src_x, double src_y, double dst_x,
														double dst_y, int desired_dir)
{
	//因为astar算法使用坐标系的原因，需要逆转y分量，详情见astar::solve函数的说明
	QPointF src(src_x, -src_y);
	QPointF dst(dst_x, -dst_y);
	astar.dst_direction = 360 - (90 - desired_dir);
	auto&& result = astar.solve(src, dst, barriers_poly, barriers);
	auto result2 = astar.optimize(result,true);
	auto result3 = astar.optimize(astar.optimize(result2), false);
	auto result4 = astar.optimize(result3, true);
	std::vector<QGeoCoordinate> ret;
	for(auto& ele : result4)
	{
		ret.push_back(QGeoCoordinate(-ele.y(), ele.x()));
	}
	decltype(ret) result5;
	result5.push_back(ret[0]);
	return ret;
	double step = 600;
	for(auto i = 1; i < ret.size(); i++)
	{
		const auto previous = result5[result5.size() - 1];
		const auto& current = ret[i];
		const auto distance = previous.distanceTo(current);
		const int lerp_points = floor(distance / step);
		for(int j = 1; j <= lerp_points; j++)
		{
			double scale = (step * j) / distance;
			result5.emplace_back(lerp(previous.latitude(), current.latitude(), scale),
				lerp(previous.longitude(), current.longitude(), scale),
				lerp(previous.altitude(), current.altitude(), scale));
		}
		result5.push_back(current);
	}

	try {
		//DrawQuadtree(astar.result_tree);
	}
	catch(...)
	{ }
	return result5;
}

bool planning_problem::read_data_scenario(const char* file)
{
	QString fileName(file);
	QFile xmlFile(fileName);

	if (!xmlFile.open(QFile::ReadOnly | QFile::Text))
	{
		return false;
	}
	//获取想定的文件夹路径
	int index = fileName.lastIndexOf('.');
	QString sceFolderPath = fileName.left(index);


	QString error;
	int row = 0, column = 0;
	QDomDocument doc;
	if (!doc.setContent(&xmlFile, false, &error, &row, &column))
	{
		xmlFile.close();
		return false;
	}
	//读取实体列表

	

	QDomElement root = doc.documentElement();
	if (root.isNull())
	{
		xmlFile.close();
		return false;
	}

	targets.clear();
	bases.clear();
	seperated_targets.clear();
	barriers.clear();
	barriers_poly.clear();

	auto Constants = root.firstChildElement("Constants");
	uav_stat.atk = Constants.firstChildElement("Attack").text().toInt();
	uav_stat.speed = Constants.firstChildElement("Speed").text().toInt();
	QDomElement EntityListModel = root.firstChildElement("EntityList");
	QDomElement  temp;
	if (!EntityListModel.isNull())
	{
		temp = EntityListModel.firstChildElement("Entity");
		while (!temp.isNull())
		{
			QString str = temp.text();
			QString entityFilePath = sceFolderPath;
			entityFilePath.append("/").append(str);
			ParseEntityFile(entityFilePath);
			temp = temp.nextSiblingElement("Entity");
		}
	}

	QDomElement AreaListModel = root.firstChildElement("AreaList");
	if (!AreaListModel.isNull())
	{
		temp = AreaListModel.firstChildElement("Area");
		while (!temp.isNull())
		{
			QString str = temp.text();
			QString entityFilePath = sceFolderPath;
			entityFilePath.append("/").append(str);
			ParseAreaFile(entityFilePath);
			temp = temp.nextSiblingElement("Area");
		}
	}

	xmlFile.close();
	return true;
}



bool planning_problem::ParseAreaFile(const QString& path)
{
	QFile xmlFile(path);

	if (!xmlFile.open(QFile::ReadOnly | QFile::Text))
	{
		return false;
	}

	QString error;
	int row = 0, column = 0;
	QDomDocument doc;
	if (!doc.setContent(&xmlFile, false, &error, &row, &column))
	{
		xmlFile.close();
		return false;
	}
	QDomElement root = doc.documentElement();
	if (root.isNull())
	{
		xmlFile.close();
		return false;
	}
	QDomElement temp = root.firstChildElement("Type");
	int area_type = temp.text().toInt();
	switch (area_type)
	{
	case 0:
		{
		auto x = root.firstChildElement("Lon").text().toDouble();
		auto y = root.firstChildElement("Lat").text().toDouble();
		auto radius = root.firstChildElement("Radius").text().toDouble();
		barriers.push_back(QGeoCircle(QGeoCoordinate(y, x), radius));
		}
		break;
	case 1:
		{
		auto top = root.firstChildElement("Top").text().toDouble();
		auto bottom = root.firstChildElement("Bottom").text().toDouble();
		auto left = root.firstChildElement("Left").text().toDouble();
		auto right = root.firstChildElement("Right").text().toDouble();
		barriers.push_back(QGeoRectangle(QGeoCoordinate(top, left), QGeoCoordinate(bottom, right)));
		}
		break;
	case 2:
		{
		auto vertices = root.firstChildElement("Points").childNodes();
		QPolygonF poly;
		for (int j = 0; j < vertices.count(); j++)
		{
			auto vertex = vertices.item(j);
			auto x = vertex.firstChildElement("Lon").text().toDouble();
			auto y = vertex.firstChildElement("Lat").text().toDouble();
			poly.push_back(QPointF(x, -y));
		}
		barriers_poly.push_back(poly);
		}
	}
	xmlFile.close();
	return true;
}

bool planning_problem::ParseEntityFile(const QString& path)
{
	QFile xmlFile(path);

	if (!xmlFile.open(QFile::ReadOnly | QFile::Text))
	{
		return false;
	}

	QString error;
	int row = 0, column = 0;
	QDomDocument doc;
	if (!doc.setContent(&xmlFile, false, &error, &row, &column))
	{
		xmlFile.close();
		return false;
	}
	QDomElement root = doc.documentElement();
	if (root.isNull())
	{
		xmlFile.close();
		return false;
	}
	QDomElement temp = root.firstChildElement("Name");
	QString entityName = temp.isNull() ? QString("Untitled") : temp.text();

	int part = 0; //默认是红方 0是红方 1是蓝防 2是中立方
	temp = root.firstChildElement("Type");
	QDomElement initEle = root.firstChildElement("Init");
	if (!temp.isNull())
	{
		if (temp.text().toUpper() == QString("BASE"))
		{
			base_data data;
			double lon = 0.0, lat = 0.0, alt = 0.0;
			double uav_count;
			QDomElement posEle = initEle.firstChildElement("Pos");
			auto uavCnt = initEle.firstChildElement("UavCount");
			uav_count = uavCnt.isNull() ? 0 : uavCnt.text().toDouble();
			if (!posEle.isNull()) 
			{
				temp = posEle.firstChildElement("Lon");
				lon = temp.isNull() ? 0.0 : temp.text().toDouble();

				temp = posEle.firstChildElement("Lat");
				lat = temp.isNull() ? 0.0 : temp.text().toDouble();

				temp = posEle.firstChildElement("Alt");
				alt = temp.isNull() ? 150 : temp.text().toDouble();
			}
			data.x = lon;
			data.y = lat;
			data.uav_count = uav_count;
			data.name = entityName;
			data.path = path;
			bases.push_back(data);
		}
		else if (temp.text().toUpper() == QString("TARGET"))
		{
			target_data data;
			double lon = 0.0, lat = 0.0, alt = 0.0;
			double start_time, end_time;
			double uav_count;
			QDomElement posEle = initEle.firstChildElement("Pos");
			auto hitpoint = initEle.firstChildElement("HitPoint");
			uav_count = hitpoint.isNull() ? 0 : hitpoint.text().toDouble();
			temp = initEle.firstChildElement("StartTime");
			start_time = temp.isNull() ? 0 : temp.text().toDouble();
			temp = initEle.firstChildElement("EndTime");
			end_time = temp.isNull() ? 0 : temp.text().toDouble();
			temp = initEle.firstChildElement("DesiredDir");
			int desired_dir = temp.isNull() ? -1 : temp.text().toInt();
			if (!posEle.isNull())
			{
				temp = posEle.firstChildElement("Lon");
				lon = temp.isNull() ? 0.0 : temp.text().toDouble();

				temp = posEle.firstChildElement("Lat");
				lat = temp.isNull() ? 0.0 : temp.text().toDouble();

				temp = posEle.firstChildElement("Alt");
				alt = temp.isNull() ? 150 : temp.text().toDouble();
			}
			data.x = lon;
			data.y = lat;
			data.alt = alt;
			data.hit_point = uav_count;
			data.start_time = start_time;
			data.end_time = end_time;
			data.name = entityName;
			data.desire_dir = desired_dir;
			data.path = path;
			targets.push_back(data);
		}
	}
	xmlFile.close();
	return true;
}

bool planning_problem::read_data_xml(const char* path)
{

	//Read Args
	{
		targets.clear();
		bases.clear();
		QFile xmlFile(path);

		if (!xmlFile.open(QFile::ReadOnly | QFile::Text))
		{
			return false;
		}

		QString error;
		int row = 0, column = 0;
		QDomDocument doc;
		if (!doc.setContent(&xmlFile, false, &error, &row, &column))
		{
			xmlFile.close();
			return false;
		}
		auto root = doc.documentElement();

		//uav_data
		uav_stat.atk = root.firstChildElement("uav").firstChildElement("atk").nodeValue().toInt();//root["uav"].toObject()["atk"].toInt();
		uav_stat.speed = root.firstChildElement("uav").firstChildElement("speed").nodeValue().toInt();//root["uav"].toObject()["speed"].toInt();
		auto targetx = root.firstChildElement("targets").childNodes();
		//target_data
		for (int i = 0; i < targetx.size(); i++)//root.firstChildElement("targets").nodeValue().toArray()
		{
			auto node = targetx.item(i);
			target_data data;
			data.x = node.firstChildElement("x").nodeValue().toDouble();
			data.y = node.firstChildElement("y").nodeValue().toDouble();
			data.start_time = node.firstChildElement("starttime").nodeValue().toInt();
			data.end_time = node.firstChildElement("endtime").nodeValue().toInt();
			data.hit_point = node.firstChildElement("hitpoint").nodeValue().toDouble();
			targets.push_back(data);
		}
		auto basex = root.firstChildElement("bases").childNodes();
		//base_data
		for (int i = 0; i< basex.size(); i++)
		{
			auto node = basex.item(i);
			base_data data;
			data.x = node.firstChildElement("x").nodeValue().toDouble();
			data.y = node.firstChildElement("y").nodeValue().toDouble();
			data.uav_count = node.firstChildElement("uavcount").nodeValue().toInt();
			bases.push_back(data);
		}
		auto barrierx = root.firstChildElement("barriers").childNodes();
		for (int i = 0; i < barrierx.size(); i++)
		{
			auto node = barrierx.item(i);
			switch (node.firstChildElement("type").nodeValue().toInt())
			{
			case 0:
			{
				//circle
				auto x = node.firstChildElement("x").nodeValue().toDouble();
				auto y = node.firstChildElement("y").nodeValue().toDouble();
				auto radius = node.firstChildElement("radius").nodeValue().toDouble();
				barriers.push_back(QGeoCircle(QGeoCoordinate(y, x), radius));
				break;
			}
			case 1:
			{
				//rect
				auto top = node.firstChildElement("top").nodeValue().toDouble();
				auto bottom = node.firstChildElement("bottom").nodeValue().toDouble();
				auto left = node.firstChildElement("left").nodeValue().toDouble();
				auto right = node.firstChildElement("right").nodeValue().toDouble();
				barriers.push_back(QGeoRectangle(QGeoCoordinate(top, left), QGeoCoordinate(bottom, right)));
				break;
			}
			case 2:
			{
				//polygon
				auto vertices = node.firstChildElement("vertices").childNodes();
				QPolygonF poly;
				for (int j = 0;j < vertices.count(); j++)
				{
					auto vertex = vertices.item(j);
					auto x = vertex.firstChildElement("x").nodeValue().toDouble();
					auto y = vertex.firstChildElement("y").nodeValue().toDouble();
					poly.push_back(QPointF(x, -y));
				}
				barriers_poly.push_back(poly);
				break;
			}
			}
		}
	}
	return true;
}

bool planning_problem::read_data(const char* path)
{
	//Read Args
	{
		targets.clear();
		bases.clear();
		QFile loadFile(path);
		if (!loadFile.open(QIODevice::ReadOnly))
		{
			return false;
		}
		QByteArray saveData = loadFile.readAll();
		QJsonDocument doc(QJsonDocument::fromJson(saveData));
		auto root = doc.object();

		//uav_data
		uav_stat.atk = root["uav"].toObject()["atk"].toInt();
		uav_stat.speed = root["uav"].toObject()["speed"].toInt();

		//target_data
		for (auto& ele : root["targets"].toArray())
		{
			auto node = ele.toObject();
			target_data data;
			data.x = node["x"].toDouble();
			data.y = node["y"].toDouble();
			data.start_time = node["starttime"].toInt();
			data.end_time = node["endtime"].toInt();
			data.hit_point = node["hitpoint"].toDouble();
			targets.push_back(data);
		}

		//base_data
		for (auto& ele : root["bases"].toArray())
		{
			auto node = ele.toObject();
			base_data data;
			data.x = node["x"].toDouble();
			data.y = node["y"].toDouble();
			data.uav_count = node["uavcount"].toInt();
			bases.push_back(data);
		}

		for(auto& ele : root["barriers"].toArray())
		{
			auto node = ele.toObject();
			switch (node["type"].toInt())
			{
			case 0:
			{
				//circle
				auto x = node["x"].toDouble();
				auto y = node["y"].toDouble();
				auto radius = node["radius"].toDouble();
				barriers.push_back(QGeoCircle(QGeoCoordinate(y, x), radius));
				break;
			}
			case 1:
			{
				//rect
				auto top = node["top"].toDouble();
				auto bottom = node["bottom"].toDouble();
				auto left = node["left"].toDouble();
				auto right = node["right"].toDouble();
				barriers.push_back(QGeoRectangle(QGeoCoordinate(top, left), QGeoCoordinate(bottom, right)));
				break;
			}
			case 2:
			{
				//polygon
				auto vertices = node["vertices"].toArray();
				QPolygonF poly;
				for (auto vertex_data : vertices)
				{
					auto vertex = vertex_data.toObject();
					auto x = vertex["x"].toDouble();
					auto y = vertex["y"].toDouble();
					poly.push_back(QPointF(x, -y));
				}
				barriers_poly.push_back(poly);
				break;
			}
			}
		}
	}
	return true;
}

void planning_problem::prepare_data()
{
	seperated_targets.clear();
	double total_hitpoint = 0;
	for (unsigned int index = 0; index < targets.size(); index++)
	{
		auto&& target = targets[index];
		int cnt = ceil(target.hit_point);
		bool isInt = (cnt - target.hit_point) < std::numeric_limits<double>::epsilon();
		for (int i = 0; i < cnt; i++)
		{
			target_data data(target);
			if (i == cnt - 1 && !isInt)
				data.hit_point = 1;// std::fmod(data.hit_point, 1);
			else
				data.hit_point = 1;
			data.target_index = index;
			total_hitpoint += target.hit_point;
			seperated_targets.push_back(data);
		}
	}
	int all_uav = 0;
	for (auto&& base : bases)
	{
		all_uav += base.uav_count;
	}
	uav_count = all_uav;// std::min((int)ceil(total_hitpoint), all_uav);
	pso.dim = seperated_targets.size() + uav_count;
	target_count = seperated_targets.size();
	pso.range_limit.clear();
	for (unsigned int i = 0; i < seperated_targets.size(); i++)
		pso.range_limit.emplace_back(0, double(uav_count) - 0.00001f);//这里用epsilon不知道会不会出问题(会
	for (int i = 0; i < uav_count; i++)
		pso.range_limit.emplace_back(-0.00001f, double(bases.size()) - 0.00001f);
}



double planning_problem::solve(const char* path, std::function<void(int, int, const std::vector<int>&)> result_callback, int iteration, std::function<void(int)> percent_callback)
{
	if(path != nullptr)
		if (!read_data_scenario(path)) return -1;

	prepare_data();

	using namespace std::placeholders;
	pso.set_obj_function(std::bind(&planning_problem::obj_func,this, _1, _2));


	double* best_result = new double[pso.dim];
	std::vector<double> results;
	double sum = 0;
	double min = 2;
	for (int iter = 0; iter < iteration; iter++)
	{
		pso.seed = rand();
		auto value = pso.calc();
		results.push_back(value);
		sum += value;
		if (value < min)
		{
			min = value;
			memcpy(best_result, pso.g_best, sizeof(double) * pso.dim);
		}
		if (percent_callback != nullptr)
			percent_callback(iter);
	}
	double avg = sum / results.size();
	double var = 0;
	for (int iter = 0; iter < iteration; iter++)
		var += pow(results[iter] - avg, 2);
	var = var / results.size();

	printf("min: %lf, avg: %lf, var : %lf", min, avg, var);

	auto args = best_result;
	const auto count = target_count;
	const auto order = new int[count];
	for (int i = 0; i < count; i++)
		order[i] = i;
	std::sort(order, order + count, [&](const int& left, const int& right)->bool {return args[left] < args[right]; });
	auto cur_uav = -1;
	std::vector<int> uav_path;
	for (auto i = 0; i<count; i++)
	{
		auto&& target = seperated_targets[order[i]];
		assert(floor(args[order[i]]) >= cur_uav);
		if (floor(args[order[i]]) > cur_uav)
		{
			if(!uav_path.empty() && result_callback != nullptr)
			{
				result_callback(cur_uav, (int)floor(args[target_count + cur_uav]), uav_path);
			}
			cur_uav = floor(args[order[i]]);
			uav_path.clear();
		}
		assert(cur_uav < uav_count);
		uav_path.insert(std::find(uav_path.begin(),uav_path.end(),target.target_index),target.target_index);
	}
	if (!uav_path.empty() && result_callback != nullptr)
	{
		result_callback(cur_uav, (int)floor(args[target_count + cur_uav]), uav_path);
	}
	delete[] best_result;
	delete[] order;
	return min;
}

void planning_problem::process(const char* inFile, const char* outFile, std::function<void(int)> percent_callback)
{
	struct solution
	{
		int uavid;
		int base;
		std::vector<int> uav_path;
	};
	std::vector<solution> solutions;
	solve(inFile, [&](int uavid, int base, const std::vector<int>& uav_path)
	{
		solution slu;
		slu.uavid = uavid;
		slu.uav_path.insert(slu.uav_path.end(), uav_path.begin(), uav_path.end());
		slu.base = base;
		solutions.push_back(slu);
	}, iteration_count, percent_callback);

	{
		std::map<int, std::vector<QGeoCoordinate>> computed_data;
		int count = bases.size() + targets.size() + 1;
		int target_index = targets.size();
		auto GetLoc = [&](int i)
		{
			if (i >= target_index)
				return QPointF(bases[i - target_index].x, bases[i - target_index].y);
			else
				return QPointF(targets[i].x, targets[i].y);
		};
		auto CalcPath = [&](int i, int j)
		{
			int index = i * count + j;
			auto value = computed_data.find(index);
			if (value != computed_data.end())
			{
				return value->second;
			}
			else
			{
				auto src = GetLoc(i);
				auto dst = GetLoc(j);
				auto&& ret = find_path(src.x(), src.y(), dst.x(), dst.y(),(j < target_index) ? targets[j].desire_dir : -1);
				computed_data[index] = ret;
				return ret;
			}
		};
		struct result_format
		{
			double lat;
			double lon;
			double alt;
			enum actions : int
			{
				Waypoint = 0,
				Target = 1
			} action;
			int target_id;
			int ammo;
			result_format() = default;
			result_format(double lat, double lon, double alt, int action, int target_id, int ammo)
				:lat(lat), lon(lon), action(static_cast<decltype(this->action)>(action)), target_id(target_id), ammo(ammo),alt(alt) {}
		};
		struct solution_format
		{
			std::vector<result_format> path;
			int base;
			int uavid;
		};
		std::vector<solution_format> output_slu;
		int uavid = 0;
		for (const auto& slu : solutions)
		{
			printf("\nuav %d, in base %d:", slu.uavid, slu.base);
			std::vector<result_format> result;
			int prev = slu.base + target_index;
			for (const auto& pt : slu.uav_path)
			{

				printf(" %d", pt);
				if (pt == prev)
				{
					++(result.back().ammo);
					continue;
				}
				auto result_astar = CalcPath(prev, pt);
				assert(result_astar.size() >= 2);
				for (auto iter = result_astar.begin(); iter != result_astar.end(); ++iter)
				{
					auto&& way_point = *iter;
					result.emplace_back(way_point.latitude(), way_point.longitude(), targets[pt].alt, result_format::Waypoint, 0, 0);
				}
				result.pop_back();
				result.emplace_back(targets[pt].y, targets[pt].x, targets[pt].alt, result_format::Target, pt, 1);
				prev = pt;
			}
			{
				const int pt = slu.base + target_index;
				auto result_astar = CalcPath(prev, pt);
				for (auto iter = result_astar.begin() + 1; iter != result_astar.end(); ++iter)
				{
					auto&& way_point = *iter;
					result.emplace_back(way_point.latitude(), way_point.longitude(), targets[prev].alt, result_format::Waypoint, 0, 0);
				}
			}
			solution_format item{ std::move(result),slu.base,slu.uavid };
			output_slu.push_back(std::move(item));
		}
		{
			QFile file(outFile);
			file.open(QIODevice::WriteOnly);
			QXmlStreamWriter writer(&file);
			writer.setAutoFormatting(true);
			writer.writeStartDocument();
			writer.writeStartElement("PlanningData");
			for (const auto& slu : output_slu)
			{
				writer.writeStartElement("Planning");
				writer.writeTextElement("UavID", QString::number(slu.uavid));
				writer.writeTextElement("Base", bases[slu.base].name);
				writer.writeStartElement("Path");
				for (const auto& pt : slu.path)
				{
					switch (pt.action)
					{
					case result_format::Waypoint:
					{
						writer.writeStartElement("Item");
						writer.writeTextElement("Lat", QString::number(pt.lat));
						writer.writeTextElement("Lon", QString::number(pt.lon));
						writer.writeTextElement("Alt", QString::number(pt.alt));
						writer.writeTextElement("Type", "Waypoint");
						writer.writeEndElement();
					}
					break;
					case result_format::Target:
					{
						writer.writeStartElement("Item");
						writer.writeTextElement("Lat", QString::number(pt.lat));
						writer.writeTextElement("Lon", QString::number(pt.lon));
						writer.writeTextElement("Alt", QString::number(pt.alt));
						writer.writeTextElement("Type", "Target");
						writer.writeTextElement("Atk", QString::number(pt.ammo));
						writer.writeTextElement("Target", targets[pt.target_id].name);
						writer.writeEndElement();
					}
					break;
					}
				}
				writer.writeEndElement();
				writer.writeEndElement();
			}
			writer.writeEndElement();
			writer.writeEndDocument();
			file.close();
		}
		{
			QFileInfo fi(outFile);
			for(const auto& slu : output_slu)
			{
				QJsonDocument doc;
				QJsonObject root;
				
				root.insert("MAV_AUTOPILOT", 6);
				root.insert("ROUTE_NAME", fi.fileName() + "-" + QString::number(slu.uavid));
				root.insert("complexitems", QJsonArray());
				root.insert("firmwareType", 12);
				QJsonArray items;
				int i = 1;
				bool is_first = true;
				for (const auto& pt : slu.path)
				{
					QJsonObject obj;
					obj.insert("autoContinue", true);
					obj.insert("command", is_first ? 22 : 16);
					obj.insert("coordinate", QJsonArray{ pt.lat, pt.lon, pt.alt });
					obj.insert("doJumpId", i);
					++i;
					obj.insert("params", QJsonArray{ 0,0,0,0 });
					obj.insert("type", "SimpleItem");
					obj.insert("frame", 3);
					items.append(obj);
					is_first = false;
				}
				root.insert("items", items);
				doc.setObject(root);
				auto data = doc.toJson();
				QFile file(outFile + QString(".") + QString::number(slu.uavid) + ".mission");
				file.open(QIODevice::WriteOnly);
				file.write(data);
				file.close();
			}
		}
	}
}

double planning_problem::obj_func(double* args, int dim)
{
	auto order = new int[target_count];
	for (int i = 0; i < target_count; i++)
		order[i] = i;
	std::sort(order, order + target_count, [&](const int& left, const int& right)->bool {return args[left] < args[right]; });
	int cur_uav = floor(args[order[0]]);
	double cur_distance = 0;
	double all_distance = 0;
	double cur_ammo = uav_stat.atk;
	double all_penalty = 0;
	double direct_penalty = 0;
	int uav_cnt = 0;
	double cur_time = 0;
	{
		const auto uav_constraint = new int[bases.size()];
		memset(uav_constraint, 0, sizeof(int) * bases.size());
		for (int i = target_count; i < dim; i++)
		{
			if (args[i] < 0)
				continue;
			uav_constraint[(int)floor(args[i])] += 1;
			uav_cnt += 1;
		}
		for (unsigned int i = 0; i < bases.size(); i++)
		{
			if (uav_constraint[i] > bases[i].uav_count)
			{
				all_penalty += (uav_constraint[i] - bases[i].uav_count) * 100;
			}
		}
		delete[] uav_constraint;
	}
	std::vector<bool> visited_targets(targets.size(),false);
	bool in_target = false;
	for (int i = 0; i<target_count; i++)
	{
		if (args[target_count + (int)floor(args[order[i]])] < 0)
		{
			direct_penalty += 1;
			break;
		}
		auto&& target = seperated_targets[order[i]];
		assert(floor(args[order[i]]) >= cur_uav);
		bool is_first = false;
		if (floor(args[order[i]]) > cur_uav)
		{
			auto&& prev_target = seperated_targets[order[i - 1]];
			auto&& uav_base = bases[(int)floor(args[target_count + cur_uav])];
			cur_distance += sqrt(pow(prev_target.x - uav_base.x, 2) + pow(prev_target.y - uav_base.y, 2));
			cur_uav = floor(args[order[i]]);
			all_distance += cur_distance;
			cur_distance = 0;
			cur_ammo = uav_stat.atk;
			is_first = true;
			cur_time = 0;
			in_target = false;
			for (unsigned int k = 0; k < visited_targets.size(); k++)
				visited_targets[k] = false;
		}
		assert(cur_uav < uav_count);
		auto&& uav_base = bases[(int)floor(args[target_count + cur_uav])];
		cur_ammo -= target.hit_point;
		if (cur_ammo < 0)
		{
			all_penalty += (-cur_ammo) * 100;
		}
		double distance = 0;
		if (!visited_targets[target.target_index]) {
			if (in_target)
			{
				auto&& prev_target = seperated_targets[order[i - 1]];
				distance += sqrt(pow(target.x - prev_target.x, 2) + pow(target.y - prev_target.y, 2));
			}
			else
				distance += sqrt(pow(target.x - uav_base.x, 2) + pow(target.y - uav_base.y, 2));
			cur_time += distance / uav_stat.speed;
			if (cur_time < target.start_time)
			{
				if (!is_first)
					all_penalty += (target.start_time - cur_time) * 1;
				cur_time = target.start_time;
			}
			else if (cur_time > target.end_time)
			{
				all_penalty += (cur_time - target.end_time) * 50;
			}
		}
		visited_targets[target.target_index] = true;
		cur_distance += distance;
		in_target = true;
	}
	if (args[target_count + cur_uav] > 0)
	{
		auto&& prev_target = seperated_targets[order[target_count - 1]];
		auto&& uav_base = bases[(int)floor(args[target_count + cur_uav])];
		cur_distance += sqrt(pow(prev_target.x - uav_base.x, 2) + pow(prev_target.y - uav_base.y, 2));
	}
	all_distance += cur_distance;
	double result = (zero_inf_to_zero_one(all_penalty) + zero_inf_to_zero_one(all_distance)) + zero_inf_to_zero_one(uav_cnt)* 0.2 + direct_penalty;
	delete[] order;
	return result;
}

