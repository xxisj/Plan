#ifndef PLANNING_PROBLEM_H
#define PLANNING_PROBLEM_H
#include <qobjectdefs.h>
#include <vector>
#include "psocore.h"
#include <QPolygonF>
#include <QGeoShape>
#include "astar.h"


struct target_data
{
	int start_time = 0;
	int end_time = 0;
	double hit_point = 1;
	double x = 0;
	double y = 0;
	double alt = 0;
	int target_index = 0;
	int desire_dir = 0;
	QString name;
	QString path;
};

struct uav_data
{
	int atk = 1;
	double x = 0;
	double y = 0;
	int base = -1;
	int speed = 1;
};

struct base_data
{
	double x = 0;
	double y = 0;
	int uav_count = 0;
	QString name;
	QString path;
};

class planning_problem
{
public:
	pso_core pso;
	astar astar;
	std::vector<target_data> targets;
	std::vector<base_data> bases;
	uav_data uav_stat;

	int uav_count = 0;
	int target_count = 0;
	std::vector<target_data> seperated_targets;
	QVector<QGeoShape> barriers;
	QVector<QPolygonF> barriers_poly;
	int iteration_count = 300;

    planning_problem();
	std::vector<QGeoCoordinate> find_path(double src_x, double src_y, double dst_x, double dst_y, int desired_dir);
	bool read_data_scenario(const char* path);
	bool ParseAreaFile(const QString& path);
	bool ParseEntityFile(const QString& path);
	bool read_data_xml(const char* path);
	bool read_data(const char* path);
	void prepare_data();
	void process(const char * inFile, const char * outFile, std::function<void(int)> percent_callback = nullptr);
	double solve(const char* path,std::function<void(int,int,const std::vector<int>&)> result_callback = nullptr, int iteration = 100, std::function<void(int)> percent_callback = nullptr);

private:
	double obj_func(double* args, int dim);
};

#endif // PLANNING_PROBLEM_H
