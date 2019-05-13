#ifndef QUADTREE_H
#define QUADTREE_H
#include <QRectF>
#include <functional>

using std::function;

class quadtree
{
public:
	quadtree();
	/*
	┌─┬─┐
	│ 1│ 0│
	├─┼─┤
	│ 3│ 2│
	└─┴─┘
	*/
	quadtree* child[4];
	quadtree* parent{ nullptr };
	int custom_data{ 0 };
	int custom_data2{ 0 };
	inline quadtree*& operator [](int index);
	bool has_child();
	void init_inner(function<void(quadtree*)> new_node_cb = nullptr);
	bool is_block = false;
	QRectF area;
	void insertblock(double x, double y, int depth, function<void(quadtree*)> new_node_cb = nullptr);
	int get_index(double x, double y);
	quadtree*& get_side(double x, double y);
	quadtree* get_root() const;
	void enum_neighbor(function<void(quadtree*)> callback);
	int get_level() const;
	int get_loc() const;
};

#endif // QUADTREE_H
