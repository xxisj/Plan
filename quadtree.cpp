#include "quadtree.h"
#include <stdexcept>
#include <vector>
#include <stack>
#include <list>

quadtree::quadtree()
{
	child[0] = nullptr;
	child[1] = nullptr;
	child[2] = nullptr;
	child[3] = nullptr;
}

quadtree*& quadtree::operator[](int index)
{
	if (index >= 4 || index < 0)
		throw std::exception("Invalid argument index");
	return child[index];
}

bool quadtree::has_child()
{
	return child[0] != nullptr || child[1] != nullptr || child[2] != nullptr || child[3] != nullptr;
}

void quadtree::init_inner(function<void(quadtree*)> new_node_cb)
{
	for (int i = 0; i < 4; i++) {
		auto inner = new quadtree();
		inner->parent = this;
		switch (i)
		{
		case 0:
			inner->area = QRectF(
				QPointF(area.center().x(), area.top()),
				QPointF(area.right(), area.center().y()));
			break;
		case 1:
			inner->area = QRectF(area.topLeft(), area.center());
			break;
		case 2:
			inner->area = QRectF(area.center(), area.bottomRight());
			break;
		case 3:
			inner->area = QRectF(
				QPointF(area.left(), area.center().y()),
				QPointF(area.center().x(), area.bottom()));
		default:;
		}
		if (new_node_cb != nullptr)
			new_node_cb(inner);
		child[i] = inner;
	}
}

void quadtree::insertblock(double x, double y, int depth, function<void(quadtree*)> new_node_cb)
{
	quadtree* current = this;
	for (;;)
	{
		if (depth == 0)
		{
			current->is_block = true;
			break;
		}
		auto inner_node = current->get_side(x, y);
		if (inner_node == nullptr)
		{
			current->init_inner(new_node_cb);
		}
		current = current->child[current->get_index(x, y)];
		depth -= 1;
	}
}

int quadtree::get_index(double x, double y)
{
	return ((y < area.center().y() ? 0 : 2) + ((x > area.center().x()) ? 0 : 1));
}

quadtree*& quadtree::get_side(double x, double y)
{
	int index = get_index(x, y);
	return child[index];
}

quadtree* quadtree::get_root() const
{
	auto current = parent;
	for (;;)
	{
		if (current->parent == nullptr)
			return current;
		current = current->parent;
	}
}
struct dir_table
{
	/**
	* \brief 方向，需要注意的是，这里direction - 5即为这个斜方向在quadtree::child中的index了
	*/
	enum direction { HALT = 0, R = 1, L = 2, D = 3, U = 4, RU = 5, LU = 6, RD = 7, LD = 8 };
	direction dir;
	int loc;
	static const dir_table table[9][4];
	static direction reverseDir(direction dir)
	{
		static direction rev[] = { HALT,L,R,U,D,LD,RD,LU,RU };
		return rev[static_cast<int>(dir)];
	}
};
const dir_table dir_table::table[9][4] = {
	{},
{/*R*/{ R,1 },{ HALT,0 },{ R,3 },{ HALT,2 } },
{/*L*/{ HALT,1 },{ L,0 },{ HALT,3 },{ L,2 } },
{/*D*/{ HALT,2 },{ HALT,3 },{ D,0 },{ D,1 } },
{/*U*/{ U,2 },{ U,3 },{ HALT,0 },{ HALT,1 } },
{/*RU*/{ RU,3 },{ U,2 },{ R,1 },{ HALT,0 } },
{/*LU*/{ U,3 },{ LU,2 },{ HALT,1 },{ L,0 } },
{/*RD*/{ R,3 },{ HALT,2 },{ RD,1 },{ D,0 } },
{/*LD*/{ HALT,3 },{ L,2 },{ D,1 },{ LD,0 } }
};

void quadtree::enum_neighbor(function<void(quadtree*)> callback)
{
	if (callback == nullptr)
		return;
	quadtree* root = get_root();
	std::list<quadtree*> results;
	struct context
	{
		std::stack<int> inner_loc;
		int relative_level = 0;
		dir_table::direction dir;
		dir_table::direction orig_dir;
		quadtree* current;
	};
	std::stack<context> pending;
	for (int i = 1; i <= 8; i++)
	{
		context ctx;
		ctx.dir = ctx.orig_dir = static_cast<dir_table::direction>(i);
		ctx.current = this;
		pending.push(ctx);
	}
	for (;;)
	{
		if (pending.empty())
			break;
		auto& ctx = pending.top();
		if (ctx.dir == dir_table::HALT)
		{
			for (;;)
			{
				if (ctx.relative_level == 0)
					break;
				if (!ctx.current->has_child())
					break;
				ctx.current = ctx.current->child[ctx.inner_loc.top()];
				ctx.inner_loc.pop();
				ctx.relative_level--;
			}
			if (ctx.relative_level != 0)
			{
				results.push_back(ctx.current);
			}
			else
			{
				if (ctx.orig_dir <= 4)
				{
					const auto dir = dir_table::reverseDir(ctx.orig_dir);
					std::stack<quadtree*> pending_enum;
					pending_enum.push(ctx.current);
					for (;;)
					{
						if (pending_enum.empty())
							break;
						const auto curr = pending_enum.top();
						pending_enum.pop();
						if (curr != nullptr)
						{
							if (curr->has_child())
							{
								static int child_table[4][2] = { { 2,0 },{ 3,1 },{ 3,2 },{ 1,0 } };
								pending_enum.emplace((curr->child[child_table[dir - 1][0]]));
								pending_enum.emplace((curr->child[child_table[dir - 1][1]]));
							}
							else
								results.push_back(curr);
						}
					}
				}
				else
				{
					const auto dir = dir_table::reverseDir(ctx.orig_dir);
					auto curr = ctx.current;
					for (;;)
					{
						if (curr->has_child())
							curr = curr->child[dir - 5];
						else
						{
							results.push_back(curr);
							break;
						}
					}
				}
			}
			pending.pop();
		}
		else
		{
			if (ctx.current->parent == nullptr)
			{
				//已经到根节点，也就是说找不到这个方向的相邻节点
				pending.pop();
			}
			else
			{
				int loc = ctx.current->get_loc();
				ctx.inner_loc.push(dir_table::table[ctx.dir][loc].loc);
				ctx.relative_level++;
				ctx.dir = dir_table::table[ctx.dir][loc].dir;
				ctx.current = ctx.current->parent;
			}
		}
	}
	results.unique();
	for (auto result : results)
		callback(result);
}

int quadtree::get_level() const
{
	int level = 0;
	auto current = parent;
	for (;;)
	{
		if (current->parent == nullptr)
			return level;
		level++;
		current = current->parent;
	}
}

int quadtree::get_loc() const
{
	int loc = 0;
	if (this->parent == nullptr)
		return -1;
	for (int i = 0; i < 4; i++)
	{
		if (this->parent->child[i] == this)
			return i;
	}
	return -1;
}

