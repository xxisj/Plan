#ifndef PSOCORE_H
#define PSOCORE_H
#include <memory>
#include <vector>
#include <functional>

class pso_core
{
public:
	struct particle_data
	{
	public:
		particle_data();
		particle_data(int dim);
		particle_data(const particle_data& src);
		particle_data(particle_data&& src);
		short dim;
		double* data;
		double* velocity;
		double fitness;
		double best_fitness;
		double* best_data;
		~particle_data();
	};
	std::vector<std::pair<double, double>> range_limit;
	double goal = 1e-5; // optimization goal (error threshold)
	int size = 148; // swarm size (number of particles)
	int print_every = 0; // ... N steps (set to 0 for no output)
	int steps = 20; // maximum number of iterations
	double c1 = 1.49445; // cognitive coefficient
	double c2 = 1.49445; // social coefficient
	int dim;
	long seed; // seed for the generator
	double inertia_min = 0.729;
	double inertia_max = 1.2;
	double max_velocity = 1.0;

	double* g_best = nullptr;
    pso_core();
	~pso_core();
	double calc();
	void set_obj_function(std::function<double(double*, int)> func);
	virtual std::function<double()> init_rng();
	inline double inertia(int round) const;
private:
	std::vector<particle_data> data_;
	std::function<double(double*, int)> obj_func_;
};

#endif // PSOCORE_H
