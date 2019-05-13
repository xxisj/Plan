#include "psocore.h"
#include <random>
#include <algorithm>

pso_core::pso_core()
{

}

pso_core::~pso_core()
{
	delete[] g_best;
}

double pso_core::calc()
{
	if (g_best != nullptr)
	{
		delete[] g_best;
	}
	g_best = new double[dim]{0};
	data_.clear();
	{
		particle_data empty(dim);
		for (int i = 0; i < size; i++)
		{
			data_.push_back(empty);
		}
	}
	auto rand = init_rng();
	double error = std::numeric_limits<double>::max();

	//initialize value
	for(auto& element : data_)
	{
		for(int d = 0; d < dim; d++)
		{
			double low = range_limit[d].first;
			double high = range_limit[d].second;
			double a = low + (high - low) * rand();
			double b = max_velocity * ( rand() - 0.5) * 2;
			element.data[d] = a;
			element.best_data[d] = a;
			element.velocity[d] = b;// (a - b) / 2;
		}
		element.fitness = obj_func_(element.data, dim);
		element.best_fitness = element.fitness;
		if(error > element.fitness)
		{
			error = element.fitness;
			memcpy(g_best, element.data, sizeof(double) * dim);
		}
	}
	double prev_error = 0;
	int low_delta_cnt = 0;
	double rho1, rho2;
	for(int step = 0; ; step++)
	{
		if(fabs(prev_error - error) <= goal)
		{
			low_delta_cnt++;
			//Solved.
			if(low_delta_cnt > steps)
				break;
		}
		else
		{
			low_delta_cnt = 0;
		}
		prev_error = error;

		//update all particles
		for(auto& element : data_)
		{
			//update particle position
			for(int d = 0; d < dim; d++)
			{
				rho1 = c1 * rand();
				rho2 = c2 * rand();
				element.velocity[d] = element.velocity[d]  * inertia(step)
					+ rho1 * (element.best_data[d] - element.data[d]) 
					+ rho2 * (g_best[d] - element.data[d]);
				auto range = range_limit[d].second - range_limit[d].first;
				element.velocity[d] = 
					std::max(-range, 
						std::min(range, element.velocity[d]));

				element.data[d] += element.velocity[d];
				if(range_limit[d].first > element.data[d])
				{
					element.data[d] = range_limit[d].first;
					element.velocity[d] *= -1;
				}
				else if(range_limit[d].second < element.data[d])
				{
					element.data[d] = range_limit[d].second;
					element.velocity[d] *= -1;
				}
			}
			
			//update particle fitness
			element.fitness = obj_func_(element.data, dim);
			if(element.fitness < element.best_fitness)
			{
				element.best_fitness = element.fitness;
				memcpy(element.best_data, element.data, sizeof(double) * dim);
			}
			if(element.fitness < error)
			{
				error = element.fitness;
				memcpy(g_best, element.data, sizeof(double) * dim);
			}
		}

		//printf("Step %d (w=%.2f) :: min err=%.5e\n", step, 1.0f, error);
	}
	data_.clear();
	return error;
}

void pso_core::set_obj_function(std::function<double(double*, int)> func)
{
	obj_func_ = func;
}

std::function<double()> pso_core::init_rng()
{
	std::default_random_engine rand(static_cast<unsigned long>(seed + 1));
	std::uniform_real_distribution<double> dist(0, 1);
	//return [=]()->double {return dist(rand); };
	return std::bind([](std::default_random_engine& rand, std::uniform_real_distribution<double>& dist) {return dist(rand); }, rand, dist);
}

double pso_core::inertia(int round) const
{
	return inertia_min + (inertia_max - inertia_min) * (std::max(0, 100 - round) * 0.01f);
}


pso_core::particle_data::particle_data()
{
	dim = -1;
	data = nullptr;
	best_data = nullptr;
	velocity = nullptr;
	best_fitness = fitness = std::numeric_limits<double>::max();
}

pso_core::particle_data::particle_data(int dim)
{
	this->dim = dim;
	data = new double[dim]{0};
	best_data = new double[dim] {0};
	velocity = new double[dim]{0};
	best_fitness = fitness = std::numeric_limits<double>::max();
}

pso_core::particle_data::particle_data(const particle_data& src)
{
	dim = src.dim;
	data = new double[dim];
	best_data = new double[dim];
	velocity = new double[dim];
	memcpy(data, src.data, sizeof(double) * dim);
	memcpy(best_data, src.best_data, sizeof(double) * dim);
	memcpy(velocity, src.velocity, sizeof(double) * dim);
	fitness = src.fitness;
	best_fitness = src.best_fitness;
}

pso_core::particle_data::particle_data(particle_data&& src)
{
	dim = src.dim;
	data = src.data;
	velocity = src.velocity;
	best_data = src.best_data;
	src.dim = -1;
	src.best_data = nullptr;
	src.data = nullptr;
	src.velocity = nullptr;
	fitness = src.fitness;
	best_fitness = src.best_fitness;
	src.best_fitness = src.fitness = std::numeric_limits<double>::max();
}

pso_core::particle_data::~particle_data()
{
	delete[] data;
	delete[] velocity;
	delete[] best_data;
}


