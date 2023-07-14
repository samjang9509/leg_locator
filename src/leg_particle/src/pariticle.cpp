#include "leg_particle/particle.hpp"


cv::Point2f Particle::Target(std::vector<cv::Point2f> _laser_pt)
{
	cv::Point2f target_coor;
	float t_distance = 10000.0;

	for (auto iter : _laser_pt)
	{

		float tmp_distance = sqrt(pow(iter.x, 2.0) + pow(iter.y, 2.0));

		if (abs(iter.y) < 500.0 && iter.x > 0.0)
		{
			if (tmp_distance < t_distance)
			{
				target_coor.x = iter.x;
				target_coor.y = iter.y;
				t_distance = tmp_distance;
			}
		}
	}

	actual_target_coordinate = pt2Coor(target_coor.x, target_coor.y);

	// caught_target = true;
	return actual_target_coordinate;
}

void Particle::initiate()
{
	parts.resize(particle_num);
	weights.resize(particle_num);

	std::random_device rd;
	std::mt19937 gen(rd());

	std::uniform_real_distribution<float> coor_distribution(0.0, 1000.0);
	std::uniform_real_distribution<float> theta_distribution(-180.0, 180.0);

	for (int i = 0; i < particle_num; i++)
	{
		p.id = i;
		p.x = coor_distribution(gen);
		p.y = coor_distribution(gen);
		p.theta = theta_distribution(gen);
		p.weight = 0.0;
		parts[i] = p;
		weights[i] = p.weight;
	}
}

void Particle::motion(std::vector<Particles> &smp_particle)
{
	int size = smp_particle.size();

	std::random_device rd;
	std::mt19937 gen(rd());

	std::uniform_real_distribution<float> gaussian_random(-10.0, 10.0);

	for (int i = 0; i < size; i++)
	{
		p.id = i;
		p.x = smp_particle[i].x + gaussian_random(gen);
		p.y = smp_particle[i].y + gaussian_random(gen);
		p.theta = smp_particle[i].theta;
		p.weight = 1.0 / size;
		// p.weight = 0.0;
		parts[i] = p;
		weights[i] = p.weight;
	}
}

void Particle::addWeight(std::vector<Particles> &smp_particle)
{
	int size = smp_particle.size();
	float total_weight = 0.0f;
	float distance;

	for (int i = 0; i < size; i++)
	{
		distance = sqrt(pow((smp_particle[i].x - actual_target_coordinate.x), 2) + pow((smp_particle[i].y - actual_target_coordinate.y), 2));
		if (distance < 20.0 && actual_target_coordinate.x != 0.0)
		{
			p.weight = smp_particle[i].weight + 1;
			weights[i] = p.weight;
		}
		total_weight = total_weight + weights[i];

		if (total_weight > 1 && caught_target == false)
		{
			caught_target = true;
		}
	}
	std::cout << "total_weight = " << total_weight << std::endl;

	for (int j = 0; j < size; j++)
	{
		p.weight = weights[j] / total_weight;
		// std::cout << "particle weight for " << j << " = " << p.weight << std::endl;
		parts[j].weight = p.weight;
		weights[j] = p.weight;
	}

}

void Particle::resampling(std::vector<Particles> &smp_particle)
{
	int size = smp_particle.size();
	float checkWeight;
	float weightSum;

	std::vector<Particles> tmp_particle;
	std::vector<float> new_weight;
	std::vector<float> cumm_weight;

	tmp_particle.resize(size);
	new_weight.resize(size);
	cumm_weight.resize(size);

	std::random_device rd;
	std::mt19937 gen(rd());

	std::uniform_real_distribution<float> rnd_weight(0.0, 1.0);
	std::uniform_real_distribution<float> gaussian_random(-10.0, 10.0);

	if (caught_target == true)
	{

		for (int i = 0; i < size; i++)
		{
			checkWeight = rnd_weight(gen);
			for (int j = 0; j < size; j++)
			{
				weightSum += smp_particle[j].weight;

				if (checkWeight <= weightSum)
				{
					std::cout << "weight sum for " << j << " = " << weightSum << std::endl;
					std::cout << "PARTICLE WEIGHT " << j << " = " << smp_particle[j].weight << std::endl;
					std::cout << "PARTICLE WEIGHT " << j - 1 << " = " << smp_particle[j - 1].weight << std::endl;
					p.id = j;
					p.x = smp_particle[j].x;
					p.y = smp_particle[j].y;
					p.weight = smp_particle[j].weight;
					parts[i] = p;
					weightSum = 0.0f;
				}
			}
		}
		tmp_particle.clear();
	}
	else if(caught_target == false)
	{
		for (int i = 0; i < size; i++)
		{
			p.id = i;
			p.x = smp_particle[i].x + gaussian_random(gen);
			p.y = smp_particle[i].y + gaussian_random(gen);
			p.theta = smp_particle[i].theta;
			p.weight = 1.0 / size;
			// p.weight = 0.0;
			parts[i] = p;
			weights[i] = p.weight;
		}
	}
}

float Particle::gaussian_weight(double p_coordinate, float target_coordinate)
{
	float px;

	float constant;
	float exponential;

	constant = (1 / (sqrt(2 * CV_PI)));

	exponential = exp(-(pow((p_coordinate - target_coordinate), 2) / 2));

	px = constant * exponential;

	return px;
}

cv::Point2f Particle::pt2Coor(float x_co, float y_co)
{
	float grid_x = (grid_row / 2) - (y_co * mm2pixel);
	float grid_y = (grid_col / 2) - (x_co * mm2pixel);

	cv::Point2f coordinate(grid_x, grid_y);
	return coordinate;
}

cv::Point2f Particle::mean_point(std::vector<Particles> &smp_particle)
{
	cv::Point2f mean_coor;
	int size = smp_particle.size();
	int total_x = 0;
	int total_y = 0;

	for (auto iteration : smp_particle)
	{
		total_x = total_x + iteration.x;
		total_y = total_y + iteration.y;
	}

	mean_coor.x = total_x / size;
	mean_coor.y = total_y / size;

	return mean_coor;
}