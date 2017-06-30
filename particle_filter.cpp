/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 1000; //used quiz number of particles, less is faster
	default_random_engine gen; //used for random sample generation

	//Given standard deviations
	double std_x = std[0];
	double std_y = std[1];
	double std_theta = std[2];

	//Normal distributions, mean around measured, don't have to add in later
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	//Initialize all of the particles that I will be using
	for (int i = 0; i < num_particles; i++)
	{
		Particle my_particle; //create a new particle structure each pass
		my_particle.id = i; //id the particle
		my_particle.x = dist_x(gen);
		my_particle.y = dist_y(gen);
		my_particle.theta = dist_theta(gen);
		my_particle.weight = 1.0;  //told to set to 1

		//Vector of all of the particles
		particles.push_back(my_particle); //like append but for vectors
	}

	//????????????????????????????????????????????????????????????????weights have be the same as num_particles resize maybe

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	default_random_engine gen; //used for random sample generation

	//Given standard deviations GPS measurement uncertainty [x [m], y [m], theta [rad]]
	double std_xNoise = std_pos[0];
	double std_yNoise = std_pos[1];
	double std_thetaNoise = std_pos[2];

	//Normal distributions for noise mean of zero, cause we are adding it in later
	normal_distribution<double> dist_xNoise(0.0, std_xNoise);
	normal_distribution<double> dist_yNoise(0.0, std_yNoise);
	normal_distribution<double> dist_thetaNoise(0.0, std_thetaNoise);

	//Predict the particles
	for (int i = 0; i < num_particles; i++)
	{

		particles[i].theta = particles[i].theta + (yaw_rate *  delta_t); //predict theta, x, and y
		
		//Avoid the divide by zero thing //if less than .5 degree, just assume half/more noise
		if (fabs(yaw_rate < 0.01))
		{
			yaw_rate = 0.01;
		}
		particles[i].x = particles[i].x + ((velocity / yaw_rate) * (sin(particles[i].theta + (yaw_rate * delta_t)) - sin(particles[i].theta))) + dist_xNoise(gen); //math from quiz
		particles[i].y = particles[i].y + ((velocity / yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + (yaw_rate * delta_t)))) + dist_yNoise(gen); //math from quiz
	}
}


void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	//???So dont' do this???
	//this is like the nearest neighbor thing


}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
	std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	//Find each particles weight, oh man I have like 1000 of them

	//Transform and Translate each of the incomming observations

	//Associate each transformered observation to a landmark

	//Probability of each observation and then multiply all together





}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	//The wheel thing

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations = associations;
	particle.sense_x = sense_x;
	particle.sense_y = sense_y;

	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
	copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1);  // get rid of the trailing space
	return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
	copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1);  // get rid of the trailing space
	return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
	copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1);  // get rid of the trailing space
	return s;
}
