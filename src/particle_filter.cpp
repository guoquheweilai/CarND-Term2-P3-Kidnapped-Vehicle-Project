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

	/*****************************************************************************
   	*  Initialization
   	****************************************************************************/
	if (!is_initialized_) {
		// Initialize number of particles
		num_particles = 100;

		// Initialize random engine
		default_random_engine gen;

		// This line creates a normal (Gaussian) distribution for x
		normal_distribution<double> dist_x(x, std[0]);

		// TODO: Create normal distributions for y and theta
		normal_distribution<double> dist_y(y, std[1]);
		normal_distribution<double> dist_theta(theta, std[2]);

		// TODO: Sample and from these normal distributions
		int i = 0;
		for (i = 0; i < num_particles; i++) {
			Particle p;

			p.id = i;
			p.x = dist_x(gen);
			p.y = dist_x(gen);
			p.theta = dist_theta(gen);
			p.weight = 1.0;

			particles.push_back(p);
		}

		is_initialized = true;
		return;
	}
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	/*****************************************************************************
   	*  Initialization
   	****************************************************************************/
	// Initialize random engine
	default_random_engine gen;

	// Create normal (Gaussian) distribution for sensor noise
	normal_distribution<double> noise_x(0, std_pos[0]);
	normal_distribution<double> noise_y(0, std_pos[1]);
	normal_distribution<double> noise_theta(0, std_pos[2]);
	
	/*****************************************************************************
   	*  Prediction
   	****************************************************************************/
	// Calculate the new state
	int i = 0;
	for (i = 0; i < num_particles; i++) {
	  // TODO: Check if yaw rate is 0 or close to 0
	  if (fabs(yaw_rate) < 0.00001) {
	    particles[i].x += velocity * delta_t * cos(particles[i].theta);
	    particles[i].y += velocity * delta_t * sin(particles[i].theta);
	  }
	  else {
	    particles[i].x += velocity / yaw_rate * ( sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
	    particles[i].y += velocity / yaw_rate * (-cos(particles[i].theta + yaw_rate * delta_t) + cos(particles[i].theta));
	    particles[i].theta += yaw_rate * delta_t;
	  }
		
	  // TODO: Add noise
	  particles[i].x += noise_x(gen);
	  particles[i].y += noise_y(gen);
	  particles[i].theta += noise_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	
	// TODO: Loop through all observation measurements
	int i = 0, j = 0;
	for (i = 0; i < observations.size(); i++) {
	  // Initialize placeholder for map id from map
	  int id_map = -1;
	  // Initialize the minimum distance with a big number
	  double min_dist = numeric_limits<double>::max();  
		
	  // TODO: Loop through all predicted landmarks
	  for (j = 0; j < predicted.size(); j++) {
	    // Calculate the distance between observation measurements and predicted landmarks
	    double dist_obs_land = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
	    
	    // Update the map id and minimum distance when a smaller distance was founded
	    if (dist_obs_land < min_dist) {
	      id_map = predicted[j].id;
	      min_dist = dist_obs_land;
	    }
	  }
	
	  // Update the observation's id with the nearest predicted landmark
	  observations[i].id = id_map;
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
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
	
	int i = 0, j = 0, k = 0;
	for (i = 0; i < num_particles; i++) {
	/*****************************************************************************
   	*  Filter landmarks
   	****************************************************************************/
	// TODO: Find landmarks within sensor range of particle
	  // Create vector for storing filtered landmarks
	  vector<LandmarkObs> vec_IR_landmarks;
	
	  // Calculate sensor range as radius square
	  double radius_square = sensor_range * sensor_range;
	  
	  // Loop through all the landmarks
	  for (j = 0; j < ; j++) {
	    // Extract data for better readability
	    int   id_landmark = map_landmarks.landmark_list[j].id_i;
	    float x_landmark  = map_landmarks.landmark_list[j].x_f;
	    float y_landmark  = map_landmarks.landmark_list[j].y_f;
		  
	    // Calculate the distance between observation measurement and landmark
	    double delta_x = particles[i].x - x_landmark;
	    double delta_y = particles[i].y - y_landmark;
	    
	    // Check if it is in range then push it to the end of the filter list
	    if (delta_x * delta_x + delta_y * delta_y <= radius_square) {
	      vec_IR_landmarks.push_back(LandmarkObs{id_landmark, x_landmark, y_landmark});
	    }
	  }
	
	/*****************************************************************************
   	*  Transform coordinates
   	****************************************************************************/
	// TODO: Transform from vehicle coordinates to map coordinates
	  // Create vector for storing transformed observation measurements
	  vector<LandmarkObs> vec_transformed_obs;
	  
	  // Loop through all observation measurements
	  for (j = 0; j < observations.size(); j++) {
	    double map_x = p_x + cos(particles[i].theta) * observations[j].x - sin(particles[i].theta) * observations[j].y;
	    double map_y = p_y + sin(particles[i].theta) * observations[j].x + cos(particles[i].theta) * observations[j].y;
	
	    vec_transformed_obs.push_back(LandmarkObs{observations[j].id, map_x, map_y});
	  }
		
	/*****************************************************************************
   	*  Identify the observation measurements
   	****************************************************************************/
	// TODO: Association the observation measurements with the predicted landmarks
	  // Run data association with predicted landmarks and transformed observation measurements
	  dataAssociation(vec_IR_landmarks, vec_transformed_obs);
		
	/*****************************************************************************
   	*  Calculate weight
   	****************************************************************************/
	// TODO: Calculate weight with mult-variate Gaussian distribution
	  // Reinitialize weight
	  particles[i].weight = 1.0;
	  
	  // Calculate normalization term
	  double gauss_norm = 1 / ( 2 * M_PI * std_landmark[0] * std_landmark[1]);
		
	  // Loop through all transformed observation measurements
	  for (j = 0; j < vec_transformed_obs.size(); j++) {
	    // Define inputs
	    double obs_x  = vec_transformed_obs[j].x;
	    double obs_y  = vec_transformed_obs[j].y;
	    int    obs_id = vec_transformed_obs[j].id;
	    
	    // Find x, y of predicted landmark with matched id
	    bool flag_match = false;
	    while (!flag_match && k < vec_IR_landmarks.size()) {
	      if (vec_IR_landmarks[k] == obs_id) {
	        landmark_x = vec_IR_landmarks[k].x;
		landmark_y = vec_IR_landmarks[k].y;
	        flag_match = true;
	      }
	      k++;
	    }
	    
	    // Calculate exponent
	    double exponent = ((obs_x - landmark_x)**2) / (2 * std_landmark[0]**2) + ((obs_y - landmark_y)**2) / (2 * std_landmark[1]**2);
		  
	    // Calculate weight using normalization terms and exponent
	    double w = gauss_norm * math.exp(-exponent);
		  
	    // Check if w is 0 or close to 0
	    if (fabs(w) < 0.00001) {
	      // Multiply with minimum number
	      particles[i].weight *= 0.00001;
	    }
	    else {
	      // Multiply with calculated weight
	      particles[i].weight *= w;
	    }
	  }
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
