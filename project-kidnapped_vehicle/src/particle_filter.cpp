/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

using std::normal_distribution;

#define MIN_YAW 0.0001
// Lesson 4: Particle filters - 12. Creating particles
#define NUM_PARTICLES 1000

void ParticleFilter::init(double x, double y, double theta, double std[]) {

  // DEFINE CONSTANTS
  /** TODO: Set the number of particles */
  num_particles = NUM_PARTICLES; // Number of particles

  /**
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
  */

  // Lesson 5: Implementation of a particle filter - 5. Program Gaussian sampling: Code
  // Initialize generators once
  normal_distribution<double> gen_x(x, std[0]);
  normal_distribution<double> gen_y(y, std[1]);
  normal_distribution<double> gen_theta(theta, std[2]);

  for(int i = 0; i<num_particles; i++) {
    particles.push_back(
      Particle {
        i,
        gen_x(gen),
        gen_y(gen),
        gen_theta(gen),
        1,
        vector<int>(),
        vector<double>(),
        vector<double>()
        }
    );
  }

  // Flag initialized
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  // Initialize generators once
  normal_distribution<double> gen_x(0, std_pos[0]);
  normal_distribution<double> gen_y(0, std_pos[1]);
  normal_distribution<double> gen_theta(0, std_pos[2]);

  // // Lesson 5: Implementation of a particle filter - 8. Calculate Prediction Step Quiz
  // For each particle, calculate prediction
  for (Particle &prt : particles) {
    // Prevent 0 division with 0 yaw_rate
    if(fabs(yaw_rate) > MIN_YAW) {
      prt.x = prt.x + (velocity/yaw_rate) * (sin(prt.theta + (yaw_rate * delta_t)) - sin(prt.theta));
      prt.y = prt.y + (velocity/yaw_rate) * (cos(prt.theta) - cos(prt.theta + (yaw_rate * delta_t)));
      prt.theta = prt.theta + (yaw_rate*delta_t);
    } else {
      // Simple right angle triangle adjacent or opposite side computation, ratioed by velocity
      prt.x += velocity * delta_t * cos(prt.theta);
      prt.y += velocity * delta_t * sin(prt.theta);
      // Yaw unchanged
    }

    // Add noise
    prt.x += gen_x(gen);
    prt.y += gen_y(gen);
    prt.theta += gen_theta(gen);

  }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

  // Declare variables 
  double min_dist;
  double rel_dst;

  // For each observed measurement
  // (sensed by vehicle, as seen from particle POV and, transformed to map coordinates)
  for (LandmarkObs &obs : observations) {
    // Initialize min distance to infinite
    min_dist = INFINITY;

    // Loop through predicted measurements
    for (LandmarkObs &prd : predicted) {

      // Compute distance between observed and predicted
      rel_dst = dist(obs.x, obs.y, prd.x, prd.y);

      // Update closest ID
      if(rel_dst < min_dist) {
        min_dist = rel_dst;
        obs.id = prd.id;
      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a multi-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

  double rel_dst;
  // Landmark Gaussian noise
  double lndmrk_std_x = std_landmark[0];
  double lndmrk_std_y = std_landmark[1];

  // For each particle, find landmarks within sensor range
  for (Particle &prt : particles) {

    // Map landmark locations within particle centric sensor range 
    vector<LandmarkObs> predictions;

    // Find landmarks in range of sensor
    std::vector<Map::single_landmark_s> landmark_list = map_landmarks.landmark_list;
    
    for (Map::single_landmark_s &sng_lndmrk : landmark_list) {

      // Compute relative distance
      rel_dst = dist(prt.x, prt.y, sng_lndmrk.x_f, sng_lndmrk.y_f);

      // Add to prediction if in range
      if (rel_dst < sensor_range) {
        // add prediction to vector
        predictions.push_back(LandmarkObs{ sng_lndmrk.id_i, sng_lndmrk.x_f, sng_lndmrk.y_f });
      }
      
    } 

    // Reset particle weight
    prt.weight = 1.0;
    
    // For each observation
    for (LandmarkObs obs : observations) {
      // Lesson 5: Implementation of a particle filter - 16. Quiz: Landmarks
      // Transform into map coordinates
      // (As if sensed by vehicle at particle POV and, transformed to map coordinates)
      double x_map = prt.x + (cos(prt.theta) * obs.x) - (sin(prt.theta) * obs.y);
      double y_map = prt.y + (sin(prt.theta) * obs.x) + (cos(prt.theta) * obs.y);

      // Lesson 5: Implementation of a particle filter - 18. Quiz: Association
      // Locate closest landmark to observation      
      // Initialize pointer to closest landmark
      LandmarkObs *closest = NULL;
      // Initialize min distance to infinite
      double min_dist = INFINITY;
      
      // Loop through predicted measurements
      for (LandmarkObs &prd : predictions) {

        // Compute distance between observed and predicted
        rel_dst = dist(x_map, y_map, prd.x, prd.y);

        // Update closest landmark data
        if(rel_dst < min_dist) {
          min_dist = rel_dst;
          closest = &prd;
        }
      }
      
      // Lesson 5: Implementation of a particle filter - 20. Particle Weights
      // Compute weight of particle
      // xoxml: X observed minus X landmark, // yoyml: Y observed minus Y landmark
      double xomxl = (x_map - closest->x);
      double yomyl = (y_map - closest->y);
      double obs_weight = exp(-(((xomxl*xomxl)/(2*lndmrk_std_x*lndmrk_std_x)) + ((yomyl*yomyl)/(2*lndmrk_std_y*lndmrk_std_y)))) / (2*M_PI*lndmrk_std_x*lndmrk_std_y);
      prt.weight = prt.weight * obs_weight;
      break;
      
    }

    // Replaced by code above
    // dataAssociation(predictions, map_observations);
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  vector<Particle> resample_prt;

  // Lesson 4: Particle filters - 15. Resampling
  // For each particle, find max weight
  double max_weight = 0;
  for (Particle &prt : particles) {
    if(prt.weight > max_weight) {
      max_weight = prt.weight;
    }
  }
  
  // Create generators
  // Uniform random distribution between 0 and max ID of particles
  std::uniform_int_distribution<int> uni_int_dist(0, num_particles-1);
  // Uniform random distribution between 0.0 and maximum weight
  std::uniform_real_distribution<double> uni_dbl_dist(0.0, max_weight);

  // Generate first index
  int  index = uni_int_dist(gen);

  // Lesson 4: Particle filters - 20. Resampling wheel
  // Resample wheel
  double beta = 0.0;
  for (int i = 0; i < num_particles; i++) {
    // Randomly add weight to beta 
    beta += uni_dbl_dist(gen) * 2.0;
    // Subtract weight and increase index until matching wheel location vs index  is found
    while (beta > particles[index].weight) {
      beta -= particles[index].weight;
      index = (index + 1) % num_particles;
    }
    resample_prt.push_back(particles[index]);
  }

  particles = resample_prt;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}