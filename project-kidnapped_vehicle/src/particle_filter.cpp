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

#define MIN_YAW 0.00001
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
      };
    );
  }

  // Flag initialized
  initialized = true;
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

  for (auto &prt : num_particles) {
    // Prevent 0 division with 0 yaw_rate
    if(fabs(yaw_rate) >  MIN_YAW) {
      prt.x = prt.x + (velocity/yaw_rate) * (sin(prt.theta+(yaw_rate* delta_t))−sin(prt.theta​));
      prt.y = prt.y + (velocity/yaw_rate) * (cos(prt.theta) - cos(prt.theta+(yaw_rate*delta_t)));
      prt.theta = prt.theta + (yaw_rate*delta_t);
    } else {
      // Simple right angle triangle adjacent or opposite side computation, ratioed by velocity
      prt.x += velocity * delta_t * cos(theta);
      prt.y += velocity * delta_t * sin(theta);
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
  double min_dist = math.INFINITY;
  double rel_x;
  double rel_y;
  double rel_dst;

  for (int i = 0; i<observations.size(); i++) {
    LandmarkObs obs = observations[i]
    rel_x = (obs.x - predicted.x);
    rel_y = (obs.x - predicted.x);
    rel_dst = sqrt(pow(rel_x, 2.0) + pow(rel_y, 2.0));
    if(rel_dst < min_dist) {
      predicted.id = obs.id;
    }
  }
}

void cal_homogenous_transform(double x_part, double y_part, double heading_part, double x_obs, double y_obs) {
  double x_map = x_part + (cos(heading_part) * x_obs) - (sin(heading_part) * y_obs);
  double y_map = y_part + (sin(heading_part) * x_obs) + (cos(heading_part) * y_obs);
  return x_map, y_map;
}

void calc_weight(double sigma_x, double sigma_y, double x_map, double y_map, double x_land, double y_land){
  xomxl = (x_map-x_land);
  yomyl = (y_map-y_land);
  return exp(-(((xomxl*xomxl)/(2*sigma_x*sigma_x)) + ((yomyl*yomyl)/(2*sigma_y*sigma_y)))) / (2*pi*sigma_x*sigma_y);
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


  for (auto &prt : num_particles) {
    for (auto &obs : observations) {
      double x_obs_map = x.prt + (cos(x.theta) * obs.x) - (sin(x.theta) * obs.y);
      double y_obs_map = y.prt + (sin(x.theta) * obs.x) + (cos(x.theta) * obs.y);

    } 
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

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