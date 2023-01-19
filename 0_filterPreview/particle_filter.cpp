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

double ParticleFilter::multiv_prob(double sig_x, double sig_y, double x_obs, double y_obs,
                   double mu_x, double mu_y) {
  // calculate normalization term
  double gauss_norm;
  gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

  // calculate exponent
  double exponent;
  exponent = (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2)))
               + (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));
    
  // calculate weight using normalization terms and exponent
  double weight;
  weight = gauss_norm * exp(-exponent);
    
  return weight;
}

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 20;  // TODO: Set the number of particles
  std::random_device rd;
  std::mt19937 gen(rd());

  std::normal_distribution<double> dist_x(0, std[0]);
  std::normal_distribution<double> dist_y(0, std[1]);
  std::normal_distribution<double> dist_theta(0, std[2]);
  
  for (int i = 0; i < num_particles; ++i){
    Particle particle;
    particle.id = i + 1;
    particle.x = x+dist_x(gen);
    particle.y =  y+dist_y(gen);
    particle.theta = fmod(theta+dist_theta(gen), 2*M_PI);
    particle.weight = 1;
    particles.push_back(particle);
    
  }

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

  // xf = x0 + v/yaw_rate * ( sin(yaw0 + yaw_rate * delta_t) - sin(yaw0))
  // yf = y0 + v/yaw_rate * ( cos(yaw0) - cos(yaw0 + yawrate * delta_t ))
  // yawf = yaw0 + yawrate*delta_t
  std::random_device rd;
  std::mt19937 gen(rd());

  std::normal_distribution<double> dist_x(0, std_pos[0]);
  std::normal_distribution<double> dist_y(0, std_pos[1]);
  std::normal_distribution<double> dist_theta(0, std_pos[2]);

  if (fabs(yaw_rate)> 0.000001){
    for (int i = 0; i < num_particles ; ++i){
      particles.at(i).x = particles[i].x + velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta)) + dist_x(gen);
      particles.at(i).y = particles[i].y + velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t)) + dist_y(gen);
      particles.at(i).theta = fmod(particles[i].theta + yaw_rate * delta_t + dist_theta(gen), 2*M_PI);
    }
  }
  else{
    for (int i = 0; i < num_particles ; ++i){
      particles.at(i).theta = fmod(particles[i].theta+ dist_theta(gen), 2*M_PI);
      particles.at(i).x = particles.at(i).x + (velocity * delta_t * cos(particles[i].theta))+ dist_x(gen);
      particles.at(i).y = particles.at(i).y + (velocity * delta_t * sin(particles[i].theta))+ dist_y(gen);
      std::cout << "Too small yaw rate input! " << std::endl;
    }
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
  float distance;
  
  for (int i = 0; i < observations.size(); ++i)
  {
    float prev_dist=10000.0;
    for (int j = 0; j < predicted.size(); ++j){
      distance = dist(predicted[j].x, predicted[j].y , observations[i].x, observations[i].y);
      //((predicted[j].x - observations[i].x) * (predicted[j].x - observations[i].x) + (predicted[j].y - observations[i].y) * (predicted[j].y - observations[i].y));
      if (distance< prev_dist){
        observations[i].id = predicted[j].id;
        prev_dist = distance;
      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
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

  
  for (int n = 0; n < num_particles; ++n){
    std::vector<LandmarkObs> transformed_observations{};
    std::vector<LandmarkObs> candidated_landmarks{};


    // STEP1. transformed the observation data
    for (int i = 0; i < observations.size(); ++i){
      LandmarkObs transformed_observation;
      
      transformed_observation.x = cos(particles[n].theta) * observations[i].x - sin(particles[n].theta) * observations[i].y + particles[n].x;
      transformed_observation.y = sin(particles[n].theta) * observations[i].x + cos(particles[n].theta) * observations[i].y + particles[n].y;

      transformed_observations.push_back(transformed_observation);
    }

    

    // STEP2. data association(Nearest Neighbor)
    for (int j = 0; j < map_landmarks.landmark_list.size(); ++j){
      if( dist(particles[n].x, particles[n].y, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f ) <=sensor_range ){
      //if( (fabs(particles[n].x - map_landmarks.landmark_list[j].x_f ) <= sensor_range ) && ( fabs(particles[n].y - map_landmarks.landmark_list[j].y_f ) <= sensor_range ) ){
        LandmarkObs candidate_landmark;
        candidate_landmark.id = map_landmarks.landmark_list[j].id_i;
        candidate_landmark.x = map_landmarks.landmark_list[j].x_f;
        candidate_landmark.y = map_landmarks.landmark_list[j].y_f;
        candidated_landmarks.push_back(candidate_landmark);
      }
    }


    
    if (candidated_landmarks.size() > 0 ){
      dataAssociation(candidated_landmarks, transformed_observations);
      particles[n].weight = 1; // re-initialization
      
      // STEP3. Update the weights of each particle using a mult-variate Gaussian distribution.
      for (int k = 0; k < transformed_observations.size(); ++k){
        for ( int p =0; p < candidated_landmarks.size() ; ++p){
          if ( candidated_landmarks[p].id ==  transformed_observations[k].id){
            particles[n].weight *= multiv_prob(std_landmark[0], std_landmark[1], transformed_observations[k].x, transformed_observations[k].y, candidated_landmarks[p].x, candidated_landmarks[p].y);
          }
        }
      }
    }
    else{
      std::cout << "NO Detected Landmark in Sensor Range !!!!" << std::endl;
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

  std::random_device rd;
  std::mt19937 gen(rd());

  float sum = 0;
  std::vector<float> weights_for_resample;
  for (int j = 0; j < num_particles; ++j)
  {
    sum += particles[j].weight;
    weights_for_resample.push_back(particles[j].weight);
  }

  std::vector<Particle> samp_particles;
  // (1) Using resampling_wheel method
  /*
  std::uniform_real_distribution<double> rand(0, 1);
  float beta = 0;
  int index = int(rand(gen) * num_particles);

  for (int i = 0; i < num_particles; ++i)
  {
    beta += rand(gen) * 2.0 * std::max_element(particles.begin(), particles.end(), [](Particle a, Particle b) { return (a.weight < b.weight); })->weight;

    while( particles[index].weight < beta ){
      beta = beta - particles[index].weight;
      index = (index + 1) % num_particles;
    }

    samp_particles.push_back( particles[index]);
  }
  */

  // (2) Using discrete_distribution
  std::discrete_distribution<> rand(weights_for_resample.begin(), weights_for_resample.end());
  for (int q = 0; q < num_particles; ++q){
    int index = rand(gen);
    samp_particles.push_back(particles[index]);
  }
  particles = samp_particles;

  // if (fabs(sum) > 3.4E-35 ) {
  //   for (int k = 0; k < num_particles; ++k){
  //     particles[k].weight = particles[k].weight / sum;
  //   }   
  // }
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
