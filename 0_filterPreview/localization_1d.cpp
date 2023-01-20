/**
 * @file localization_1d.cpp
 * @author sungwook LEE
 * @date 2023-01-20
 * @brief Marcov Bayes
 * @copyright Copyright (c) 2023
 */


#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>

class helper{
    public:
    constexpr static double STATIC_ONE_OVER_SQRT_2PI = 1/std::sqrt((double)2*M_PI);
    static double normpdf(double x, double mu, double std){
        return (STATIC_ONE_OVER_SQRT_2PI/std)*exp(-0.5*pow((x-mu)/std,2));
    }
    
    //static function to normalize a vector:
	static std::vector<double> normalize_vector(std::vector<double> inputVector){

		//declare sum:
		double sum = 0.0f;

		//declare and resize output vector:
		std::vector<double> outputVector ;
		outputVector.resize(inputVector.size());

		//estimate the sum:
		for (unsigned int i = 0; i < inputVector.size(); ++i) {
			sum += inputVector[i];
		}

		//normalize with sum:
		for (unsigned int i = 0; i < inputVector.size(); ++i) {
			outputVector[i] = inputVector[i]/sum;
		}
		//return normalized vector:
		return outputVector;
	}
};

class localization_1D{
    public:
    localization_1D(int map): map_size(map){
        distance_max=map_size;

        priors.resize(map_size, 0);
        posteriors.resize(map_size,0);

        initialize_priors();
    }
    int map_size;
    std::vector<double> priors;
    std::vector<double> posteriors;

    /**
     * @brief 
     * distance_max in 1D map
     */
    double distance_max;
    
    /**
     * @brief 
     * define landmarks
     */
    std::vector<double> landmark_positions = {3, 9, 14, 23};

    /**
     * @brief 
     * Set standard deviation of control/observation
     */
    double control_stdev = 1.0;
    double observation_stdev = 1.0;
    double position_stdev = 1.0;

    /**
     * @brief 
     * meters vehicle moves per time step
     */
    double movement_per_timestep = 1.0;

    /**
     * @brief 
     * uniform distribution for initialization prior
     * initialize priors assuming vehicle at landmark +/- 1.0 meters position stddev
     * 랜드 마크 옆에 있을 확률이 높다는 가정 하에 아래와 같이 distribution 한건데, 꼭 이렇게 할 필요는 없다.
     */
    void initialize_priors(){
        // set each landmark position +/- 1 
        double norm_term = landmark_positions.size() * (position_stdev*2+1);
        for(int i = 0; i < landmark_positions.size() ; ++i){
            for(int j =1 ; j <= position_stdev; ++j){
                priors[int(landmark_positions[i]+j+map_size)%map_size] +=1.0/norm_term;
                priors[int(-j+landmark_positions[i]+map_size)%map_size] +=1.0/norm_term;
            }
            priors[landmark_positions[i]] += 1.0/ norm_term;
        }
    }

    /**
     * @brief `predict`
     * motion model: 
     * calculates prob of being at an estimated position at time t
     */
    double motion_model(double pseudo_position, double movement){
        double position_prob =0.0;
        for(int i =0 ; i < map_size; ++i){
            double distance = pseudo_position  -double(i)- movement;
            double transition_prob = helper::normpdf(distance, 0.0, control_stdev);
            // 전확률 법칙에 의거하여 다 더해주는 것
            position_prob += transition_prob * priors[i];
        }
        return position_prob;
    }

    /**
     * @brief `update`
     * declare pseudo_range_estimator function
     * calculate range from psuedo postion to each randmark
     */
    std::vector<double> pseudo_range_estimator(double pseudo_position){
        std::vector<double> res;
        for(int i=0 ; i < landmark_positions.size() ; ++i){
            double pseudo_range;
            if ( landmark_positions[i] > pseudo_position){
                pseudo_range = landmark_positions[i] - pseudo_position;
                res.push_back(pseudo_range);
            }
        }
        std::sort(res.begin(), res.end());
        return res;
    }

    /**
     * @brief `update`
     * get observation probability
     * matching from observation and pseudo range to landmark
     * @param observation sensor measurement
     * @param pseudo_ranges ranges between pseudo point to landmark
     */
    double observation_model(std::vector<double> observations, std::vector<double> pseudo_ranges){
        
        double prob=1;        
        
        for(int i =0 ; i < observations.size() ; ++i){

            double x = distance_max;
            if (pseudo_ranges.size() != 0){
                double min_dist = pseudo_ranges[0];
                pseudo_ranges.erase(pseudo_ranges.begin());
                x = observations[i] - min_dist;
            }

            double prob_single =helper::normpdf(x, 0.0, observation_stdev);
            // 업데이트 곱해주는 단계: main문에서 모델 결과와도 곱하는 과정이 있다. 
            prob*=prob_single;
        }

        return prob;        
    }
};

int main(){
    // number of x positions on map (1D)
    int map_size = 25;
    localization_1D bayes(map_size);
    double movement_per_timestep = 1.0;
    
    std::vector<std::vector<double>> sensor_obs = {{1,7,12,21}, {0,6,11,20}, {5,10,19},
                                     {4,9,18}, {3,8,17}, {2,7,16}, {1,6,15}, 
                                     {0,5,14}, {4,13}, {3,12}, {2,11}, {1,10},
                                     {0,9}, {8}, {7}, {6}, {5}, {4}, {3}, {2},
                                     {1}, {0}, {}, {}, {}};
    std::vector<double> obs_data;
    for(auto obs : sensor_obs){
        if ( !obs.empty() )
            obs_data = obs; 
        else
            obs_data = {double(map_size)};


        double movement = bayes.movement_per_timestep;
        for(int i = 0 ; i < map_size; ++i){
            double pseudo_position = double(i);
            double motion_prob = bayes.motion_model(pseudo_position, movement);
            
            std::vector<double> pseudo_ranges = bayes.pseudo_range_estimator(pseudo_position);
            double observation_prob = bayes.observation_model(obs_data, pseudo_ranges);

            // 업데이트 곱해주는 단계: 최종적인 posterior고, 아직 노말라이제이션 전단계
            bayes.posteriors[i] = motion_prob*observation_prob;
        }
        bayes.posteriors = helper::normalize_vector(bayes.posteriors);
        bayes.priors = bayes.posteriors;
        
        // print posteriors vectors to stdout
        for (int p = 0; p < bayes.posteriors.size(); ++p) {
          std::cout << bayes.posteriors[p] << std::endl;  
        }
        std::cout << std::endl;
    }
    return 0;
}