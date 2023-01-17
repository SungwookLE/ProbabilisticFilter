#include <iostream>
#include <vector>

template <typename T>
class Kalman{
    public:
    Kalman(T covariance_measurement_, T covariance_prediction_){
        covariance_measurement = covariance_measurement_;
        covariance_prediction = covariance_prediction_;
    }

    void measure_upate(T mean_measure){
        // 베이즈 법칙에 따라 조건부확률에 따라 업데이트 된다.
        T mean_now = mean;
        T covariance_now = covariance;


        T new_covariance = 1/(1/covariance_now+ 1/covariance_measurement );
        T new_mean = (mean_now * 1/covariance_now + mean_measure*1/covariance_measurement)
                     / (1/new_covariance);

        mean = new_mean;
        covariance = new_covariance;

        std::cout << "update: " << "[" << mean << "," <<covariance << "]" << std::endl;

    }

    void model_prediction(T input){
        //input 이 들어옴에 따라 전확률 법칙에 의거하여 전체 분산은 (+)가 된다.
        mean = mean + input;
        covariance = covariance + covariance_prediction ;

        std::cout << "predict: " << "[" << mean << "," <<covariance << "]" << std::endl;
    }

    private:
    T covariance_measurement;
    T covariance_prediction;
    T mean=0;
    T covariance=1000.0;
};

int main(){

    std::vector<double> inputs={1,1,2,1,1};
    std::vector<double> measurements={5,6,7,9,10};

    double measure_cov = 4;
    double model_cov = 2;

    Kalman<double> kf(measure_cov, model_cov);

    for(int i =0 ; i < measurements.size(); ++i){
        kf.measure_upate(measurements[i]);
        kf.model_prediction(inputs[i]);
    }

    return 0;
}