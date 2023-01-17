#include <iostream>
#include <vector>
#include "Eigen/Dense" // g++ $(pkg-config --cflags eigen3) kalman_filter_2D_linear.cpp

class Kalman{
    public:
    Kalman(int size, double sampling_time, Eigen::MatrixXd& _Ac, Eigen::MatrixXd& _Bc, Eigen::MatrixXd& _Cc, Eigen::MatrixXd& _P, Eigen::MatrixXd& _Q, Eigen::MatrixXd& _R){
        state_size = size;
        dt = sampling_time;

        Ad = _Ac * dt + Eigen::MatrixXd::Identity(size, size);
        Bd = _Bc * dt;
        Cd = _Cc;

        P = _P;
        Q = _Q;
        R = _R;

        state = Eigen::MatrixXd::Zero(size,1);
    }

    void model_prediction(double input){

        state = Ad*state + Bd*input;
        P = Ad*P*Ad.transpose() + Q;

        
        std::cout << "* predict: \n";
        std::cout << "state(position , velocity) is \n";
        std::cout << state << std::endl;
        std::cout << "kalman covariance is \n";
        std::cout << P << std::endl << std::endl;        

    }
    void measure_update(double measure){

        Eigen::MatrixXd meas(1,1);
        meas << measure;
        
        K = P*Cd.transpose()* (Cd*P*Cd.transpose() + R).inverse();
        state = state + K* ( meas - Cd*state);
        P = P - K*Cd*P;

        std::cout << "** update: \n";
        std::cout << "state(position , velocity) is \n";
        std::cout << state << std::endl;
        std::cout << "kalman covariance is \n";
        std::cout << P << std::endl << std::endl;
    }

    private:
    Eigen::MatrixXd state;
    Eigen::MatrixXd Ad;
    Eigen::MatrixXd Bd;
    Eigen::MatrixXd Cd;
    
    double dt;
    int state_size;

    Eigen::MatrixXd P, Q, R;
    Eigen::MatrixXd K;
};

int main(){

    // 칼만 시스템 매트릭스 및 분산값 세팅
    int size = 2;
    Eigen::MatrixXd Ac(size,size);
    Ac << 0,1,0,0;
    Eigen::MatrixXd Bc(size,1);
    Bc << 0,1;
    Eigen::MatrixXd Cc(1,size);
    Cc << 1,0;

    Eigen::MatrixXd P(2,2);
    Eigen::MatrixXd Q(2,2);
    Eigen::MatrixXd R(1,1);
    P = Eigen::MatrixXd::Identity(size,size) * 1000;
    Q = Eigen::MatrixXd::Identity(size,size) * 0;
    R << 1;

    // 칼만 필터 RUN
    Kalman kf(2, 1, Ac, Bc, Cc, P, Q, R);
    std::vector<double> inputs = {2,0,0,0};
    std::vector<double> measures = {1,3,5};
    for (int i = 0 ; i < measures.size() ; ++i){
        std::cout << "=======> Iter: " << i+1 << std::endl;
        kf.model_prediction(inputs[i]);
        kf.measure_update(measures[i]);
    }
    std::cout << "=======> Predict: "  << std::endl;
    kf.model_prediction(inputs.back());

    return 0;
}