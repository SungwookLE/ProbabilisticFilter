#include <iostream>
#include <vector>
#include "Eigen/Dense"

class Unscented_KF{
    public:
        /**
         * @brief Construct a new Unscented_KF object
         * 
         * @param sampling_time 
         * @param _kappa 
         * @param _P 
         * @param _Q 
         * @param _R 
         * @param _num_state 
         * @param _num_measured 
         */
        Unscented_KF(double sampling_time, double _kappa, Eigen::MatrixXd _P, Eigen::MatrixXd _Q, Eigen::MatrixXd _R, int _num_state, int _num_measured ){
            dt = sampling_time;
            P = _P;
            Q = _Q;
            R = _R;
            kappa = _kappa;

            num_state = _num_state;
            num_measured = _num_measured;

            X = Eigen::VectorXd::Zero(num_state);

            std::cout << "Constructor\n";
        }
        /**
         * @brief Step1. SigmaPoints and Weights Selection based on P(Kalman Covariance)
         * 
         */
        void SigmaPoints_WeightSelect(){
            Xi = Eigen::MatrixXd::Zero(num_state, 2*num_state + 1); // state 마다 2*num_state + 1 개의 샘플을 생성
            W  = Eigen::MatrixXd::Zero(2*num_state+1, 1);

            for(int i = 0; i < num_state ; ++i){
                Xi(i, 0) = X(i);
            }
            W(0) = kappa / (num_state + kappa);

            Eigen::LLT<Eigen::MatrixXd> _U((num_state+kappa)*P); // 칼만시스템의 현재 분산에 근거하여 새로운 샘플의 분포 정도와 가중치를 계산
            Eigen::MatrixXd U_ = _U.matrixL();

            // right
            for(int j =0 ; j < num_state; ++j){
                Xi.col(j+1) = X + U_.row(j).transpose();
                W(j+1) = 1/(2*(num_state + kappa));
            }

            // left
            for(int j =0 ; j < num_state; ++j){
                Xi.col(j+num_state+1) = X - U_.row(j).transpose();
                W(j+num_state+1) = 1/(2*(num_state + kappa));
            }

            std::cout << "[1] SigmanPoints and WeightSelect\n";
        }
        /**
         * @brief Step2. Predict Process: Sigmapoints are calculated with non-linear function, and new mean and deviation are calculated using UnscentedTransform
         * 
         * @param measured 
         */
        void Predict(Eigen::VectorXd measured){
            fXi = Eigen::MatrixXd::Zero(num_state, 2*num_state+1);

            for(int k = 0 ; k < (2*num_state+1); ++k){
                fXi.col(k) = system_process(Xi.col(k), measured);
            }

            std::vector<Eigen::MatrixXd> ret = UnscentedTransform(fXi, Q);
            X = ret[0].col(0);
            P = ret[1];

            std::cout << "[2] Predict\n";

        }
        /**
         * @brief Step3. Kalman Gain is Calculated. Note that zp(output equation) also could be calculated with non-linearity
         * 
         */
        void KalmanGainCalculation(){
            hXi = Eigen::MatrixXd::Zero(num_measured, 2*num_state+1);

            for(int k = 0 ; k <(2*num_state+1); ++k){
                hXi.col(k) = output_equation(fXi.col(k));
            }

            std::vector<Eigen::MatrixXd> ret = UnscentedTransform(hXi, R);

            zp = ret[0].col(0);
            Pz = ret[1];
            Pxz = Eigen::MatrixXd(num_state, num_measured);
            
            for(int k= 0; k < 2*num_state+1 ; ++k)
                Pxz = Pxz + W(k) * (fXi.col(k) - X) * (hXi.col(k) - zp).transpose();

            K   = Pxz*Pz.inverse();

            std::cout << "[3] KalmanGain\n";
        }
        /**
         * @brief Step4. Update: correct state and kalman covariance using calculated Kalman gain.
         * 
         * @param measured 
         */
        void Update(Eigen::VectorXd measured){

            X = X + K* (measured - zp);
            P = P - K * Pz * K.transpose();

            std::cout << "[4] Update(Estimated): \n";
            std::cout << " 4-1) State: \n";
            std::cout << "  - Roll is " << X(0) << std::endl;
            std::cout << "  - Pitch is " << X(1) << std::endl;
            std::cout << "  - Yaw is " << X(2) << std::endl;
            std::cout << " 4-2) Covariance: \n";
            std::cout << P << std::endl;

        };

    private:
        double dt;
        double kappa;
        Eigen::MatrixXd P, Q, R;
        Eigen::MatrixXd K;
        Eigen::VectorXd X, zp;
        Eigen::MatrixXd Pz, Pxz;
        int num_state, num_measured;

        /**
         * @brief Sigma Points and Weight 
         */
        Eigen::MatrixXd Xi;
        Eigen::MatrixXd W;

        /**
         * @brief 
         */
        Eigen::MatrixXd fXi, hXi;


        /**
         * @brief non-linear system model, transition function
         * 
         * @param X_in 
         * @param measured 
         * @return Eigen::VectorXd 
         */
        Eigen::VectorXd system_process(Eigen::VectorXd X_in, Eigen::VectorXd measured){
            double phi = X_in(0), theta = X_in(1);

            double p = 0, q=0, r=0;
            if (num_measured ==2){
                p = measured(0);
                q = measured(1);
            }
            else if (num_measured==3){
                p = measured(0);
                q = measured(1);
                r = measured(2);
            }

            double x_dot1, x_dot2, x_dot3;

            x_dot1 = p + q*sin(phi) * tan(theta) + r*cos(phi)*tan(theta);
            x_dot2 =     q*cos(phi)              - r*sin(phi);
            x_dot3 =     q*sin(phi)*(1.0)/cos(theta) + r*cos(phi)*(1.0)/cos(theta);

            Eigen::VectorXd Xdot = Eigen::VectorXd::Zero(num_state);
            Xdot << x_dot1, x_dot2, x_dot3;

            Eigen::VectorXd Xp = X + Xdot * dt;
            return Xp;
        };
        
        /**
         * @brief output_equation
         * 
         * @param X_in 
         * @return Eigen::VectorXd 
         */
        Eigen::VectorXd output_equation(Eigen::VectorXd X_in){
            // double yp1, yp2, yp3;
            // yp1 = X_in(0);
            // yp2 = X_in(1);
            // yp3 = X_in(2);
            Eigen::VectorXd Yp = Eigen::VectorXd::Zero(num_measured);
            for(int i =0 ; i < num_measured ; ++i )
                Yp(i) = X_in(i);

            return Yp;
        }

        /**
         * @brief UnscentedTransform could express new mean and deviation using sigma points
         * 
         * @param Xi_in 
         * @param noiseCov 
         * @return std::vector<Eigen::MatrixXd> 
         */
        std::vector<Eigen::MatrixXd> UnscentedTransform(Eigen::MatrixXd Xi_in , Eigen::MatrixXd noiseCov){
            int n = Xi_in.rows();
            int kmax = Xi_in.cols();

            Eigen::MatrixXd xm = Eigen::MatrixXd::Zero(n,1);
            for (int k =0 ; k < kmax; ++k){
                xm.col(0) = xm.col(0) + W(k) * Xi_in.col(k);
            }


            Eigen::MatrixXd xcov = Eigen::MatrixXd::Zero(n, n);
            for (int k =0 ; k < kmax ; ++k){
                xcov = xcov + W(k) * (Xi_in.col(k) - xm.col(0))* (Xi_in.col(k)-xm.col(0)).transpose(); 
            }

            xcov = xcov + noiseCov;
            std::vector<Eigen::MatrixXd> ret = {xm, xcov};
            return ret;
        }
};

int main(){
    // measured data for testing
    std::vector<std::vector<double>> measured_data= {{0,0,0},{1,1,1},{2,2,2},{3,3,3}};
    double sampling_time = 0.01;
    double kappa = 3.0; 
    int num_state = 3;
    int num_measured = 2;

    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(num_state, num_state);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(num_state, num_state);
    Q << 0.0001, 0, 0,
         0, 0.0001, 0,
         0, 0,   1;
        
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(num_measured,num_measured);
    R << 6, 0,  
         0, 12;

    Unscented_KF UKF(sampling_time, kappa, P, Q, R, num_state, num_measured);
    for (int sequence =0 ; sequence < measured_data.size() ; ++sequence){
        std::vector<double> one_measure = measured_data[sequence];
        Eigen::VectorXd measured = Eigen::VectorXd(num_measured);
        for (int put = 0 ; put < num_measured; ++put)
            measured(put) = one_measure[put];


        // Kalman Iterative Process
        UKF.SigmaPoints_WeightSelect();
        UKF.Predict(measured);
        UKF.KalmanGainCalculation();
        UKF.Update(measured);
    }

    return 0;
}