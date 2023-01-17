/**
 * @file kalmanLinearX.cpp
 * @brief Example1, 
 * @brief Linear Case, CONSTANT ACCELERATION MODEL with 1DOF, X = [position ,velocity, acceleration], z = position, all states are observable.
 * @date 2023-01-17
 */

#include <iostream>
#include "Eigen/Dense" // g++ $(pkg-config --cflags eigen3) kalman.cpp
#include <thread>
#include <chrono>
#include <iomanip>
#include <vector>

using namespace std;

class Kalman
{
public:
    Kalman()
    {
        P = Eigen::MatrixXd::Identity(3, 3) * 100;
        Q = Eigen::MatrixXd::Identity(3, 3);
        R = Eigen::MatrixXd::Identity(1, 1) * 0.0001;
        X = Eigen::MatrixXd::Random(3, 1);

        Ad = Eigen::MatrixXd::Zero(3, 3);
        Bd = Eigen::MatrixXd::Zero(3, 1);
        Cd = Eigen::MatrixXd::Zero(1, 3);

        Ad << 1, dt, dt * dt / 2,
            0, 1, dt,
            0, 0, 1;
        Bd << dt * dt / 2, dt, 1;
        Cd << 1, 0, 0;
    }

    void predict(double _u)
    {
        u = _u;
        X = Ad * X + Bd * u;
        P = Ad * P * Ad.transpose() + Q;
    }

    void update(double measurement)
    {
        Eigen::MatrixXd M(1, 1);
        M << measurement;

        K = P * Cd.transpose() * (Cd * P * Cd.transpose() + R).inverse();
        X = X + K * (M - Cd * X);
        P = P - K * Cd * P;
    }

    void monitoring(vector<double> ref)
    {
        cout << "Monitoring: " << endl;
        cout << "Covariance: " << endl
             << P << endl;
        cout << "State: " << X(0) << ", " << X(1) << ", " << X(2) << endl;
        cout << "Error: " << abs(ref[0] - X(0)) << ", " << abs(ref[1] - X(1)) << ", " << abs(ref[2] - X(2)) << endl;
    }

private:
    Eigen::MatrixXd P, Q, R;
    Eigen::MatrixXd Ad, Bd, Cd;
    Eigen::MatrixXd X;
    Eigen::MatrixXd K;

    double u;
    double dt = 0.1;
};

int main()
{
    // matlab 데이터
    vector<double> Sensor_data = {0.11, 0.19, 0.31, 0.43, 0.45, 0.61, 0.65, 0.82, 0.92, 1.1, 1.2, 1.14, 1.22, 1.31, 1.42, 1.61, 1.75, 1.4, 1.92, 2.1, 2.2, 2.18, 2.22, 2.31, 2.45, 2.61, 2.68, 2.81, 2.91, 3.0};
    vector<vector<double>> Ref_data = {{0.1, 1, 0}, {0.2, 1, 0}, {0.3, 1, 0}, {0.4, 1, 0}, {0.5, 1, 0}, {0.6, 1, 0}, {0.7, 1, 0}, {0.8, 1, 0}, {0.9, 1, 0}, {1.0, 1, 0}, {1.1, 1, 0}, {1.2, 1, 0}, {1.3, 1, 0}, {1.4, 1, 0}, {1.5, 1, 0}, {1.6, 1, 0}, {1.7, 1, 0}, {1.8, 1, 0}, {1.9, 1, 0}, {2.0, 1, 0}, {2.1, 1, 0}, {2.2, 1, 0}, {2.3, 1, 0}, {2.4, 1, 0}, {2.5, 1, 0}, {2.6, 1, 0}, {2.7, 1, 0}, {2.8, 1, 0}, {2.9, 1, 0}, {3.0, 1, 0}};

    Kalman filter;

    for (int idx = 0; idx < Sensor_data.size(); ++idx)
    {
        filter.predict(0);
        filter.update(Sensor_data[idx]);
        filter.monitoring(Ref_data[idx]);
    }

    return 0;
}
