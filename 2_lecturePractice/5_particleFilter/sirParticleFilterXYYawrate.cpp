/**
 * @file sirParticleFilterXYYawrate.cpp
 * @brief NonLinear Case, X = [종거리, 종속도, 횡거리, 횡속도, 요레이트]
 * @brief Z = [종거리, 횡거리]
 * @brief For analysis, file in & out is available
 * @name Sungwook Lee (joker1251@naver.com)
 * @date 2023-01-20
 */

#include <iostream>
#include "Eigen/Dense" // g++ $(pkg-config --cflags eigen3) sirParticleFilterXYYawrate.cpp
#include <thread>
#include <chrono>
#include <iomanip>
#include <vector>
#include <random>
#include <algorithm>
#include <numeric>

#include <fstream> // for csv file read
#include <string>  // for csv file read

using namespace std;
#define SAMPLING_MS 100
#define SHOWING_NUMBER_OF_PARTICLES 5

std::random_device rd;
std::mt19937 gen(rd());
normal_distribution<double> randN(0, 1);
uniform_real_distribution<double> randU(0.0, 1.0);

class FileReader
{
public:
    FileReader(string _file)
    {
        string line, data;
        stream.open(_file);

        if (stream.is_open())
        {
            while (getline(stream, line))
            {
                vector<double> row;
                replace(line.begin(), line.end(), ',', ' ');
                istringstream linestream(line);

                while (linestream >> data)
                {
                    row.push_back(stod(data));
                }
                parsed.push_back(row);
            }
        }
        stream.close();
    }
    void print()
    {
        for (auto row : parsed)
        {
            for (auto col : row)
                cout << col << " ";
            cout << endl;
        }
    }
    vector<vector<double>> parsed;

private:
    ifstream stream;
};

class Particle
{
public:
    Particle(int num_state, int num_particles, Eigen::VectorXd init_X)
    {
        X = init_X.array() + randN(gen);
        W = 1.0 / num_particles;
        Z = Eigen::VectorXd::Zero(2); // output state의 개수가 2개여서 2로 하드코딩함
    }
    int id;            // particle ID
    double W;          // particle weight
    Eigen::VectorXd X; // state vector
    Eigen::VectorXd Z; // output vector
};

class ParticleFilterSIR
{
public:
    ParticleFilterSIR(int _num_particles, bool _makeFileFlag = false)
    {
        Q = Eigen::MatrixXd::Zero(1, 1);
        R = Eigen::MatrixXd::Zero(1, 1);
        B = Eigen::MatrixXd::Zero(5, 2);
        U = Eigen::MatrixXd::Zero(5, 1);
        C = Eigen::MatrixXd::Zero(2, 5);
        X = Eigen::MatrixXd::Zero(5, 1);
        Z = Eigen::MatrixXd::Zero(2, 1);

        X << 0, 10, 0, 10, 1;
        Q << 0.1;
        R << 0.1;
        B << 0.5 * dt * dt, 0,
            dt, 0,
            0, 0.5 * dt * dt,
            0, dt,
            0, 0;
        U << 0, 0, 0, 0, 1;
        C << 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0;

        num_states = X.rows();
        num_particles = _num_particles; // 예제에선 500;

        particles = vector<Particle>(num_particles, Particle(num_states, num_particles, X));
        for (int i = 0; i < particles.size(); ++i)
        {
            particles[i].id = i;
            particles[i].X = X.array() + randN(gen);
        }

        makeFileFlag = _makeFileFlag;
        if (makeFileFlag) // for export file
        {
            writeFile.open("output.csv", ios::out);
            writeFile << "X_pred"
                      << ", X_true"
                      << ", dotX_pred"
                      << ", dotX_true"
                      << ", Y_pred"
                      << ", Y_true"
                      << ", dotY_pred"
                      << ", dotY_true"
                      << ", YawRate_pred"
                      << ", YawRate_true" << endl;
        }
        cout << "[1] CONSTRUCT \n";
    }

    void sampling()
    {

        Eigen::MatrixXd _u = Eigen::MatrixXd::Zero(1, 1) * Q; // 각속도 제어입력, Q를 그냥 분산이면서 표준편차라 보겠다.
        Eigen::MatrixXd _q = Eigen::MatrixXd::Zero(2, 1) * Q; // 속도 제어입력, Q를 그냥 분산이면서 표준편차라 보겠다.
        Eigen::MatrixXd _r = Eigen::MatrixXd::Zero(2, 1) * R; // 관측 값, R를 그냥 분산이면서 표준편차라 보겠다.

        for (int i = 0; i < particles.size(); ++i)
        {
            _u << randN(gen) * Q;
            _q << randN(gen) * Q, randN(gen) * Q;
            _r << randN(gen) * R, randN(gen) * R;

            particles[i].X = process(particles[i].X, _u, _q);
            particles[i].Z = outputEq(particles[i].X, _r);
            particles[i].id = i; // 새로운 id 부여
        }

        cout << "[2] Sampling with Predict \n";
    }

    double weight(Eigen::VectorXd _Z, Eigen::VectorXd _Zp)
    {
        double _w = 0;
        Eigen::MatrixXd _SigR = Eigen::MatrixXd::Zero(2, 2);
        _SigR << R, 0, 0, R;

        _w = 1.0 / sqrt((2 * M_PI * _SigR).determinant()) * exp(-0.5 * ((_Zp - _Z).transpose() * _SigR.inverse() * (_Zp - _Z))(0));
        return _w;
    }

    void resampling(vector<Particle> _particles)
    {
        vector<Particle> _Pold = _particles;

        vector<double> _prob;
        double cumsum = 0;
        for (int i = 0; i < num_particles; ++i)
        {
            cumsum += _Pold[i].W;
            _prob.push_back(cumsum);
        }

        for (int i = 0; i < num_particles; ++i)
        {
            double rand_sample = randU(gen);
            for (int j = 0; j < num_particles; ++j)
            {
                if (rand_sample < _prob[j])
                { // 0~1 사이에서 뽑은 값이 누적합 배열보다 작은 시점의 선택된 j를 파티클로 선택하여 넣어준다.
                    particles[i] = _Pold[j];
                    break;
                }
            }
        }
    }

    void update(Eigen::VectorXd _Z)
    {

        double sum_W = 0;
        for (int i = 0; i < particles.size(); ++i)
        {
            double _W = weight(_Z, particles[i].Z);
            sum_W += _W;
            particles[i].W = _W;
        }
        for (auto &particle : particles)
        {
            particle.W = particle.W / sum_W;
        }
        cout << "[3-1] Update Weight \n";

        resampling(particles);
        cout << "[3-2] Update with Resampling \n";

        Eigen::VectorXd X_Particles_SUM = Eigen::VectorXd::Zero(5);
        for (auto particle : particles)
        {
            X_Particles_SUM += particle.X;
        }
        X = X_Particles_SUM / particles.size(); // New mean
    }

    Eigen::VectorXd process(Eigen::VectorXd _Xold, Eigen::MatrixXd _u, Eigen::MatrixXd _q)
    {
        double w = _Xold(4);
        Eigen::VectorXd _X = Eigen::VectorXd::Zero(_Xold.rows());

        Eigen::MatrixXd _F = Eigen::MatrixXd::Zero(_Xold.rows(), _Xold.rows());

        _F << 1, sin(w * dt) / w, 0, -(1 - cos(w * dt) / w), 0,
            0, cos(w * dt), 0, -sin(w * dt), 0,
            0, (1 - cos(w * dt) / w), 1, sin(w * dt) / w, 0,
            0, sin(w * dt), 0, cos(w * dt), 0,
            0, 0, 0, 0, 1;
        _X = _F * _Xold + B * _q + U * _u;

        return _X;
    }

    Eigen::VectorXd outputEq(Eigen::VectorXd _X, Eigen::MatrixXd _r)
    {
        Eigen::VectorXd _Z = Eigen::VectorXd::Zero(_r.rows());
        _Z = C * _X + _r;

        return _Z;
    }

    void monitoring(vector<double> ref)
    {
        cout << "[4] Monitoring(" << setw(3) << now << "s): " << endl;
        cout << "State: " << endl
             << X << endl;
        cout << "Error: " << abs(ref[0] - X(0)) << ", " << abs(ref[1] - X(1)) << ", " << abs(ref[2] - X(2)) << ", "
             << abs(ref[3] - X(3)) << ", " << abs(ref[4] - X(4)) << endl;

        cout << "Particle(ID, Weight): ";
        for (int i = 0; i < num_particles; ++i)
        {
            if (i >= SHOWING_NUMBER_OF_PARTICLES)
                break;
            else
                cout << "(" << particles[i].id << ", " << particles[i].W << "), ";
        }
        cout << endl;

        if (makeFileFlag)
        {
            writeFile << X(0) << ", " << ref[0] << ", " << X(1) << ", " << ref[1] << ", " << X(2) << ", " << ref[2]
                      << ", " << X(3) << ", " << ref[3] << ", " << X(4) << ", " << ref[4] << endl;
        }
        now += dt;
    }

    ~ParticleFilterSIR()
    {
        if (makeFileFlag)
            writeFile.close(); // for export file
    }

private:
    Eigen::MatrixXd Q, R, B, U, C, X, Z;

    double dt = SAMPLING_MS / 1000.0;
    double now = 0;
    int num_particles, num_states;
    vector<Particle> particles;

    ofstream writeFile; // for export file
    bool makeFileFlag;  // for export file
};

int main()
{
    FileReader reader("inputRef_XYYawrate.csv");
    vector<vector<double>> Ref = reader.parsed;

    FileReader reader2("inputSensor_XYYawrate.csv");
    vector<vector<double>> Sensor = reader2.parsed;

    bool makeFileFlag = false;
    cout << "Export the result as CSV file: Yes(1), No(0): ";
    cin >> makeFileFlag;

    int num_particles = 1;
    cout << "Insert the number of particles(1~500): ";
    cin >> num_particles;

    ParticleFilterSIR filter(num_particles, makeFileFlag);

    Eigen::VectorXd _Z = Eigen::VectorXd::Zero(2);
    for (int col = 0; col < Sensor[0].size(); ++col)
    {
        _Z << Sensor[0][col], Sensor[1][col];
        filter.sampling();
        filter.update(_Z);
        filter.monitoring({Ref[0][col], Ref[1][col], Ref[2][col], Ref[3][col], Ref[4][col]});
    }

    return 0;
}
