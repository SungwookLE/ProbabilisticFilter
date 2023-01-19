/**
 * @file sirParticFilterXYYawrate.cpp
 * @brief NonLinear Case, X = [종거리, 종속도, 횡거리, 횡속도, 요레이트]
 * @brief Z = [종거리, 횡거리]
 * @brief For analysis, file in & out is available
 * @name Sungwook Lee (joker1251@naver.com)
 * @date 2023-01-19
 */

#include <iostream>
#include "Eigen/Dense" // g++ $(pkg-config --cflags eigen3) sirParticFilterXYYawrate.cpp
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

std::random_device rd;
std::mt19937 gen(rd());
normal_distribution<double> randN(0,1.0);
uniform_int_distribution<int> randInt(0, 499); // 예제에서 500개여서 일단 하드코딩함

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



class Particle{
    public:
    Particle(int num_state, int num_particles){
        X = Eigen::VectorXd::Zero(num_state).array() + randN(gen) ; //element-wise adding
        W = 1.0 / num_particles;
        Z = Eigen::VectorXd::Zero(2); // output state의 개수가 2개여서 2로 하드코딩함
    }
    int id; // particle ID
    double W; // particle weight
    Eigen::VectorXd X; // state vector
    Eigen::VectorXd Z; // output vector
};



class ParticleFilterSIR
{
public:
    ParticleFilterSIR(int _num_particles, bool _makeFileFlag=false){
        Q = Eigen::MatrixXd::Zero(1,1);
        R = Eigen::MatrixXd::Zero(1,1);
        B = Eigen::MatrixXd::Zero(5,2);
        U = Eigen::MatrixXd::Zero(5,1);
        C = Eigen::MatrixXd::Zero(2,5);
        X = Eigen::MatrixXd::Zero(5,1);
        Z = Eigen::MatrixXd::Zero(2,1);

        Q << 0.1;
        R << 1;
        B << 0.5*dt*dt, 0, 
            dt, 0, 
            0, 0.5*dt*dt, 
            0, dt, 
            0, 0;
        U << 0, 0, 0, 0, 1;
        C << 1, 0, 0, 0, 0, 
             0, 0, 1, 0, 0;

        num_states = X.rows();
        num_particles=_num_particles; // 예제에선 500;
        X_Particles = Eigen::MatrixXd::Zero(num_states, num_particles);
        Z_Particles = Eigen::MatrixXd::Zero(C.rows(), num_particles);

        particles = vector<Particle>(num_particles, Particle(num_states, num_particles));
        for(int i = 0 ; i < particles.size(); ++i){
            particles[i].id = i;
        }

        makeFileFlag = _makeFileFlag;
        if (makeFileFlag) // for export file
        {
            writeFile.open("output.csv", ios::out);
            writeFile <<  endl;
        }

        cout << "[1] CONSTRUCT \n";
    }

    void sampling(){
        
        Eigen::MatrixXd _u = Eigen::MatrixXd::Zero(1,1) * Q; // 각속도 제어입력, Q를 그냥 분산이면서 표준편차라 보겠다.
        Eigen::MatrixXd _q = Eigen::MatrixXd::Zero(2,1) * Q; // 속도 제어입력, Q를 그냥 분산이면서 표준편차라 보겠다.
        Eigen::MatrixXd _r = Eigen::MatrixXd::Zero(2,1) * R; // 관측 값, R를 그냥 분산이면서 표준편차라 보겠다.
         
        _u << randN(gen)*Q;
        _q << randN(gen)*Q, randN(gen)*Q;
        _r << randN(gen)*R, randN(gen)*R;


        for(int i = 0 ; i <particles.size(); ++i){
            X_Particles.col(i) = process(particles[i].X, _u, _q);
            Z_Particles.col(i) = outputEq(X_Particles.col(i), _r);
            particles[i].X = X_Particles.col(i);
            particles[i].Z = Z_Particles.col(i);
        }

        cout << "[2] Sampling with Predict \n";

    }

    double weight(Eigen::VectorXd _Z, Eigen::VectorXd _Zp){
        double _w = 0;
        Eigen::MatrixXd _SigR = Eigen::MatrixXd::Zero(2,2);
        _SigR << R, 0, 0, R;

        _w = 1 / sqrt((2*M_PI *_SigR).determinant()) * exp(  -0.5* ((_Zp - _Z).transpose() * _SigR.inverse() * (_Zp - _Z))(0)    );
        return _w;
    }

    void resampling(Eigen::MatrixXd _X_Particles){

        Eigen::MatrixXd _Xold = _X_Particles;
        vector<Particle> _Pold = particles;

        vector<double> _prob;
        vector<int> _rNums;
        double cumsum = 0;
        for(int i = 0 ; i < particles.size(); ++i){
            cumsum += particles[i].W;
            _prob.push_back(cumsum);
            _rNums.push_back(randInt(gen));

            cout << _prob[i] << endl; // (1/19, cumsum 이상하네.. 해결하자)
        }

        sort(_rNums.begin(), _rNums.end());

        int fitIn = 0, newIn = 0;

        while(newIn <= num_particles){
            if (_rNums[newIn] <= _prob[fitIn]){
                X_Particles.col(newIn) = _Xold.col(fitIn);
                particles[newIn] =  _Pold[fitIn];
                newIn += 1;
            }
            else{
                fitIn += 1;
            }

        }

        cout << "[3-1] Resampling \n";
    }

    void update(Eigen::VectorXd _Z){

        sum_W=0;
        for(int i = 0; i< Z_Particles.cols(); ++i){
            double _W = weight(_Z, Z_Particles.col(i));
            sum_W += _W;
            particles[i].W = _W;
        }

        for(auto particle: particles)
            particle.W = particle.W / sum_W;
        
        resampling(X_Particles);
        X = X_Particles.colwise().mean();

        cout << "[3] Update with Resampling \n";
        cout << X << endl;

    }


    Eigen::VectorXd process(Eigen::VectorXd _Xold, Eigen::MatrixXd _u, Eigen::MatrixXd _q){

        double w = _Xold(4);
        Eigen::VectorXd _X = Eigen::VectorXd::Zero(_Xold.rows());
        

        Eigen::MatrixXd _F = Eigen::MatrixXd::Zero(_Xold.rows(), _Xold.rows());

        _F << 1, sin(w*dt)/w, 0, -(1-cos(w*dt)/w), 0,
              0, cos(w*dt), 0, -sin(w*dt), 0,
              0, (1-cos(w*dt)/w), 1, sin(w*dt)/w, 0,
              0, sin(w*dt), 0, cos(w*dt), 0,
              0, 0, 0, 0, 1;
        
        _X = _F*_Xold + B*_q + U *_u;
        
        return _X;
    }

    Eigen::VectorXd outputEq(Eigen::VectorXd _X, Eigen::MatrixXd _r){
        Eigen::VectorXd _Z = Eigen::VectorXd::Zero(_r.rows());
        _Z = C * _X + _r;

        return _Z;
    }
    



   

   void monitoring(vector<double> ref)
    {
        cout << "Monitoring(" << setw(3) << now << "s): " << endl;
        cout << "State: " << endl << X << endl;

        if (makeFileFlag)
        {
            writeFile << endl;
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
    Eigen::MatrixXd X_Particles, Z_Particles;


    double dt = SAMPLING_MS / 1000.0;
    double now = 0;
    int num_particles, num_states;
    double sum_W;
    vector<Particle> particles;
    
    
    
    ofstream writeFile; // for export file
    bool makeFileFlag;  // for export file
};

int main()
{
    FileReader reader("inputRef_XYYawrate.csv");
    vector<vector<double>> Ref = reader.parsed;

    FileReader reader2("inputSensors_XYYawrate.csv");
    vector<vector<double>> Sensor = reader2.parsed;

    // bool makeFileFlag = false;
    // cout << "Export the result as CSV file: Yes(1), No(0): ";
    // cin >> makeFileFlag;
    
    ParticleFilterSIR filter(500);

    Eigen::VectorXd _Z = Eigen::VectorXd::Zero(2);
    for(int col = 0 ; col < Sensor[0].size(); ++col){
        _Z << Sensor[0][col], Sensor[1][col];
        filter.sampling();
        filter.update(_Z);
    }



    return 0;
}
