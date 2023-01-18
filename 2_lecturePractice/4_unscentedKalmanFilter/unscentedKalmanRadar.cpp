/**
 * @file unscentedKalmanRadar.cpp
 * @brief NonLinear Case, X = [종거리, 종속도, 횡거리]
 * @brief Z = 유클리드 거리
 * @brief For analysis, file in & out is available
 * @name Sungwook Lee (joker1251@naver.com)
 * @date 2023-01-18
 */

#include <iostream>
#include "Eigen/Dense" // g++ $(pkg-config --cflags eigen3) unscentedKalmanRadar.cpp
#include <thread>
#include <chrono>
#include <iomanip>
#include <vector>
#include <fstream> // for csv file read
#include <string>  // for csv file read

using namespace std;
#define SAMPLING_MS 50

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

class UnscentedKalman
{
public:
    UnscentedKalman(double _Kappa, bool _makeFileFlag = false)
    {
        P = Eigen::MatrixXd::Zero(3, 3);
        Q = Eigen::MatrixXd::Zero(3, 3);
        R = Eigen::MatrixXd::Zero(1, 1);
        X = Eigen::MatrixXd::Zero(3, 1);

        P = Eigen::MatrixXd::Identity(3, 3) * 10;
        Q << 0, 0, 0, 0, 0.001, 0, 0, 0, 0.001;
        R << 10;
        X << 0, 90, 1100;

        Ad = Eigen::MatrixXd::Zero(3, 3);
        Bd = Eigen::MatrixXd::Zero(3, 1);
        
        Ad << 1, dt, 0,
              0,  1, 0,
              0,  0, 1;

        Kappa = _Kappa;
        Xi = Eigen::MatrixXd::Zero(num_state, 2*num_state+1);
        W = Eigen::MatrixXd::Zero(2*num_state +1, 1);

        makeFileFlag = _makeFileFlag;
        if (makeFileFlag) // for export file
        {
            writeFile.open("output.csv", ios::out);
            writeFile << "COV_X,"
                      << "COV_dotX,"
                      << "COV_Y,"
                      << "X,"
                      << "dotX,"
                      << "Y,"
                      << "Error X,"
                      << "Error dotX,"
                      << "Error Y" << endl;
        }

    }

    void predict()
    {
        X = Ad * X; // 6x1 + 6x2 x 2x1
        P = Ad * P * Ad.transpose() + Q;

    }

    void sigmaPoints_weightSelect(){
        
        Xi.col(0) = X.col(0);
        W(0) = Kappa / (num_state + Kappa);

        Eigen::LLT<Eigen::MatrixXd> U((num_state + Kappa) * P);
        Eigen::MatrixXd U_ = U.matrixL();


        //right
        for(int j =0 ; j < num_state; ++j){
            Xi.col(j+1) = X + U_.row(j).transpose();
            W(j+1) = 1/(2*(num_state + Kappa));
        }

        //left
        for(int j =0 ; j <num_state; ++j){
            Xi.col(j+num_state+1) = X - U_.row(j).transpose();
            W(j+num_state+1) = 1/(2*(num_state + Kappa));
        }

    }

    std::vector<Eigen::MatrixXd> unscentedTransform(Eigen::MatrixXd outX, Eigen::MatrixXd noiseCov){

        int n_state = outX.rows();
        int n_weight = outX.cols();

        Eigen::MatrixXd Xm = Eigen::MatrixXd::Zero(n_state, 1);
        for (int k = 0 ; k < n_weight; ++k){
            Xm.col(0) = Xm.col(0)+ W(k) * outX.col(k);
        }

        Eigen::MatrixXd Xcov = Eigen::MatrixXd::Zero(n_state, n_state);
        for (int k =0 ; k < n_weight ; ++k){
            Xcov = Xcov+ W(k) * (outX.col(k) - Xm.col(0)) * (outX.col(k) - Xm.col(0)).transpose();
        }

        Xcov = Xcov + noiseCov;

        return {Xm, Xcov};
    }

    void update(double z1)
    {
        Eigen::MatrixXd Z(1, 1);
        Z << z1;

        Eigen::MatrixXd hXi = Eigen::MatrixXd::Zero(1, 2*num_state+1);
        for (int k =0 ; k < (2*num_state+1); ++k ){
            hXi.col(k) = NonlinearH(Xi.col(k));
        }

        vector<Eigen::MatrixXd> ret = unscentedTransform(hXi, R);
        Zp = ret[0].col(0);
        Pz = ret[1];

        Pxz = Eigen::MatrixXd(num_state, 1);
        for (int k = 0 ; k < (2*num_state +1); ++k){
            Pxz = Pxz + W(k)*(Xi.col(k) - X) * (hXi.col(k) - Zp).transpose();
        }

        K = Pxz * Pz.inverse();

        X = X + K * (Z - NonlinearH(X));
        P = P - K * Pz * K.transpose();
    }


    Eigen::MatrixXd NonlinearH(Eigen::MatrixXd _X)
    {
        Eigen::MatrixXd ret(1, 1);
        ret << sqrt(_X(0) * _X(0) + _X(2) * _X(2));
        return ret;
    }

    void monitoring(vector<double> ref)
    {
        cout << "Monitoring(" << setw(3) << now << "s): " << endl;
        cout << "Covariance: " << endl
             << P << endl;
        cout << "State: " << endl
             << X << endl;
        cout << "Error: " << abs(ref[0] - X(0)) << ", " << abs(ref[1] - X(1)) << ", " << abs(ref[2] - X(2))
             << endl;

        if (makeFileFlag)
        {
            writeFile << P(0, 0) << ", " << P(1, 1) << ", " << P(2, 2) << ", " << X(0) << ", " << X(1) << ", " << X(2)
                      << ", " << abs(ref[0] - X(0)) << ", " << abs(ref[1] - X(1)) << ", " << abs(ref[2] - X(2)) << endl;
        }
        now += dt;
    }

    ~UnscentedKalman()
    {
        if (makeFileFlag)
            writeFile.close(); // for export file
    }

private:
    Eigen::MatrixXd P, Q, R;
    Eigen::MatrixXd Pz, Pxz;

    Eigen::MatrixXd Ad, Bd;
    Eigen::MatrixXd X, Zp;
    Eigen::MatrixXd Xi, W;
    Eigen::MatrixXd K;

    double Kappa;
    int num_state = 3;
    double dt = SAMPLING_MS / 1000.0;
    double now = 0;
    ofstream writeFile; // for export file
    bool makeFileFlag;  // for export file
};

int main()
{
    FileReader reader("inputRef_Radar.csv");
    vector<vector<double>> Ref = reader.parsed;

    FileReader reader2("inputSensor_Radar.csv");
    vector<vector<double>> Sensor = reader2.parsed;

    bool makeFileFlag = false;
    cout << "Export the result as CSV file: Yes(1), No(0): ";
    cin >> makeFileFlag;

    UnscentedKalman filter(0.01, makeFileFlag);

    for (int col = 0; col < Sensor[0].size(); ++col)
    {
        filter.predict();
        filter.sigmaPoints_weightSelect();
        this_thread::sleep_for(chrono::milliseconds(SAMPLING_MS));
        filter.update(Sensor[0][col]);
        
        vector<double> RefCol = {Ref[0][col], Ref[1][col], Ref[2][col]};
        filter.monitoring(RefCol);
    }
    return 0;
}
