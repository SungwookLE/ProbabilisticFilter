/**
 * @file unscentedKalmanRadar.cpp
 * @brief NonLinear Case, X = [종거리, 종속도, 횡거리]
 * @brief Z = 유클리드 거리
 * @brief For analysis, file in & out is available
 * @brief Pxz 0으로 초기화 안해줘서 무슨 문제가 하고 수식 다 들여봄... 새벽 2시까지 디버깅 함 ㅠㅠ
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

        P = Eigen::MatrixXd::Identity(3, 3) * 100.0;
        Q << 0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01;
        R << 100;
        X << 0, 90, 1100;

        Ad = Eigen::MatrixXd::Zero(3, 3);
       

        Kappa = _Kappa;

        K = Eigen::MatrixXd::Zero(num_state,1);

        
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

        std::cout << "Constructor\n";

    }

    void sigmaPoints_weightSelect(){
        Xi = Eigen::MatrixXd::Zero(num_state, 2*num_state+1);
        W = Eigen::MatrixXd::Zero(2*num_state+1, 1);

        Xi.col(0) = X;
        W(0) = Kappa / (num_state + Kappa);

        Eigen::LLT<Eigen::MatrixXd> _U( (num_state + Kappa)*P );
        Eigen::MatrixXd U_ = _U.matrixU();

        //right
        for(int j =0 ; j < num_state; ++j){
            Xi.col(j+1) = X + U_.row(j).transpose();
            W(j+1) = 1/(2*(num_state + Kappa));
        }

        //left
        for(int j =0 ; j <num_state; ++j){
            Xi.col(num_state+j+1) = X - U_.row(j).transpose();
            W(num_state+j+1) = 1/(2*(num_state + Kappa));
        }

        std::cout << "[1] SigmanPoints and WeightSelect\n";
    }

     std::vector<Eigen::MatrixXd> unscentedTransform(Eigen::MatrixXd outX, Eigen::MatrixXd noiseCov){
        int n = outX.rows();
        int kmax = outX.cols();

        Eigen::MatrixXd Xm = Eigen::MatrixXd::Zero(n, 1);
        for (int k = 0 ; k < kmax; ++k){
            Xm = Xm  + W(k) * outX.col(k);
        }

        Eigen::MatrixXd Xcov = Eigen::MatrixXd::Zero(n, n);
        for (int k =0 ; k < kmax ; ++k){
            Xcov = Xcov + W(k) * (outX.col(k) - Xm) * (outX.col(k) - Xm).transpose();
        }

        Xcov = Xcov + noiseCov;
        return {Xm, Xcov};
    }


    Eigen::MatrixXd processF(Eigen::MatrixXd _X){
        Eigen::MatrixXd Xp = Eigen::MatrixXd::Zero(3,1);
        Ad << 1, dt, 0,
              0,  1, 0,
              0,  0, 1;
        Xp = Ad*_X;

        return Xp;
    }

    void predict()
    {
        fXi = Eigen::MatrixXd::Zero(num_state, 2*num_state+1);

        for(int k = 0 ; k < (2*num_state+1); ++k){
            fXi.col(k) = processF(Xi.col(k));
        }

        vector<Eigen::MatrixXd> ret = unscentedTransform(fXi, Q);

        X = ret[0];
        P = ret[1];

        std::cout << "[2] Predict\n";
    }
    
    Eigen::MatrixXd nonlinearH(Eigen::MatrixXd _X)
    {
        Eigen::MatrixXd ret = Eigen::MatrixXd::Zero(1,1);
        ret << sqrt(_X(0)*_X(0) + _X(2) * _X(2));

        return ret;
    }

    void kalmanGainCalculate(){
        hXi = Eigen::MatrixXd::Zero(1, 2*num_state+1);

        for (int k =0 ; k < (2*num_state+1); ++k){
            hXi.col(k) = nonlinearH(fXi.col(k));
        }

        vector<Eigen::MatrixXd> ret = unscentedTransform(hXi, R);
        Zp = ret[0];
        Pz = ret[1];

        Pxz = Eigen::MatrixXd::Zero(num_state, 1); // Pxz 0으로 초기화 안해줘서 무슨 문제가 하고 수식 다 들여봄... 새벽 2시까지 디버깅 함 ㅠㅠ
        for(int k= 0; k < (2*num_state+1) ; ++k){
            Pxz = Pxz + W(k) * (fXi.col(k) - X) * (hXi.col(k) - Zp).transpose();
        }
        K = Pxz*Pz.inverse();

        std::cout << "[3] KalmanGain\n";
    }


    void update(double z1)
    {
        Eigen::MatrixXd Z  = Eigen::MatrixXd::Zero(1,1);
        Z << z1;

        X = X + K * (Z - Zp);
        P = P - K * Pz * K.transpose();

        std::cout << "[4] Update(Estimated): \n";
    }


    void monitoring(vector<double> ref)
    {
        cout << "Monitoring(" << setw(3) << now << "s): " << endl;
        cout << "Covariance: " << endl
             << P << endl;
        cout << "KalmanGain: " << endl
             << K << endl;
        cout << "State: " << X(0) << ", " << X(1) << ", " << X(2)  << ", " <<sqrt(X(0)*X(0) + X(2)*X(2))<< endl;
        cout <<"REF: " << ref[0] << ", " << ref[1] << ", " << ref[2] << ", " << sqrt(ref[0]*ref[0] + ref[2]*ref[2])  << ", " << ref[3] << endl;
        cout << "Error: " << abs(ref[0] - X(0)) << ", " << abs(ref[1] - X(1)) << ", " << abs(ref[2] - X(2)) << endl;


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

    double dt = SAMPLING_MS / 1000.0;
    double Kappa;

    Eigen::MatrixXd P, Q, R;
    Eigen::MatrixXd K;
    Eigen::MatrixXd X, Zp;
    Eigen::MatrixXd Pz, Pxz;
    Eigen::MatrixXd Ad;

    Eigen::MatrixXd Xi, W;
    Eigen::MatrixXd fXi, hXi;

    int num_state = 3;
    
    double now = 0;
    bool makeFileFlag;  // for export file
    ofstream writeFile; // for export file
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

    UnscentedKalman filter(3, makeFileFlag);

    for (int col = 0; col < Sensor[0].size(); ++col)
    {
        filter.sigmaPoints_weightSelect();
        filter.predict();
        filter.kalmanGainCalculate();
        this_thread::sleep_for(chrono::milliseconds(SAMPLING_MS));
        filter.update(Sensor[0][col]);

        vector<double> RefCol = {Ref[0][col], Ref[1][col], Ref[2][col], Sensor[0][col]};
        filter.monitoring(RefCol);
    }

    return 0;
}
