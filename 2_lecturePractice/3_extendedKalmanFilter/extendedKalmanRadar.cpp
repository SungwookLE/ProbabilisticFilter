/**
 * @file extendedKalmanRadar.cpp
 * @brief Example3,
 * @brief NonLinear Case, X = [종거리, 종속도, 횡거리]
 * @brief Z = 유클리드 거리
 * @brief For analysis, file in & out is available
 * @name Sungwook Lee (joker1251@naver.com)
 * @date 2023-01-18
 */

#include <iostream>
#include "Eigen/Dense" // g++ $(pkg-config --cflags eigen3) extendedKalmanRadar.cpp
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

class ExtendedKalman
{
public:
    ExtendedKalman(bool _makeFileFlag = false)
    {
        P = Eigen::MatrixXd::Zero(3, 3);
        Q = Eigen::MatrixXd::Zero(3, 3);
        R = Eigen::MatrixXd::Zero(1, 1);
        X = Eigen::MatrixXd::Zero(3, 1);

        Ad = Eigen::MatrixXd::Zero(3, 3);
        Bd = Eigen::MatrixXd::Zero(3, 1);
        Hd = Eigen::MatrixXd::Zero(1, 3);

        P = Eigen::MatrixXd::Identity(3, 3) * 10;
        Q << 0, 0, 0, 0, 0.001, 0, 0, 0, 0.001;
        R << 10;
        X << 0, 90, 1100;

        Ad << 1, dt, 0, 0, 1, 0, 0, 0, 1;

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

    void update(double z1)
    {
        Eigen::MatrixXd Z(1, 1);
        Z << z1;

        Hd = HJacob(X);

        K = P * Hd.transpose() * (Hd * P * Hd.transpose() + R).inverse();
        X = X + K * (Z - NonlinearH(X));
        P = P - K * Hd * P;
    }

    Eigen::MatrixXd HJacob(Eigen::MatrixXd _X)
    {
        Eigen::MatrixXd ret(1, 3);
        ret << _X(0) / sqrt((_X(0) * _X(0) + _X(2) * _X(2))), 0, _X(2) / sqrt((_X(0) * _X(0) + _X(2) * _X(2)));
        return ret;
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

    ~ExtendedKalman()
    {
        if (makeFileFlag)
            writeFile.close(); // for export file
    }

private:
    Eigen::MatrixXd P, Q, R;
    Eigen::MatrixXd Ad, Bd, Hd;
    Eigen::MatrixXd X;
    Eigen::MatrixXd K;

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

    ExtendedKalman filter(makeFileFlag);

    for (int col = 0; col < Sensor[0].size(); ++col)
    {
        filter.predict();
        this_thread::sleep_for(chrono::milliseconds(SAMPLING_MS));
        filter.update(Sensor[0][col]);
        vector<double> RefCol = {Ref[0][col], Ref[1][col], Ref[2][col]};
        filter.monitoring(RefCol);
    }
    return 0;
}
