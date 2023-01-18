/**
 * @file extendedKalmanBenchmark.cpp
 * @brief Example4,
 * @brief Extreme NonLinear Case, X, Z, non linear and time varying
 * @brief For analysis, file in & out is available
 * @name Sungwook Lee (joker1251@naver.com)
 * @date 2023-01-18
 */

#include <iostream>
#include "Eigen/Dense" // g++ $(pkg-config --cflags eigen3) extendedKalmanBenchmark.cpp
#include <thread>
#include <chrono>
#include <iomanip>
#include <vector>
#include <fstream> // for csv file read
#include <string>  // for csv file read

using namespace std;

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
        P = Eigen::MatrixXd::Zero(1, 1);
        Q = Eigen::MatrixXd::Zero(1, 1);
        R = Eigen::MatrixXd::Zero(1, 1);
        X = Eigen::MatrixXd::Zero(1, 1);

        Fd = Eigen::MatrixXd::Zero(1, 1);
        Bd = Eigen::MatrixXd::Zero(1, 1);
        Hd = Eigen::MatrixXd::Zero(1, 1);

        P = Eigen::MatrixXd::Identity(1, 1) * 10;
        Q << 4;
        R << 4;
        X << 0;

        makeFileFlag = _makeFileFlag;
        if (makeFileFlag) // for export file
        {
            writeFile.open("output.csv", ios::out);
            writeFile << "COV_X,"
                      << "X,"
                      << "Error X," << endl;
        }
    }

    void predict()
    {
        iteration += 1;
        Fd = FJacob(X);

        X = NonlinearF(X);
        P = Fd * P * Fd.transpose() + Q;
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

    Eigen::MatrixXd FJacob(Eigen::MatrixXd _X)
    {
        Eigen::MatrixXd ret(1, 1);
        ret << c1 + c2 * (1.0 + _X(0) * _X(0) - _X(0) * 2.0 * _X(0)) / ((1 + _X(0) * _X(0)) * (1 + _X(0) * _X(0)));
        return ret;
    }

    Eigen::MatrixXd HJacob(Eigen::MatrixXd _X)
    {
        Eigen::MatrixXd ret(1, 1);
        ret << 0.1 * _X(0);
        return ret;
    }

    Eigen::MatrixXd NonlinearF(Eigen::MatrixXd _X)
    {
        Eigen::MatrixXd ret(1, 1);
        ret << c1 * _X(0) + c2 * _X(0) / (1 + _X(0) * _X(0)) + c3 * cos(1.2 * (iteration - 1));
        return ret;
    }

    Eigen::MatrixXd NonlinearH(Eigen::MatrixXd _X)
    {
        Eigen::MatrixXd ret(1, 1);
        ret << (_X(0) * _X(0)) / 20.0;
        return ret;
    }

    void monitoring(vector<double> ref)
    {
        cout << "Monitoring(" << setw(3) << iteration << "): " << endl;
        cout << "Covariance: " << endl
             << P << endl;
        cout << "State: " << endl
             << X << endl;
        cout << "Error: " << abs(ref[0] - X(0)) << endl;

        if (makeFileFlag)
        {
            writeFile << P(0, 0) << ", " << X(0) << ", " << abs(ref[0] - X(0)) << endl;
        }
    }

    ~ExtendedKalman()
    {
        if (makeFileFlag)
            writeFile.close(); // for export file
    }

private:
    Eigen::MatrixXd P, Q, R;
    Eigen::MatrixXd Fd, Bd, Hd;
    Eigen::MatrixXd X;
    Eigen::MatrixXd K;

    double c1 = 1.0;
    double c2 = 12.0;
    double c3 = 7.0;

    double iteration = 0;
    ofstream writeFile; // for export file
    bool makeFileFlag;  // for export file
};

int main()
{
    FileReader reader("inputRef_Benchmark.csv");
    vector<vector<double>> Ref = reader.parsed;

    FileReader reader2("inputSensor_Benchmark.csv");
    vector<vector<double>> Sensor = reader2.parsed;

    bool makeFileFlag = false;
    cout << "Export the result as CSV file: Yes(1), No(0): ";
    cin >> makeFileFlag;

    ExtendedKalman filter(makeFileFlag);

    for (int col = 0; col < Sensor[0].size(); ++col)
    {
        filter.predict();
        filter.update(Sensor[0][col]);
        vector<double> RefCol = {Ref[0][col]};
        filter.monitoring(RefCol);
    }
    return 0;
}
