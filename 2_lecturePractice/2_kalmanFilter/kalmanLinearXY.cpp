/**
 * @file kalmanLinearXY.cpp
 * @brief Example2,
 * @brief Linear Case, CONSTANT ACCELERATION MODEL with 2DOF, X = [position ,velocity, acceleration], z = position, all states are observable.
 * @date 2023-01-17
 */

#include <iostream>
#include "Eigen/Dense" // g++ $(pkg-config --cflags eigen3) kalmanLinearXY.cpp
#include <thread>
#include <chrono>
#include <iomanip>
#include <vector>
#include <fstream> // for csv file read
#include <string>  // for csv file read

using namespace std;
#define SAMPLING_MS 100

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

class Kalman
{
public:
    Kalman(bool _makeFileFlag = false)
    {
        P = Eigen::MatrixXd::Zero(6, 6);
        Q = Eigen::MatrixXd::Zero(6, 6);
        R = Eigen::MatrixXd::Zero(2, 2);
        X = Eigen::MatrixXd::Zero(6, 1);

        Ad = Eigen::MatrixXd::Zero(6, 6);
        Bd = Eigen::MatrixXd::Zero(6, 2);
        Cd = Eigen::MatrixXd::Zero(2, 6);

        Eigen::MatrixXd Px = Eigen::MatrixXd::Identity(3, 3) * 100;
        Eigen::MatrixXd Qx = Eigen::MatrixXd::Identity(3, 3);
        Eigen::MatrixXd Rx = Eigen::MatrixXd::Identity(1, 1) * 0.01;
        Eigen::MatrixXd Xx = Eigen::MatrixXd::Random(3, 1);

        Eigen::MatrixXd Adx = Eigen::MatrixXd::Zero(3, 3);
        Eigen::MatrixXd Bdx = Eigen::MatrixXd::Zero(3, 1);
        Eigen::MatrixXd Cdx = Eigen::MatrixXd::Zero(1, 3);

        Eigen::MatrixXd Pad6 = Eigen::MatrixXd::Zero(3, 3);
        Eigen::MatrixXd Pad3 = Eigen::MatrixXd::Zero(3, 1);

        Adx << 1, dt, dt * dt / 2,
            0, 1, dt,
            0, 0, 1;
        Bdx << dt * dt / 2, dt, 1;
        Cdx << 1, 0, 0;

        P << Px, Pad6,
            Pad6, Px;
        Q << Qx, Pad6,
            Pad6, Qx;
        R << Rx, 0, 0, Rx;
        X << Xx, Xx;

        Ad << Adx, Pad6, Pad6, Adx;
        Bd << Bdx, Pad3, Pad3, Bdx;
        Cd << Cdx, Pad3.transpose(), Pad3.transpose(), Cdx;

        makeFileFlag = _makeFileFlag;
        if (makeFileFlag) // for export file
        {
            writeFile.open("output.csv", ios::out);
            writeFile << "COV_X,"
                      << "COV_dotX,"
                      << "COV_ddotX,"
                      << "COV_Y,"
                      << "COV_dotY,"
                      << "COV_ddotY,"
                      << "X,"
                      << "dotX,"
                      << "ddotX,"
                      << "Y,"
                      << "dotY,"
                      << "ddotY,"
                      << "Error X,"
                      << "Error dotX,"
                      << "Error ddotX,"
                      << "Error Y,"
                      << "Error dotY,"
                      << "Error ddotY" << endl;
        }
    }

    void predict(double u1, double u2)
    {
        Eigen::MatrixXd U(2, 1);
        U << u1, u2;
        X = Ad * X + Bd * U; // 6x1 + 6x2 x 2x1
        P = Ad * P * Ad.transpose() + Q;
    }

    void update(double z1, double z2)
    {
        Eigen::MatrixXd Z(2, 1);
        Z << z1, z2;

        K = P * Cd.transpose() * (Cd * P * Cd.transpose() + R).inverse();
        X = X + K * (Z - Cd * X);
        P = P - K * Cd * P;
    }

    void monitoring(vector<double> ref)
    {
        cout << "Monitoring(" << setw(3) << now << "s): " << endl;
        cout << "Covariance: " << endl
             << P << endl;
        cout << "State: " << endl
             << X << endl;
        cout << "Error: " << abs(ref[0] - X(0)) << ", " << abs(ref[1] - X(1)) << ", " << abs(ref[2] - X(2))
             << ", " << abs(ref[3] - X(3)) << ", " << abs(ref[4] - X(4)) << ", " << abs(ref[5] - X(5)) << endl;

        writeFile << P(0, 0) << ", " << P(1, 1) << ", " << P(2, 2) << ", " << P(3, 3) << ", " << P(4, 4) << ", " << P(5, 5) << ", " << X(0) << ", " << X(1) << ", " << X(2) << ", " << X(3) << ", " << X(4) << ", " << X(5)
                  << ", " << abs(ref[0] - X(0)) << ", " << abs(ref[1] - X(1)) << ", " << abs(ref[2] - X(2)) << ", " << abs(ref[3] - X(3)) << ", " << abs(ref[4] - X(4)) << ", " << abs(ref[5] - X(5)) << endl;
        now+= dt;
    }

    ~Kalman()
    {
        if (makeFileFlag)
            writeFile.close(); // for export file
    }

private:
    Eigen::MatrixXd P, Q, R;
    Eigen::MatrixXd Ad, Bd, Cd;
    Eigen::MatrixXd X;
    Eigen::MatrixXd K;

    double dt = SAMPLING_MS/1000.0;
    double now = 0;
    ofstream writeFile; // for export file
    bool makeFileFlag;  // for export file
};

int main()
{
    FileReader reader("inputRef_XY.csv");
    vector<vector<double>> Ref = reader.parsed;

    FileReader reader2("inputSensors_XY.csv");
    vector<vector<double>> Sensors = reader2.parsed;

    bool makeFileFlag = false;
    cout << "Export the result as CSV file: Yes(1), No(0): ";
    cin >> makeFileFlag;

    Kalman filter(makeFileFlag);

    for (int col = 0; col < Sensors[0].size(); ++col)
    {
        filter.predict(0, 0);
        this_thread::sleep_for(chrono::milliseconds(SAMPLING_MS));
        filter.update(Sensors[0][col], Sensors[1][col]);
        vector<double> RefCol = {Ref[0][col], Ref[1][col], Ref[2][col], Ref[3][col], Ref[4][col], Ref[5][col]};
        filter.monitoring(RefCol);
    }
    return 0;
}
