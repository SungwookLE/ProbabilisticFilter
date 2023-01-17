/**
 * @file robotBayesWithInput.cpp
 * @brief Door Open using Bayes Filter With Input
 * @date 2023-01-16
 */

#include <iostream>
#include "Eigen/Dense" // g++ $(pkg-config --cflags eigen3) robotBayesWithInput.cpp
#include <thread>
#include <chrono>
#include <iomanip>
#include <vector>

#define P_OPEN_INIT 0.5
#define P_CLOSE_INIT 0.5

using namespace std;

class BayesFilter
{
public:
    void run()
    {
        while (true)
        {
            tick += 1;
            double belOpenPrev = belOpen, belClosePrev = belClose;

            cout << "Robot Input Pushing(1) or Not(0): ";
            cin >> isPushing;
            cout << "Robot Measurement Open(1) or Closed(0): ";
            cin >> isOpen;

            if (isPushing)
            {
                belBarOpen = pXOpenBarUPushXOpen * belOpenPrev + pXOpenBarUPushXClose * belClosePrev;
                belBarClose = pXCloseBarUPushXClose * belClosePrev + pXCloseBarUPushXOpen * belOpenPrev;
            }
            else
            {
                belBarOpen = pXOpenBarUNotXOpen * belOpenPrev + pXOpenBarUNotXClose * belClosePrev;
                belBarClose = pXCloseBarUNotXClose * belClosePrev + pXCloseBarUNotXOpen * belOpenPrev;
            }
            cout << setw(3) << tick << ": belBarOpen = " << setw(5) << belBarOpen << ", belBarClose = " << setw(5) << belBarClose << endl;

            if (isOpen)
            {
                double normalizer = (pZOpenBarXOpen * belBarOpen + pZOpenBarXClose * belBarClose);
                belOpen = pZOpenBarXOpen * belBarOpen / normalizer;
                belClose = pZOpenBarXClose * belBarClose / normalizer;
            }
            else
            {
                double normalizer = (pZCloseBarXClose * belBarClose + pZCloseBarXOpen * belBarOpen);
                belOpen = pZCloseBarXOpen * belBarOpen / normalizer;
                belClose = pZCloseBarXClose * belBarClose / normalizer;
            }
            cout << setw(3) << tick << ": belOpen = " << setw(5) << belOpen << ", belClose = " << setw(5) << belClose << endl;
        }
    }

    void run_step(vector<pair<bool, bool>> inputsAndMeasures)
    {

        for (auto step : inputsAndMeasures)
        {
            tick += 1;
            double belOpenPrev = belOpen, belClosePrev = belClose;

            cout << "Robot Input Pushing(1) or Not(0): " << step.first << endl;
            isPushing = step.first;

            cout << "Robot Measurement Open(1) or Closed(0): " << step.second << endl;
            isOpen = step.second;

            if (isPushing)
            {
                belBarOpen = pXOpenBarUPushXOpen * belOpenPrev + pXOpenBarUPushXClose * belClosePrev;
                belBarClose = pXCloseBarUPushXClose * belClosePrev + pXCloseBarUPushXOpen * belOpenPrev;
            }
            else
            {
                belBarOpen = pXOpenBarUNotXOpen * belOpenPrev + pXOpenBarUNotXClose * belClosePrev;
                belBarClose = pXCloseBarUNotXClose * belClosePrev + pXCloseBarUNotXOpen * belOpenPrev;
            }
            cout << setw(3) << tick << ": belBarOpen = " << setw(5) << belBarOpen << ", belBarClose = " << setw(5) << belBarClose << endl;

            if (isOpen)
            {
                double normalizer = (pZOpenBarXOpen * belBarOpen + pZOpenBarXClose * belBarClose);
                belOpen = pZOpenBarXOpen * belBarOpen / normalizer;
                belClose = pZOpenBarXClose * belBarClose / normalizer;
            }
            else
            {
                double normalizer = (pZCloseBarXClose * belBarClose + pZCloseBarXOpen * belBarOpen);
                belOpen = pZCloseBarXOpen * belBarOpen / normalizer;
                belClose = pZCloseBarXClose * belBarClose / normalizer;
            }
            cout << setw(3) << tick << ": belOpen = " << setw(5) << belOpen << ", belClose = " << setw(5) << belClose << endl;
            this_thread::sleep_for(chrono::milliseconds(samplingMs));
        }
    }

    void run_step_eigen(vector<pair<bool, bool>> inputsAndMeasures) // g++ $(pkg-config --cflags eigen3) robotBayesWithInput.cpp
    {
        // for matrix Eigen library
        vectorZOpen << pZOpenBarXOpen, pZOpenBarXClose;
        vectorZClose << pZCloseBarXOpen, pZCloseBarXClose;
        matrixUPush << pXOpenBarUPushXOpen, pXOpenBarUPushXClose,
            pXCloseBarUPushXOpen, pXCloseBarUPushXClose;
        matrixUNot << pXOpenBarUNotXOpen, pXOpenBarUNotXClose,
            pXCloseBarUNotXOpen, pXCloseBarUNotXClose;
        vectorBelBar << belBarOpen, belBarClose;
        vectorBel << belOpen, belClose;

        for (auto step : inputsAndMeasures)
        {
            tick += 1;
            Eigen::Vector2d vectorBelPrev = vectorBel;

            cout << "Robot Input Pushing(1) or Not(0): " << step.first << endl;
            isPushing = step.first;

            cout << "Robot Measurement Open(1) or Closed(0): " << step.second << endl;
            isOpen = step.second;

            if (isPushing)
            {
                vectorBelBar = matrixUPush * vectorBelPrev;
            }
            else
            {
                vectorBelBar = matrixUNot * vectorBelPrev;
            }
            cout << setw(3) << tick << ": belBarOpen = " << setw(5) << vectorBelBar(0) << ", belBarClose = " << setw(5) << vectorBelBar(1) << endl;

            if (isOpen)
            {
                double normalizer = vectorZOpen.transpose() * vectorBelBar;
                vectorBel << vectorZOpen(0) * vectorBelBar(0) / normalizer, vectorZOpen(1) * vectorBelBar(1) / normalizer;
            }
            else
            {
                double normalizer = vectorZClose.transpose() * vectorBelBar;
                vectorBel << vectorZClose(0) * vectorBelBar(0) / normalizer, vectorZClose(1) * vectorBelBar(1) / normalizer;
            }
            cout << setw(3) << tick << ": belOpen = " << setw(5) << vectorBel(0) << ", belClose = " << setw(5) << vectorBel(1) << endl;
            this_thread::sleep_for(chrono::milliseconds(samplingMs));
        }
    }

private:
    double pZOpenBarXOpen = 0.6, pZCloseBarXOpen = 0.4, pZOpenBarXClose = 0.2, pZCloseBarXClose = 0.8;                 // 센서 noise
    double pXOpenBarUPushXOpen = 1, pXCloseBarUPushXOpen = 0, pXOpenBarUPushXClose = 0.8, pXCloseBarUPushXClose = 0.2; // input noise when doing push
    double pXOpenBarUNotXOpen = 1, pXCloseBarUNotXOpen = 0, pXOpenBarUNotXClose = 0, pXCloseBarUNotXClose = 1;         // input noise when doing nothing

    bool isPushing = false, isOpen = false; // isPushing is Robot's input, isOpen is Robot's measurement
    double belBarOpen, belBarClose;
    double belOpen = P_OPEN_INIT, belClose = P_CLOSE_INIT; // belOpen: 현재 센서값을 기준으로 문이 열려있을 확률, belClose: 현재 센서값을 기준으로 문이 닫혀있을 확률

    int samplingMs = 1000; // for sleep
    int tick = 0;          // for monitoring

    Eigen::Vector2d vectorZOpen = Eigen::Vector2d::Zero(2, 1);  // for matrix Eigen library
    Eigen::Vector2d vectorZClose = Eigen::Vector2d::Zero(2, 1); // for matrix Eigen library
    Eigen::Matrix2d matrixUPush = Eigen::Matrix2d::Zero(2, 2);  // for matrix Eigen library
    Eigen::Matrix2d matrixUNot = Eigen::Matrix2d::Zero(2, 2);   // for matrix Eigen library
    Eigen::Vector2d vectorBelBar = Eigen::Vector2d::Zero(2, 1); // for matrix Eigen library
    Eigen::Vector2d vectorBel = Eigen::Vector2d::Zero(2, 1);    // for matrix Eigen library
};

int main()
{
    vector<pair<bool, bool>> inputsAndMeasures = {{0, 0}, {1, 1}, {0, 1}, {1,0}}; // user input

    BayesFilter filter1;
    filter1.run_step(inputsAndMeasures);

    BayesFilter filter2;
    filter2.run_step_eigen(inputsAndMeasures); // g++ $(pkg-config --cflags eigen3) robotBayesWithInput.cpp

    BayesFilter filter3;
    filter3.run(); 

    return 0;
}