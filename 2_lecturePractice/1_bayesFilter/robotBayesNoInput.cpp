/**
 * @file robotBayes.cpp
 * @brief Door Open using Bayes Filter
 * @name Sungwook Lee (joker1251@naver.com)
 * @date 2023-01-16
 */

#include <iostream>
// #include "Eigen/Dense" // g++ $(pkg-config --cflags eigen3) kalman_filter_2D_linear.cpp
#include <thread>
#include <chrono>
#include <iomanip>

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
            cout << "p(Z|open), p(Z|close): ";
            cin >> pZBarOpen >> pZBarClose;

            double pOpenPrev = pOpen, pClosePrev = pClose;

            pOpen = pZBarOpen * pOpenPrev / (pZBarOpen * pOpenPrev + pZBarClose * pClosePrev);
            pClose = pZBarClose * pClosePrev / (pZBarClose * pClosePrev + pZBarOpen * pOpenPrev);

            cout << setw(3) << tick << ": p(open|z) = " << pOpen << ", p(close|z) = " << pClose << endl;
            this_thread::sleep_for(chrono::milliseconds(samplingMs));
        }
    }
private:
    double pZBarOpen, pZBarClose;                      // pZBarOpen: 문이 열리면 센서 플래그가 뜰 확률, pZBarClose: 문히 닫혀있으면 센서 플래그가 뜰 확률
    double pOpen = P_OPEN_INIT, pClose = P_CLOSE_INIT; // pOpen = pOpenBarZ: 현재 센서값을 기준으로 문이 열려있을 확률, pClose = pCloseBarZ: 현재 센서값을 기준으로 문이 닫혀있을 확률
    int samplingMs = 1000;                             // for sleep
    int tick = 0;                                      // for monitoring
};

int main()
{

    BayesFilter filter;
    filter.run();

    return 0;
}