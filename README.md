# Kalman Filter Review
- Sungwook LEE(joker1251@naver.com)
- Date: 23.1.16~19

## Prerequirements
- Install `eigen3`
    - ubuntu20.04: `sudo apt install libeigen3-dev`
    - 설치 확인: `dpkg -L libeigen3-dev`
    - header: `#include "Eigen/Dense"`
    - compile flag: `g++ $(pkg-config --cflags eigen3) file.cpp`

## Reference
- `Beyond the kalman filter particle filters for tracking applications` by M.S.Arulampalam
    - `A tutorial on particle filters for online nonlinear/non-Gaussian Bayesian tracking` by M.S.Arulampalam
- `Probabilistic Robotics` by S.Thrun

## Day1, Bayes

1. In the beginning ...
    - Total Probability Theorem 
        - ![](img/2023-01-17-09-15-46.png)
    - Conditional Probability (Likelihood)
        - ![](img/2023-01-17-09-16-35.png)
    - Bayes Rule
        - ![](img/2023-01-17-09-16-09.png)

2. Bayes Filter (수식 매우 중요)
    - ![](img/2023-01-16-15-12-21.png)
    - ![](img/2023-01-16-15-13-58.png)

3. Lecture Practice
    - [robotBayesNoInput.cpp](./2_lecturePractice/1_bayesFilter/robotBayesNoInput.cpp)
    - [robotBayesWithInput.cpp](./2_lecturePractice/1_bayesFilter/robotBayesWithInput.cpp)

4. Naive Bayes Filter Review
    - ![](img/2023-01-17-09-21-42.png)
    1. ![](img/2023-01-17-09-20-43.png)
    2. ![](img/2023-01-17-09-20-59.png)
    3. ![](img/2023-01-17-09-21-09.png)
    4. ![](img/2023-01-17-09-21-20.png)

## Day2, Kalman
- Bayes 필터는, 모든 state에 대한 확률 적분값(Total Probability Theorem)을 구하는 것이 현실적으로 불가능하다.
- 이를 실현 가능한 표현형으로 풀어낸 것이 칼만필터이다. 바로 가우시안 확률분포를 활용하여서..,
- 모델의 분산과, 센서값의 분산, 이 둘을 이용해서 new_X를 만들어 내는 과정이 칼만필터
    - ![](img/2023-01-17-19-43-42.png)

1. Derivation
- Assumption: Linear Model, Noise is follow Gaussian
- ![](img/2023-01-17-19-43-01.png)
- ![](img/2023-01-17-11-03-16.png)
- ![](img/2023-01-17-11-05-00.png)
- 자료에 오타가 있는데, 한번 정리한 후에 올려야할 듯

2. Lecture Practice
    - [kalmanLinearX](./2_lecturePractice/2_kalmanFilter/kalmanLinearX.cpp)
    - [kalmanLinearXY](./2_lecturePractice/2_kalmanFilter/kalmanLinearXY.cpp)

3. Extended Kalman Filter



