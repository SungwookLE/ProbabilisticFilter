# Probabilistic Filter
- Sungwook LEE(joker1251@naver.com)
    - Full Authority in lecturePractice code to `Sungwook LEE`
- Date: 23.1.16. ~ 23.1.19.

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
    - [pdf link](https://docs.ufpr.br/~danielsantos/ProbabilisticRobotics.pdf)

## Day1, Bayes

1. In the beginning ...
    - Total Probability Theorem 
        - ![](img/2023-01-17-09-15-46.png)
    - Conditional Probability (Likelihood)
        - ![](img/2023-01-17-09-16-35.png)
    - Bayes Rule
        - ![](img/2023-01-17-09-16-09.png)

2. Bayes Filter (수식 매우 중요)
    - Bayes Rule에 `Marcov Process` + `Recursive` 을 부여하여 `Belief`를 Recursively Update!
    - ![](img/2023-01-16-15-12-21.png)
    - ![](img/2023-01-16-15-13-58.png)

3. Lecture Practice
    - [robotBayesNoInput.cpp](./2_lecturePractice/1_bayesFilter/robotBayesNoInput.cpp)
    - [robotBayesWithInput.cpp](./2_lecturePractice/1_bayesFilter/robotBayesWithInput.cpp)

4. Naive Bayes Filter Review
    - ![](img/2023-01-17-09-21-42.png)
    - ILLUSTRAION OF MARKOV
        - ![](img/2023-01-19-11-54-56.png)
    - 베이즈 필터, 칼만 필터, 파티클 필터의 입력에 의한 predict, 관측에 의한 update 단계에서의 `bel=covariance`는 정확히 위의 그림과 같이 움직인다.
    - 예측 단계에서는 covariance가 움직이고(`+더하기` 불확실성 증대), update 단계에서는 covariance가 보정된다.(`x곱하기, 확률은 1보다 작은값이어서 곱하면 작아진다.` 불확실성 감쇄)

## Day2, Kalman
- Bayes 필터는, 모든 state에 대한 확률 적분값(Total Probability Theorem)을 구하는 것이 현실적으로 불가능하다.
    - 칼만필터 등장!
- 이를 실현 가능한 표현형으로 풀어낸 것이 칼만필터이다. 바로 가우시안 확률분포를 활용하여서..,
- 모델의 분산과, 센서값의 분산, 이 둘을 이용해서 new_X를 만들어 내는 과정이 칼만필터
    - ![](img/2023-01-17-19-43-42.png)

1. Derivation
- Assumption: Linear Model, Noise is follow Gaussian
    - Linear여야지만, 칼만 필터의 Equation을 유도할 수 있고 (예를 들어, `A'PA+Q`) 가우시안 이여야지만 확률분포를 임의의 함수가 아닌, 평균과 분산으로만 기술할 수 있게 된다.
- ![](img/2023-01-17-19-43-01.png)
- ![](img/2023-01-17-11-03-16.png)
- ![](img/2023-01-17-11-05-00.png)
- 자료에 오타가 있는데, 한번 정리한 후에 올려야할 듯

- ILLUSTRAION OF KALMAN
    - ![](img/2023-01-19-11-55-58.png)

2. Lecture Practice
    - [kalmanLinearX.cpp](./2_lecturePractice/2_kalmanFilter/kalmanLinearX.cpp)
    - [kalmanLinearXY.cpp](./2_lecturePractice/2_kalmanFilter/kalmanLinearXY.cpp)

3. Extended Kalman Filter
    - 선형화(Jacobian)을 통한 [Covariance, State] Linear Expansion
    - ![](img/2023-01-18-09-21-24.png)
    - ![](img/2023-01-18-09-21-41.png)
    - ![](img/2023-01-18-09-21-49.png)


## Day3, Nonlinear Kalman

1. Lecture Practice(EFK)
    - [extendedKalmanRadar.cpp](./2_lecturePractice/3_extendedKalmanFilter/extendedKalmanRadar.cpp)
    - [extendedKalmanBenchmark.cpp](./2_lecturePractice/3_extendedKalmanFilter/extendedKalmanBenchmark.cpp)

2. Unscented Kalman Filter
    - 비선형성이 크면 클수록, EKF의 성능은 저하된다.
    - `Sigma Points`인 (1+2N)개 포인트 만으로도 로 가우시안을 평가할 수 있다. `Unscented Transform`
        - N은 State의 개수를 의미함
        - ![](img/2023-01-18-13-29-51.png)
    - 점들은 어떻게 고를까?
        - Unscented Transform 에 정의대로 1+2N개를 고른다.
        - ![](img/2023-01-18-13-50-54.png)
    - ![](img/2023-01-18-14-22-38.png)
    - ![](img/2023-01-18-14-22-47.png)

3. Lecture Practice(UKF)
    - [unscentedKalmanRadar.cpp](./2_lecturePractice/4_unscentedKalmanFilter/unscentedKalmanRadar.cpp)

3. 비선형 칼만 한계
    - 가우시안 노이즈를 따르지 않는 문제에서, 어떻게든 선형화(Extended), 선형평가(Unscented) 방식으로는 필터의 성능은 나오지 않는다.
    - 비선형을 무조건 선형화(Relaxation)할 수 있는 것이 아니다.
    - 베이즈 필터는 적분에 한계가 있었기 때문에.., 가우시안 분포를 가져와 칼만필터를 만든 것
        - 칼만필터: `Parameteric Approach`: N(0, Sigma^2)
    - 다른식의 접근법: `Non Parameteric Approach`: 가우시안 함수 등을 사용하지 말고 적분해야 하는 샘플의 개수를 유한개(몬테카를로 Sampling)로 하여 적분값을 평가하자
        - 파티클필터 등장!!

4. Particle Filter
    - ![](img/2023-01-18-16-50-57.png)
    - ![](img/2023-01-18-16-51-07.png)

## Day4, Particle Filter
1. 확률필터 리뷰
    - 알고싶은 정보: $P(x_t|u_{1:t}, z_{1:t})$
    - 어떻게 하면 알 수 있을까?, 마르코프 프로세스와 재귀적 구조를 사용하여 구하자.
        - 재귀적 구조로 belief update
        - ![](img/2023-01-19-09-30-35.png)

2. 칼만필터 접근법 한계 리뷰
    - 리니어야하고, 노이즈는 가우시안 분포를 따라야 한다.
    - 근데, 가우시안 분포를 따르지 않는다면?, 더 이상 $N(\mu,\sigma^2)$ 의 파라미터인 $\mu$, $\Sigma$로 식을 derivation 할 수 없다.
    - 칼만필터의 접근법을 `Parameteric Approach` 라고 한다.
    - 가우시안 함수를 사용치 말고 샘플의 개수를 유한개로(몬테카를로 샘플링)하여 적분값을 평가하자. (`Non Parameteric Approach`)

3. 파티클필터
    - 컨셉
        - 분산이나, 기대값을 어떤 모형 함수(예를들어, 가우시안)로 구하지 않고, 직접 구하자
        - 모든 정보를 가지고 분산을 평가할 것이 아니라, 아는 정보만을 가지고 분산을 평가하자

    - Predict(control): Sampling
    - Update(measurement): 관측값과 확률매칭하여 웨이트 갱신
        - ![](img/2023-01-19-10-13-35.png)
    
    - 두가지 종류
        1. SIS(Sequential Importance Sampling): 파티클 하나가 웨이트 하나를 가짐으로써, 웨이트가 높은 파티클이 잘못 수렴되었을때, 추정 결과값이 이상해지는것을 막을 수 없다. (웨이트의 크기에 따라 입자 선택)
        2. SIR(sequential Importance resampling): Resampling Wheel 과정을 통해 이전 스텝에서의 웨이트를 기준으로 파티클을 할당하고 초기화함으로써(할당된 파티클의 개수가 웨이트의 크기라고 볼 수 있음) 파티클을 새롭게 분포시킴으로써, 잘못된 결과값에 수렴되지않게끔 -> 일반적으로 우리가 부르는 파티클 필터 (참고: [particleLocalization.cpp](./0_filterPreview/particle_filter.cpp))

