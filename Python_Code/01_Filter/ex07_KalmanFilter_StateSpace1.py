import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class KalmanFilter:
    '''
    칼만 필터로 행렬 받아서 계산하기
    '''
    def __init__(self, y_Measure_init, step_time=0.1, m=1.0, Q_x=0.02, Q_v=0.05, R=5.0, errorCovariance_init=10.0):
        self.A = np.array([[1.0, step_time], [0.0, 1.0]])
        self.B = np.array([[0.0], [step_time / m]])
        self.C = np.array([[1.0, 0.0]])
        self.D = 0.0
        self.Q = np.array([[Q_x, 0.0], [0.0, Q_v]])
        self.R = R
        self.x_estimate = np.array([[y_Measure_init],[0.0]])
        self.x_prediction = self.x_estimate   
        self.P_estimate = np.array([[errorCovariance_init, 0.0],[0.0, errorCovariance_init]])
        self.K = (self.P_estimate * self.C.T) / (self.C * self.P_estimate * self.C.T + self.R) 

    def estimate(self, y_measure, input_u):
        # Prediction (예측 단계)
        self.x_prediction = self.A * self.x_estimate + self.B * input_u  # 상태 예측
        self.P_prediction = self.A * self.P_estimate * self.A.T + self.Q  # 오차 공분산 예측
        self.y_prediction = self.C * self.x_prediction

        # Kalman Gain (칼만 이득)
        self.K = (self.P_estimate * self.C.T) / (self.C * self.P_estimate * self.C.T + self.R) 

        # Update (업데이트 단계)
        self.x_estimate = self.x_prediction + self.K * (y_measure - self.y_prediction)  # 상태 업데이트
        self.P_estimate = (np.ones(2) - self.K * self.C) * self.P_prediction  # 오차 공분산 업데이트



if __name__ == "__main__":
    signal = pd.read_csv("01_filter/Data/example07.csv")

    y_estimate = KalmanFilter(signal.y_measure[0], Q_x = 0.1, Q_v = 0.1)
    for i, row in signal.iterrows():
        y_estimate.estimate(signal.y_measure[i],signal.u[i])
        signal.y_estimate[i] = y_estimate.x_estimate[0][0]

    plt.figure()
    plt.plot(signal.time, signal.y_measure,'k.',label = "Measure")
    plt.plot(signal.time, signal.y_estimate,'r-',label = "Estimate")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()



