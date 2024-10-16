import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class KalmanFilter:
    def __init__(self, y_Measure_init, step_time = 0.1, m = 0.1, modelVariance = 0.01, measureVariance = 1.0, errorVariance_init = 10.0):
        self.A = (1.0) #A는 우선 그대로 반영함으로서 놔두기
        self.B = step_time/m 
        self.C = 1.0 # C는 y에 x의 추정치를 얼마나 반영 할 것인지 결정 하는 것
        self.D = 0.0 #D는 보통 0으로 둠
        self.Q = modelVariance #모델을 얼마나 믿을 것인지, 이 값이 높다면 모델을 덜 믿는 것
        self.R = measureVariance #센서를 얼마나 믿을 것인가, 이 값이 높다면 센서를 덜 믿는 것
        self.x_estimate = y_Measure_init
        self.x_prediction = self.x_estimate
        self.y_prediction = self.x_estimate
        self.P_estimate = errorVariance_init
        self.K = self.P_estimate * self.C / (self.C * self.P_estimate * self.C + self.R) # K는 이와 같은 과정을 통해 계산 될 것

    def estimate(self, y_measure, input_u):
        # Prediction (예측 단계)
        self.x_prediction = self.A * self.x_estimate + self.B * input_u  # 상태 예측
        self.P_prediction = self.A * self.P_estimate * self.A + self.Q  # 오차 공분산 예측
        self.y_prediction = self.C * self.x_prediction

        # Kalman Gain (칼만 이득)
        self.K = self.P_prediction * self.C / (self.C * self.P_prediction * self.C + self.R)

        # Update (업데이트 단계)
        self.x_estimate = self.x_prediction + self.K * (y_measure - self.y_prediction)  # 상태 업데이트
        self.P_estimate = (1 - self.K * self.C) * self.P_prediction  # 오차 공분산 업데이트



if __name__ == "__main__":
    signal = pd.read_csv("01_filter/Data/example06.csv")

    y_estimate = KalmanFilter(signal.y_measure[0], modelVariance=0.3, measureVariance=0.97)
    for i, row in signal.iterrows():
        y_estimate.estimate(signal.y_measure[i],signal.u[i])
        signal.y_estimate[i] = y_estimate.x_estimate

    plt.figure()
    plt.plot(signal.time, signal.y_measure,'k.',label = "Measure")
    plt.plot(signal.time, signal.y_estimate,'r-',label = "Estimate")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()



