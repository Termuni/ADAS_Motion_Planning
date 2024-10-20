import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class MovingAverageFilter:
    def __init__(self, y_initial_measure, num_average=2.0):
        self.num_average = num_average
        self.y_measurements = [y_initial_measure]
        self.y_estimate = y_initial_measure
        
    def estimate(self, y_measure):
        # 새로운 측정값을 추가
        self.y_measurements.append(y_measure)
        
        # 이동 평균 계산, 지정된 개수만큼의 최신 값 사용
        if len(self.y_measurements) > self.num_average:
            self.y_measurements.pop(0)  # 오래된 데이터 삭제
        
        self.y_estimate = np.mean(self.y_measurements)    
        #return np.mean(self.y_measurements)  # 최신 측정값들의 평균 반환
        

    
if __name__ == "__main__":
    #signal = pd.read_csv("week_01_filter/Data/example_Filter_1.csv")      
    # signal = pd.read_csv("week_01_filter/Data/example_Filter_2.csv")
    signal = pd.read_csv("01_filter/Data/example_Filter_2.csv")

    y_estimate = MovingAverageFilter(signal.y_measure[0])
    for i, row in signal.iterrows():
        y_estimate.estimate(signal.y_measure[i])
        signal.y_estimate[i] = y_estimate.y_estimate
        
    plt.figure()
    plt.plot(signal.time, signal.y_measure,'k.',label = "Measure")
    plt.plot(signal.time, signal.y_estimate,'r-',label = "Estimate")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()



