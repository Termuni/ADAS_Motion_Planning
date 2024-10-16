from vehicle_model import VehicleModel
import numpy as np
import matplotlib.pyplot as plt

class P_Controller(object):
    def __init__(self, P_Gain=0.5):
        # P 제어기의 이득 설정
        self.P_Gain = P_Gain
        self.u = 0  # 제어 입력 (초기값 0)

    def ControllerInput(self, reference, measure):
        # 제어 입력 계산 (reference: 목표값, measure: 현재 측정값)
        error = reference - measure  # 목표와 현재 측정값의 차이(오차)
        self.u = self.P_Gain * error  # P 제어법을 적용하여 제어 입력 계산


if __name__ == "__main__":
    target_y = 0.0
    measure_y =[]
    time = []
    step_time = 0.1
    simulation_time = 30   
    plant = VehicleModel(step_time, 0.0, 0.99, 0.1)
    controller = P_Controller()
    
    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        measure_y.append(plant.y_measure[0][0])
        controller.ControllerInput(target_y, plant.y_measure[0][0])
        plant.ControlInput(controller.u)
    
    plt.figure()
    plt.plot([0, time[-1]], [target_y, target_y], 'k-', label="reference")
    plt.plot(time, measure_y,'r-',label = "Vehicle Position")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()
