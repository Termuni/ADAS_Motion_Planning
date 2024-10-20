from vehicle_model import VehicleModel
import numpy as np
import matplotlib.pyplot as plt

class PID_Controller(object):
    def __init__(self, reference, measure, step_time):
        self.Kp = 0.4
        self.Kd = 0.9
        self.Ki = 0.02
        self.dt = step_time
        self.error_prev = reference - measure
        self.error_acc = 0
        self.alpha = 0.8
        self.u = 0.0
        
    def ControllerInput(self,reference,measure):
        self.error = (reference - measure)
        # self.error = self.alpha * (reference - measure) + (1-self.alpha) * self.error_prev
        self.error_d = (self.error - self.error_prev) / self.dt
        self.error_i = self.error_acc + self.error * self.dt
        self.u = self.Kp * self.error + self.Kd * self.error_d + self.Ki * self.error_i
        self.error_prev = self.error
        self.error_acc = self.error_i + self.error * self.dt
        
class LowPassFilter:
    def __init__(self, measure):
        self.y_estimate = measure
        self.alpha = 0.9
 
    def estimate(self, measure):
        self.y_estimate = self.alpha * self.y_estimate + (1-self.alpha) * measure


if __name__ == "__main__":
    target_y = 0.0
    measure_y =[]
    estimated_y = []
    time = []
    step_time = 0.1
    simulation_time = 30   
    plant = VehicleModel(step_time, 0.25, 0.99, 0.05)
    estimator = LowPassFilter(plant.y_measure[0][0])
    controller = PID_Controller(target_y, plant.y_measure[0][0], step_time)
    
    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        measure_y.append(plant.y_measure[0][0])
        estimated_y.append(estimator.y_estimate)
        estimator.estimate(plant.y_measure[0][0])
        controller.ControllerInput(target_y, estimator.y_estimate)
        plant.ControlInput(controller.u)
    
    plt.figure()
    plt.plot([0, time[-1]], [target_y, target_y], 'k-', label="reference")
    plt.plot(time, measure_y,'r-',label = "Vehicle Position(Measure)")
    plt.plot(time, estimated_y,'c-',label = "Vehicle Position(Estimator)")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()
