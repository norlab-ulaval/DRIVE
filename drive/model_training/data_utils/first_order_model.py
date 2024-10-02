import numpy as np
from scipy.optimize import minimize

class FirstOrderModelWithDelay():

    def __init__(self,initial_gain,initial_time_constant, inital_delay) -> None:
        
        self.X =initial_gain,initial_time_constant,inital_delay 
        
    def foptd(self,t, K=1, tau=1, tau_d=0):
        #tau_d = max(0,tau_d)
        #tau = max(0,tau)

        test = K*(1-np.exp(-(t-tau_d)/tau))
        #print("test ", test.shape)
        return test

    def err(self,X,t,y):
        K,tau,tau_d = X
        z = self.foptd(t,K,tau,tau_d)


        iae = sum(abs((z-y)))
        #print("z shape",z.shape)
        #print("y shape",y.shape)
        return iae
    

    

    def train(self,t,u_step,y_centered,max_time_bounds):
        self.ts = t - t[0]
        self.us = u_step
        ys = y_centered/self.us
        #print(ys.shape)
        #print(self.ts)

        #print("ts",self.ts.shape)
        #print("ys",ys.shape)
        self.K,self.tau,self.tau_d = minimize(self.err,self.X,args=(self.ts,ys),bounds = [(None, None),(0.001,max_time_bounds),(0.001,max_time_bounds)] ).x

        
        return self.K, self.tau, self.tau_d, # gain, time cosntant, delay, step, point operation
    
    def train_all_calib_state(self,time_vec, u_step_array, y_centered_array,operatio_points,max_time_bounds):

        n_step = u_step_array.shape[0]
        time_constants_computed = np.zeros((n_step))
        time_delay_computed = np.zeros((n_step))
        gains_computed = np.zeros((n_step))


        
        predictions = np.zeros(y_centered_array.shape)

        for calib_step in range(n_step):
            t = time_vec
            #y = y_array[calib_step]
            u_step = u_step_array[calib_step]
            
            y_centered = y_centered_array[calib_step,:]
            #print("ustep",u_step)
            #print("ycentered",y_centered.shape)
            operation_point= operatio_points[calib_step]
            #print("operation_points",operation_point.shape)
            gains_computed[calib_step], time_constants_computed[calib_step], time_delay_computed[calib_step] =  self.train(t,u_step,y_centered,max_time_bounds)

            predictions[calib_step,:] = self.ypred(operation_point)
        return gains_computed, time_constants_computed, time_delay_computed,predictions
    
    def ypred(self,y_operation_pont):


        z = self.foptd(self.ts,self.K,self.tau,self.tau_d)
        ypred = y_operation_pont + z*self.us

        return ypred