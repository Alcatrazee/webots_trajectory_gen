from matplotlib import pyplot
import numpy as np
import math
from matplotlib.pyplot import plot

def get_derivative(poly_order,derivative_order,t):
    cof_array = np.ones((derivative_order+1,poly_order+1))

    for i in range(cof_array.shape[0]):
        for j in range(cof_array.shape[1]):
            if j>=(poly_order + 1 - i):
                cof_array[i,j:] = 0
                continue
            else:
                cof_array[i,j] = math.factorial(int(poly_order-j))/math.factorial(int(poly_order-i-j))*t**int(poly_order-i-j)

    return cof_array

def trajectory_generation(initial_state,end_state,T,steps):
    poly_n = 7


    b = np.concatenate((initial_state,end_state),axis=0).T

    A = np.zeros((8,poly_n+1))
    A[0][7] = 1
    A[1][6] = 1
    A[2][5] = 1
    A[3][4] = 1

    A[4:][:] = get_derivative(poly_n,3,T)

    x = np.dot(np.linalg.pinv(A),b)
    t_series = np.linspace(0,T,steps)

    s_series = np.polyval(x,t_series)

    return t_series,s_series

if __name__ == "__main__":
    initial_state = np.array([0,0,0,0])
    end_state = np.array([10,0,0,0])
    duration = 1
    steps = 1000
    t_series,s_series = trajectory_generation(initial_state,end_state,duration,steps)
    plot(t_series, s_series)
    pyplot.show()



