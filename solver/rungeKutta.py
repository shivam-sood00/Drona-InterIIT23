import numpy as np
def rk4(A, B, C, X, U, dt, no_of_steps):
    for _ in range(no_of_steps):
        k1 = (dt/no_of_steps) * (A @ X + B @ U + C)
        k2 = (dt/no_of_steps) * (A @ X + A @ (0.5 * k1) + B @ U + C)
        k3 = (dt/no_of_steps) * (A @ X + A @ (0.5 * k2) + B @ U + C)
        k4 = (dt/no_of_steps) * (A @ X + A @ (k3) + B @ U + C)
        X = X + (1/6) * (k1 + 2*k2 + 2*k3 + k4)
    return X
    

