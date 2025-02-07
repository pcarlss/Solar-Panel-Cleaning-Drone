import numpy as np
# Code from https://www.geeksforgeeks.org/kalman-filter-in-python/, validate before use
class KalmanFilter:
    def __init__(self, F, B, H, Q, R, x0, P0):
        self.F = F  # State transition model
        self.B = B  
        self.H = H  
        self.Q = Q  
        self.R = R  
        self.x = x0  
        self.P = P0  
    def predict(self, u):
        # Predict the state and state covariance
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x
    def update(self, z):
        # Compute the Kalman gain
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))      
        # Update the state estimate and covariance matrix
        y = z - np.dot(self.H, self.x)  
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.P.shape[0])
        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P), (I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)
        return self.x


if __name__ == "__main__":
    # Example usage
    F = np.array([[1, 1], [0, 1]]) 
    B = np.array([[0.5], [1]])     
    H = np.array([[1, 0]])         
    Q = np.array([[1, 0], [0, 1]]) 
    R = np.array([[1]])             
    # Initial state and covariance
    x0 = np.array([[0], [1]]) 
    P0 = np.array([[1, 0], [0, 1]]) 
    # Create Kalman Filter instance
    kf = KalmanFilter(F, B, H, Q, R, x0, P0)
    # Predict and update with the control input and measurement
    u = np.array([[1]])  
    z = np.array([[1]]) 
    # Predict step
    predicted_state = kf.predict(u)
    print("Predicted state:\n", predicted_state)
    # Update step
    updated_state = kf.update(z)
    print("Updated state:\n", updated_state)