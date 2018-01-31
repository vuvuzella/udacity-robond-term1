import numpy as np

# Setting the random seed
seed = 42
np.random.seed(42)

def sigmoid(x):
    return 1 / 1 + np.exp(-x)

def sigmoid_prime(x):
    return sigmoid(x) * (1 - sigmoid(x))

# X - array of data
# W - array of weights
# b - scalar bias
def prediction(X, W, b):
    # sigmoid(Wx + b)
    return sigmoid(np.matmul(W, x) + b)

def error_vector(y, y_hat):
    return [ -y[i]*np.log(y_hat[i]) - (1 - y[i])*(np.log(1 - y_hat[i]))  for i in range(len(y))]

# Function to average out the error vector (1/m where m is the number of points/data)
def error(y, y_hat):
    ev = error_vector(y, y_hat)
    return sum(ev)/len(ev)

# Fill in the code below to calculate the gradient of the error function.
# The result should be a list of three lists:
# The first list should contain the gradient (partial derivatives) with respect to w1
# The second list should contain the gradient (partial derivatives) with respect to w2
# The third list should contain the gradient (partial derivatives) with respect to b
def dErrors(X, y, y_hat):
    # print(X)
    # DErrorsDx1 = -(y - y_hat) * X[:, 0]
    # DErrorsDx2 =  -(y - y_hat) * X[:, 1]
    # DErrorsDb = -(y - y_hat)
    DErrorsDx1 = [X[i][0]*(y[i]-y_hat[i]) for i in range(len(y))]
    DErrorsDx2 = [X[i][1]*(y[i]-y_hat[i]) for i in range(len(y))]
    DErrorsDb = [y[i]-y_hat[i] for i in range(len(y))]
    return DErrorsDx1, DErrorsDx2, DErrorsDb

# TODO: Fill in the code below to implement the gradient descent step.
# The function should receive as inputs the data X, the labels y,
# the weights W (as an array), and the bias b.
# It should calculate the prediction, the gradients, and use them to
# update the weights and bias W, b. Then return W and b.
# The error e will be calculated and returned for you, for plotting purposes.
def gradientDescentStep(X, y, W, b, learn_rate=0.01):
    # Calculate the prediction
    # TODO: Calculate the gradient
    # TODO: Update the weights
    # This calculates the error
    y_hat = prediction(X, W, b)
    # DErrorsDx1, DErrorsDx2, DErrorsDb = dErrors(X, y, y_hat)
    e = error(y, y_hat)
    # W = W - learn_rate * DErrorsDx1
    # b = b - learn_rate * DE
    y_hat = prediction(X,W,b)
    errors = error_vector(y, y_hat)
    derivErrors = dErrors(X, y, y_hat)
    W[0] += sum(derivErrors[0])*learn_rate
    W[1] += sum(derivErrors[1])*learn_rate
    b += sum(derivErrors[2])*learn_rate
    return W, b, e

# This function runs the perceptron algorithm repeatedly on the dataset,
# and returns a few of the boundary lines obtained in the iterations,
# for plotting purposes.
# Feel free to play with the learning rate and the num_epochs,
# and see your results plotted below.
def trainLR(X, y, learn_rate=0.01, num_epochs=100):
    x_min, x_max = min(X.T[0]), max(X.T[0])
    y_min, y_max = min(X.T[1]), max(X.T[1])
    # Initialize the weights randomly
    W = np.array(np.random.rand(2, 1))*2 - 1
    b = np.random.rand(1)[0]*2 - 1
    # Solution lines that get plotted 
    boundary_lines = []
    errors = []
    for i in range(num_epochs):
        # In each epoch, we apply the gradient descent step
        W, b, error = gradientDescentStep(X, y, W, b, learn_rate)
        boundary_lines.append((-W[0]/W[1], -b/W[1]))
        errors.append(error)
    return boundary_lines, errors

if __name__ == "__main__":
    # TODO: Read in data.csv into numpy arrays
    # TODO: Train Logistic Regression
    # TODO: Plot the result