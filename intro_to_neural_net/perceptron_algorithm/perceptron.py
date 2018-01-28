import numpy as np

# Setting the random seed
seed = 42
np.random.seed(seed)

# Check if the data was predicted as 1 or 0
def stepFunction(t):
    if t >= 0:
        return 1
    return 0

# X is the data vector of size n
# W is the weight vector of size n
# b is the bias
def prediction(X, W, b):
    return stepFunction((np.matmul(X, W) + b)[0])   # np.matmul() + b result is a scalar?

# TODO: Fill in the code below to implement the perceptron trick.
# The function should receive as inputs the data X, the labels y,
# the weights W (as an array), and the bias b,
# update the weights and bias W, b, according to the perceptron algorithm,
# and return W and b.
def perceptronStep(X, y, W, b, learn_rate = 0.01):
    n = len(X)
    m = 0
    for i in range(n):
        label = y[i]
        data = X[i]
        judgement = prediction(data, W, b)
        if judgement != label:  # misclassified
            # change the weight
            m = len(W)
            if judgement == 1:  # point is negative but on a positive area
                for j in range(m):
                    W[j] = W[j] + (learn_rate * data[j])
                b += learn_rate
            else:               # point is positive but on a negative area
                for j in range(m):
                    W[j] = W[j] - (learn_rate * data[j])
                b -= learn_rate
                pass
        else:
            # the data is correctly judged
            pass
    return W, b

# This function runs the perceptron algorithm repeatedly on the dataset,
# and returns a few of the boundary lines obtained in the iterations,
# for plotting purposes.
# Feel free to play with the learning rate and the num_epochs,
# and see your results plotted below.
def trainPerceptronAlgorithm(X, y, learn_rate=0.1, num_epochs=25):
    x_min, x_max = min(X.T[0]), max(X.T[0])
    y_min, y_max = min(X.T[1]), max(X.T[1])
    W = np.array(np.random.rand(2, 1))
    b = np.random.rand(1)[0] + x_max    # TODO: ??
    boundary_lines = [] # solution lines that gets plotted
    for i in range(num_epochs):
        # In each epoch, we apply the perceptron step
        W, b = perceptronStep(X, y, W, b, learn_rate)
        boundary_lines.append((-W[0]/W[1], -b/W[1]))

    return boundary_lines