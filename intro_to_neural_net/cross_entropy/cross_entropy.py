# import cross_entropy.py
import numpy as np
import math

# Write a function that takes as input two lists Y, P,
# and returns the float corresponding to their cross-entropy.
def cross_entropy(Y, P):
    if len(Y) != len(P):
        print('Lengths of success and probabilities must be same')
        return

    sum = 0.0
    # iterative version
    for i in range(len(P)):
        sum += (Y[i] * math.log(P[i])) + ((1 - Y[i]) * math.log(1 - P[i]))

    # Numpy pythonic solution
    # Y = np.float_(Y)
    # P = np.float_(P)
    # return -np.sum(Y * np.log(P) + (1 - Y) * np.log(1 - P))
    return -sum

if __name__ == '__main__':
    success = (1, 1, 0)
    prob = (0.8, 0.7, 0.1)
    xtropy = cross_entropy(success, prob)
    print('Cross entropy: ' + str(xtropy))