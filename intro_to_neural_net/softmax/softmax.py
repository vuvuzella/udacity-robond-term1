import numpy as np 

def softmax(z):
    '''
    z - an array like input
    '''
    # Solution:
    # return np.exp(z) / np.sum(np.exp(z), axis=0)
    #
    z_arr = np.asarray(z)
    shape_arr_len = len(z_arr.shape)
    if shape_arr_len == 1:
        sum_exp = np.sum(np.exp(z_arr))
        return np.exp(z_arr) / sum_exp
    elif shape_arr_len == 2:
        # 2-dimensional array
        # each column 
        rows, cols = z_arr.shape
        n_soft = []
        for col in range(cols):
            col_vector = z_arr[:, col]
            sum_exp = np.sum(np.exp(col_vector))
            n_soft.append(np.exp(col_vector) / sum_exp)
        return np.transpose(n_soft)
    else:
        # None, not valid input
        return

if __name__ == '__main__':
    # logit1 is a one-dimensional array
    logit1 = [3.0, 1.0, 0.2]
    # logit2 is a two-dimensional array
    logit2 = np.array([
        [1, 2, 3, 6],
        [2, 4, 5, 6],
        [3, 8, 7, 6]])
    soft1 = softmax(logit1)
    # print(soft1)
    soft2 = softmax(logit2)
    print(soft2)