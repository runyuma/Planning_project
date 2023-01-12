import numpy as np

acc = np.ones((100,)) * 0.1
d_delta = np.zeros((100,))


data_all = np.array([acc,d_delta]).transpose()

with open('seq1.npy', 'wb') as f:
    np.save(f, data_all)
