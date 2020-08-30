import numpy as np

# Write a function that takes as input two lists Y, P,
# and returns the float corresponding to their cross-entropy.
def cross_entropy(Y, P):
    return -np.sum(Y*np.log(P)+(np.ones(len(Y))-Y)*np.log(np.ones(len(Y))-P))
