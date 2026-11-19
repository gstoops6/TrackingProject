import numpy as np
import scipy.stats as stat
from time import perf_counter

a = np.ones((2, 1), dtype = np.float64)
at = a.T

a[1] = 2

print(np.outer(a, at))
