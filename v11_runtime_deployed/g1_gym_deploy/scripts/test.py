import numpy as np

a = np.load("recored_mocapbase.npy")
for _ in range(len(a)):
    print(a[_])
