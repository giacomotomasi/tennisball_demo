import numpy as np

a = np.zeros([3,1])
b = [1, 2, 3]
print("a:", a)
print("b:", b)
print(a.shape)


# 1d array to list
arr = np.array([1, 2, 3])
print(f'NumPy Array:\n{arr}')

list1 = arr.tolist()
print(f'List: {list1}')