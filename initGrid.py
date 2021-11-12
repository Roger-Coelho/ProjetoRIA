import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

fig = plt.figure(figsize=(8,8), dpi=100)
ax = fig.add_subplot(111, aspect='equal')

map_size = np.array([20, 20])
cell_size = 1

rows, cols = (map_size/cell_size).astype(int)

m = np.random.uniform(low=0.0, high=1.0, size=(rows, cols))

m[0,0] = 1
plt.imshow(m, cmap='Greys', origin='upper', extent=(0, cols, rows, 0))

#ax.set_xticks(np.arange(0, cols, cell_size))
#ax.set_yticks(np.arange(0, rows, cell_size))

plt.colorbar()