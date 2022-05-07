import numpy as np, pandas as pd
from isotree import IsolationForest
import matplotlib.pyplot as plt
from pylab import rcParams
rcParams['figure.figsize'] = 10, 8

np.random.seed(1)
group1 = pd.DataFrame({
    "x" : np.random.normal(loc=-1, scale=.4, size = 1000),
    "y" : np.random.normal(loc=-1, scale=.2, size = 1000),
})
group2 = pd.DataFrame({
    "x" : np.random.normal(loc=+1, scale=.2, size = 1000),
    "y" : np.random.normal(loc=+1, scale=.4, size = 1000),
})
X = pd.concat([group1, group2], ignore_index=True)

### Now add an obvious outlier which is within the 1d ranges
### (As an interesting test, remove it and see what happens,
###  or check how its score changes when using sub-sampling)
X = X.append(pd.DataFrame({"x" : [-1], "y" : [1]}), ignore_index = True)

### Single-variable Isolatio Forest
iso_simple = IsolationForest(ndim=1, ntrees=100,
                             penalize_range=False,
                             prob_pick_pooled_gain=0)
iso_simple.fit(X)

### Extended Isolation Forest
iso_ext = IsolationForest(ndim=2, ntrees=100,
                          penalize_range=False,
                          prob_pick_pooled_gain=0)
iso_ext.fit(X)

### SCiForest
iso_sci = IsolationForest(ndim=2, ntrees=100, ntry=10,
                          penalize_range=True,
                          prob_pick_avg_gain=1,
                          prob_pick_pooled_gain=0)
iso_sci.fit(X)

### Fair-Cut Forest
iso_fcf = IsolationForest(ndim=2, ntrees=100,
                          penalize_range=False,
                          prob_pick_avg_gain=0,
                          prob_pick_pooled_gain=1)
iso_fcf.fit(X)

### Plot as a heatmap
pts = np.linspace(-3, 3, 250)
space = np.array( np.meshgrid(pts, pts) ).reshape((2, -1)).T
Z_sim = iso_simple.predict(space)
Z_ext = iso_ext.predict(space)
Z_sci = iso_sci.predict(space)
Z_fcf = iso_fcf.predict(space)
space_index = pd.MultiIndex.from_arrays([space[:, 0], space[:, 1]])

def plot_space(Z, space_index, X):
    df = pd.DataFrame({"z" : Z}, index = space_index)
    df = df.unstack()
    df = df[df.columns.values[::-1]]
    plt.imshow(df, extent = [-3, 3, -3, 3], cmap = 'hot_r')
    plt.scatter(x = X['x'], y = X['y'], alpha = .15, c = 'navy')

plt.suptitle("Outlier and Density Regions", fontsize = 20)

plt.subplot(2, 2, 1)
plot_space(Z_sim, space_index, X)
plt.title("Isolation Forest", fontsize=15)

plt.subplot(2, 2, 2)
plot_space(Z_ext, space_index, X)
plt.title("Extended Isolation Forest", fontsize=15)

plt.subplot(2, 2, 3)
plot_space(Z_sci, space_index, X)
plt.title("SCiForest", fontsize=15)

plt.subplot(2, 2, 4)
plot_space(Z_fcf, space_index, X)
plt.title("Fair-Cut Forest", fontsize=15)
plt.show()

### (As another interesting variation, try setting
###  'penalize_range=True' for the last model)

print("(Note that the upper-left corner has an outlier point,\n\
        and that there is a slight slide in the axes of the heat colors and the points)")