# svm.py
import numpy as np
import matplotlib.pyplot as plt
from sklearn import svm
from generate_clusters import cluster_gen

np.random.seed(100) # Change the number to generate a different cluster

n_clusters = 5
clusters_x, clusters_y, labels = cluster_gen(n_clusters)

# Convert to a training dataset in sklearn format
X = np.float32((np.concatenate(clusters_x), np.concatenate(clusters_y))).transpose()    # feature set vector
y = np.float32((np.concatenate(labels)))    # labels vector

# Create an instance of SVM and fit the data
ker = 'rbf'  # rbf, poly, sigmoid, precomputed, callable function
svc = svm.SVC(kernel=ker).fit(X, y)

# Create a mesh that we will use to colorfully plot the decision surface
# Plotting Routine courtesy of:
# http://scikit-learn.org/stable/auto_examples/svm/plot_iris.html#sphx-glr-auto-examples-svm-plot-iris-py
# Note: this coloring scheme breaks down at > 7 clusters or so
h = 0.2 # step size in mesh
x_min, x_max = X[:,0].min() - 1, X[:, 0].max() + 1    # -+1 to add some margin
y_min, y_max = X[:,1].min() - 1, X[:, 1].max() + 1

mesh_x, mesh_y = np.meshgrid(np.arange(x_min, x_max), np.arange(y_min, y_max))

# Classify each block of the mesh (used to assign its color)
Z = svc.predict(np.c_[mesh_x.ravel(), mesh_y.ravel()])

# Put the result into a color plot
Z = Z.reshape(mesh_x.shape)
plt.contourf(mesh_x, mesh_y, Z, cmap=plt.cm.coolwarm, alpha=0.8)

# Plot the training points
plt.scatter(X[:, 0], X[:, 1], c=y, cmap=plt.cm.coolwarm, edgecolors='black')
plt.xlim(mesh_x.min(), mesh_x.max())
plt.ylim(mesh_y.min(), mesh_y.max())
plt.xticks(())
plt.yticks(())
plt.title('SVC with '+ker+' kernel', fontsize=20)
plt.show()