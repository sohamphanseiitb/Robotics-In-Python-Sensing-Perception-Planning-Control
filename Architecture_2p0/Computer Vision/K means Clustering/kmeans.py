"""
DOCSTRING:
We implement a K-means clustering class in this file, it has all the necessary supporting functions and methods.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import cv2
import time
import tqdm
import copy
from matplotlib.colors import ListedColormap

class KMeans():

    def __init__(self, image, k, T, imshape):
        self.data = image
        self.k = k
        self.N, self.d = np.shape(self.data)
        self.T = T
        self.imshape = imshape

        if self.k >= self.N :
            raise ValueError('Number of clusters is greater than/equal to total number of data points available.')
    
    def random_centroids(self):

        self.centroids = self.data[np.random.choice(self.N, size=self.k, replace=False)]
        return self.centroids

    def dist2c(self):

        """
        data: N * d
        cent: c * d
        outp: c * N
        """
        self.distmat = np.sqrt(np.sum((self.data[:, np.newaxis, :] - self.centroids[np.newaxis, :, :]) ** 2, axis=2))
        return self.distmat
    
    def nearest_centroid(self, axis=1):

        # for every point, find which centroid is the closest
        self.cluster_vals = np.min(self.distmat, axis)
        self.cluster_idx = np.argmin(self.distmat, axis)

        self.cluster_data = [self.cluster_vals, self.cluster_idx]

    def find_pts(self):
        """
        This function finds all the datapoints for which a particular centroid is the closest, and then assign that point with a cluster ID
        """

        self.datapts = []

        for id in range(self.k):

            buffer = []

            for i in range(len(self.cluster_idx)):
                if self.cluster_idx[i] == id:
                    buffer.append(i)
            
            self.datapts.append(buffer)
            buffer = []

    def KMeansLoss(self):
        self.loss = np.zeros(self.k)
        for i in range(self.k):
            for j in self.datapts[i]:
                self.loss[i] += np.linalg.norm(self.data[j] - self.centroids[i])**2
        return self.loss

    
    def MyKMeans(self):

        self.store_centroids = np.zeros([self.T, self.k, self.d])
        self.KMloss = np.zeros([self.T, self.k])

        for i in tqdm.tqdm(range(self.T)):

            self.distmat = self.dist2c()
            self.nearest_centroid()
            self.find_pts()

            averages = np.zeros([self.k, self.d])
            for j in range(self.k):
                averages[j] = np.mean(self.data[self.datapts[j]], axis=0)

            self.store_centroids[i] = averages

            self.centroids = averages
            self.KMloss[i] = self.KMeansLoss()
    
    def visualize(self):

        self.d = np.shape(self.centroids)[1]

        ### Insert your solution here ###
        # first create a figure and set projection axes as 3D
        fig = plt.figure(figsize=(20, 10))
        ax1 = fig.add_subplot(121, projection='3d')
        ax2 = fig.add_subplot(122)

        # create the colors array as per the number of centroids
        #colors = plt.cm.get_cmap('tab20', k)
        #cmap = ListedColormap([colors(i) for i in range(k)])
        cmap = plt.cm.get_cmap('tab10', self.k);

        self.sample = np.random.choice(np.arange(self.data.shape[0]), size=500, replace=False)

        # create the sample from 'data' and sample_idx
        sample = self.data[self.sample]
        cluster_idsample = self.cluster_idx[self.sample]

        # Create 3D scatter plot with corresponding colours as first sub-plot
        scatter_plot = ax1.scatter(sample[:, 0], sample[:, 1], sample[:, 2], c=cluster_idsample, cmap=cmap)
        centroids_plot = ax1.scatter(self.centroids[:, 0], self.centroids[:, 1], self.centroids[:, 2], s=400, c='black', marker='x')
        legend1 = ax1.legend(*scatter_plot.legend_elements(), loc="upper right", title="Cluster ID")
        legend2 = ax1.legend([centroids_plot], ['Centroids'], loc="upper left", title="Markers")
        ax1.set_title('Scatter Plot of Clusters and Data Points', size=20)
        ax1.add_artist(legend1)
        ax1.set_xlabel('X', size=20)
        ax1.set_ylabel('Y', size=20)
        ax1.set_zlabel('Z', size=20)

        # Create visualization of the clustered image as second sub-plot

        # the array 'cluster_idx' is sufficient for plotting the clustered image
        ## reshape the cluster_idx into the original image shape
        clustered_image = self.cluster_idx.reshape(self.imshape[:2])

        ## plot
        ax2.imshow(clustered_image, cmap=cmap)
        ax2.set_title('Clustered Image', size=20)

        plt.savefig("kmeans_visualization.png", bbox_inches='tight')
        plt.show()
