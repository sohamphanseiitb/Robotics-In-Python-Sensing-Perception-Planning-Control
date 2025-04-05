import cv2
import matplotlib.pyplot as plt
import numpy as np
from kmeans import KMeans

img = cv2.cvtColor(cv2.imread("Architecture_2p0\\Computer Vision\\images\\emma_stonee.jpg"), cv2.COLOR_BGR2RGB)

# Create a KMeans object
np.random.seed(17)
img2d = np.float32(img.reshape((-1, 3)))

KM = KMeans(img2d, 6, 5, np.shape(img))
KM.random_centroids()
KM.MyKMeans()
KM.visualize()
plt.show(KM.clustered_image)
