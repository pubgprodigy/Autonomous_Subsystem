import numpy as np
from scipy import ndimage
import matplotlib.pyplot as plt
from sklearn.mixture import GaussianMixture
import cv2


img=cv2.imread("ball-2.png")
img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

img=img.astype(np.uint8)

mod_img=(3*img[:,:,1]-2*img[:,:,0]-2*img[:,:,2]+4*255)/7
hist, bin_edges = np.histogram(mod_img, bins=60)
bin_centers = 0.5*(bin_edges[:-1] + bin_edges[1:])

classif = GaussianMixture(n_components=2)
classif.fit(mod_img.reshape((mod_img.size, 1)))
sorted_classif = sorted(classif.means_)

result=img.copy()
for i in range(img.shape[0]):
	for j in range(img.shape[1]):
		if (((3*img[i][j][1]-2*img[i][j][0]-2*img[i][j][2]+4*255)/7)< sorted_classif[1][0]):
			result[i][j]=[0,0,0]
		else:
			result[i][j]=[255,255,255]

plt.figure(figsize=(11,4))

plt.subplot(131)
plt.imshow(img)
plt.axis('off')
plt.subplot(132)
plt.plot(bin_centers, hist, lw=2)
plt.axvline(0.5, color='r', ls='--', lw=2)
plt.text(0.57, 0.8, 'histogram', fontsize=20, transform = plt.gca().transAxes)
plt.yticks([])
plt.subplot(133)
plt.imshow(result, cmap=plt.cm.gray, interpolation='nearest')
plt.axis('off')

plt.subplots_adjust(wspace=0.02, hspace=0.3, top=1, bottom=0.1, left=0, right=1)
plt.show()