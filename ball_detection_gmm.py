import numpy as np
from scipy import ndimage
import matplotlib.pyplot as plt
from sklearn.mixture import GaussianMixture
import cv2


img=cv2.imread("ball.png")
img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

img=img.astype(np.uint8)

mod_img=(3*img[:,:,1]-2*img[:,:,0]-2*img[:,:,2]+4*255)/7
hist, bin_edges = np.histogram(mod_img, bins=60)
bin_centers = 0.5*(bin_edges[:-1] + bin_edges[1:])

classif = GaussianMixture(n_components=2)
classif.fit(mod_img.reshape((mod_img.size, 1)))
sorted_classif = sorted(classif.means_)

result=img_gray.copy()
for i in range(img.shape[0]):
	for j in range(img.shape[1]):
		if (((3*img[i][j][1]-2*img[i][j][0]-2*img[i][j][2]+4*255)/7)< sorted_classif[1][0]):
			result[i][j]=0
		else:
			result[i][j]=255

def edge_contour(image):
	area_list=[]
	contour_list=[]
	edge_detected_image = cv2.Canny(image, 75, 200)
	_, contours, _= cv2.findContours(edge_detected_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	for contour in contours:
		approx = cv2.approxPolyDP(contour,0.01*cv2.arcLength(contour,True),True)
		area = cv2.contourArea(contour)
		area_list.append(area)
		if ( (len(approx) > 8) & (area > 30) ):
			contour_list.append(contour)
	return area_list, contour_list

area_result, contour_result = edge_contour(result)
x_c = np.mean(contour_result[1][:,0,0])
y_c = np.mean(contour_result[1][:,0,1])
x_max = np.max(contour_result[1][:,0,0])
y_max = np.max(contour_result[1][:,0,1])
x_range = 7*(x_max-x_c)
y_range = 7*(y_max-y_c)

x_min = int(max(0, x_c - x_range))
x_max = int(min(x_c + x_range, 255))
y_min = int(max(0, y_c - y_range))
y_max = int(min(y_c+y_range, 255))

img_extracted = img[ x_min:x_max, y_min:y_max, :]

img_extracted = img[ int(x_c - x_range) : int(x_c + x_range) , int(y_c - y_range) : int(y_c + y_range) , :]
bilateral_filtered_img = cv2.bilateralFilter(img_extracted, 5, 175, 175)
area_ext, contour_ext = edge_contour(img_extracted)

if (max(area_result)/max(area_ext) >= 0.7):
	print("Ball Detected around x =", x_c, "and y =", y_c, "of the image coordinates")
	print("Contour Area ratio = ", max(area_result)/max(area_ext) )
else:
	print("Ball Not Detected")

cv2.drawContours(result, contour_result,  -1, (255,0,0), 2)
cv2.drawContours(img, contour_ext,  -1, (255,0,0), 2)
cv2.drawContours(img_extracted, contour_ext,  -1, (255,0,0), 2)


plt.figure(figsize=(11,4))

plt.subplot(131)
plt.imshow(img)
plt.axis('off')
plt.subplot(132)
plt.imshow(img_extracted)
plt.axis('off')
plt.subplot(133)
plt.imshow(result, cmap=plt.cm.gray, interpolation='nearest')
plt.axis('off')

plt.subplots_adjust(wspace=0.02, hspace=0.3, top=1, bottom=0.1, left=0, right=1)
plt.show()