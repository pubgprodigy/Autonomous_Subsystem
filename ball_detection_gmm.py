import numpy as np
from scipy import ndimage
import matplotlib.pyplot as plt
from sklearn.mixture import GaussianMixture
import cv2


img=cv2.imread("ball.png")
img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

img=img.astype(np.uint8)

# Penalize the red and blue channel to reduce the noise created in the thresholded image by white colour
mod_img=(3*img[:,:,1]-2*img[:,:,0]-2*img[:,:,2]+4*255)/7
hist, bin_edges = np.histogram(mod_img, bins=60)
bin_centers = 0.5*(bin_edges[:-1] + bin_edges[1:])

# Gaussian Mixture Model provided by sklearn module
classif = GaussianMixture(n_components=2)
classif.fit(mod_img.reshape((mod_img.size, 1))) 
sorted_classif = sorted(classif.means_) # sort the means of the gaussian obtained by applying GMM on the image

result=img_gray.copy() # equal to operator allots reference of the image so use the copy() function to get the shape of the ouput image
for i in range(img.shape[0]):
	for j in range(img.shape[1]):
# If the penalized image colour is ablove the threshold, assign it black colour. Else assign it white colour		
		if (((3*img[i][j][1]-2*img[i][j][0]-2*img[i][j][2]+4*255)/7)< sorted_classif[1][0]): 
			result[i][j]=0
		else:
			result[i][j]=255

# Function for edge detection and contour detecition in an image
# returns a list of detected circular contours and their areas in that image
def edge_contour(image):
	area_list=[] #To store the areas of the circular contours
	contour_list=[] # To store the image coordinates of the contours
	edge_detected_image = cv2.Canny(image, 75, 200) # Canny edge detection on the image- Procsssing the image before contour detection
	_, contours, _= cv2.findContours(edge_detected_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # Finding contours in the image
	# Now checking for circular contours in the contours detected by the above function
	for contour in contours: 
		approx = cv2.approxPolyDP(contour,0.01*cv2.arcLength(contour,True),True) #approxPolyDP calculates the percentage of arcLength or perimeter of the contour
# Approximates the contour with that information. The higher the percentage, the lower the number of vertices
		area = cv2.contourArea(contour)
		if ( (len(approx) > 8) & (area > 30) ): # Only consider significant circular contours - len(approx) is approximate number of vertices
			contour_list.append(contour)
			area_list.append(area)
	return area_list, contour_list

area_result, contour_result = edge_contour(result) # Evaluate area and contours of circles in resultant image
index = np.argmax(area_result) #index of the largest contour
# Centre of the circlular contour
x_c = np.mean(contour_result[index][:,0,0])
y_c = np.mean(contour_result[index][:,0,1])
# Maximum x and y coordinates of the circular contours. Required to extract the part of the original image that contains tennis ball
x_max = np.max(contour_result[1][:,0,0])
y_max = np.max(contour_result[1][:,0,1])
# Span of the extracted image that is expected to contain the ball in the original image 
x_range = 7*(x_max-x_c)
y_range = 7*(y_max-y_c)

# Make sure the span of the does not become negative or exceeds the width of the original image 
x_min = int(max(0, x_c - x_range))
x_max = int(min(x_c + x_range, img.shape[0]))
y_min = int(max(0, y_c - y_range))
y_max = int(min(y_c+y_range, img.shape[1]))

img_extracted = img[ x_min:x_max, y_min:y_max, :] #Extracted portion of the original image expected to contain the tennis ball

bilateral_filtered_img = cv2.bilateralFilter(img_extracted, 5, 175, 175) # Processing the RGB extracted image before edge and contour detection
area_ext, contour_ext = edge_contour(bilateral_filtered_img) # Performing this process on the extracted image rather than the complete original image is more efficient

# If the area of the largest circular contour in resultant image from GMM is within 70 percent of the area of the largest circular contour in the extracted image,
# Then we classify the image as containing the ball 
if (max(area_result)/max(area_ext) >= 0.7): 
	print("Ball Detected around x =", x_c, "and y =", y_c, "of the image coordinates")
	print("Contour Area ratio = ", max(area_result)/max(area_ext) )
else:
	print("Ball Not Detected")

# Visualize the contours detected

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