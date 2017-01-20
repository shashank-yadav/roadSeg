import cv2
import numpy as np
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate

# x = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10 ,11, 12, 13, 14, 15])
# y = np.array([5, 7, 9, 11, 13, 15, 28.92, 42.81, 56.7, 70.59, 84.47, 98.36, 112.25, 126.14, 140.03])


img = cv2.imread('data/segmentation_wi.png')
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
img2 = cv2.imread('/home/shashank/Documents/deepL/SegNet-Tutorial-master/kitti/data_road/training/gt_image_2/uu_road_000086.png')
img2 = cv2.resize(img2, (img.shape[1],img.shape[0]))
img3 = cv2.imread('/home/shashank/Documents/deepL/SegNet-Tutorial-master/kitti/data_road/training/image_2/uu_000086.png')
img3 = cv2.resize(img3, (img.shape[1],img.shape[0]))

cv2.imwrite("data/ans.png",img3)


# edges = cv2.Sobel(img,cv2.CV_8UC1,1,0,ksize=5)
edges = cv2.Canny(img,50,150,apertureSize = 3)
# edges = cv2.Canny(img,50,150,apertureSize = 3)
# cv2.imwrite('data/segmentation_wi_edges.png',edges)

# x = []
# y = []

# for i in xrange(0,laplacian.shape[0]):
# 	for j in xrange(0,laplacian.shape[1]):
# 		if laplacian[i][j] > 0:
# 			# l.append((i,j))
# 			y.append(i*1.0)
# 			x.append(j)

# x = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10 ,11, 12, 13, 14, 15])
# y = np.array([5, 7, 9, 11, 13, 15, 28.92, 42.81, 56.7, 70.59, 84.47, 98.36, 112.25, 126.14, 140.03])


# tck = interpolate.splrep(np.array(x), np.array(y), k=2, s=0)


lines = cv2.HoughLines(edges,1,np.pi/180,30)

for i in xrange(0,lines.shape[0]):
	for rho,theta in lines[i]:
	    a = np.cos(theta)
	    b = np.sin(theta)
	    x0 = a*rho
	    y0 = b*rho
	    x1 = int(x0 + 1000*(-b))
	    y1 = int(y0 + 1000*(a))
	    x2 = int(x0 - 1000*(-b))
	    y2 = int(y0 - 1000*(a))

	    eps = np.pi/10
	    if theta > (np.pi/2-eps) and theta < (np.pi/2+eps):
	    	pass
	    else:	
	    	cv2.line(img2,(x1,y1),(x2,y2),(255,255,255),2)
	    
cv2.imwrite('data/houghlines3.jpg',img2)


# laplacian = np.gradient(img,2)