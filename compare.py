import numpy as np
import cv2

img1 = cv2.imread('data/segmentation_wi.png')
img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)

img2 = cv2.imread('/home/shashank/Documents/deepL/SegNet-Tutorial-master/kitti/data_road/training/gt_image_2/uu_road_000060.png')
img2 = cv2.resize(img2, (img1.shape[1],img1.shape[0]))
cv2.imwrite("data/ans.png",img2)

y = np.zeros((img1.shape[0],img1.shape[1]))

tp = 0.0
fp = 0.0
fn = 0.0

for i in xrange(0,img1.shape[0]):
	for j in xrange(0,img1.shape[1]):
		if np.array_equal( img2[i][j] , [255,0,255] ) :
			y[i][j] = 100

img2 = y
# img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

for i in xrange(0,img1.shape[0]):
	for j in xrange(0,img1.shape[1]):
		if img1[i][j] > 0  and img2[i][j] > 0:
			tp += 1
		if img1[i][j] > 0  and img2[i][j] == 0:
			fp += 1
		if img1[i][j] == 0  and img2[i][j] > 0:
			fn += 1



# tp = np.sum(np.all(img1 > 0 and img2 > 0))
# fp = np.sum(np.where(img1 > 0 and img2 == 0))
# fn = np.sum(np.where(img1 == 0 and img2 > 0))

print "Precision : "+str(tp/(tp+fp))
print "Recall : "+str(tp/(tp+fn))

