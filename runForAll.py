import os
import cv2

rawDataDir = '/home/shashank/Documents/deepL/SegNet-Tutorial-master/kitti/data_road/training/image_2/'
# deepLDataDir = '/home/shashank/Documents/deepL/SegNet-Tutorial-master/kitti/data_road/training/image_2_Segnet/'
deepLDataDir = '/home/shashank/Documents/deepL/SegNet-Tutorial-master/kitti/data_road/training/image_2_SegnetCityS/'
# resultsGC = '/home/shashank/Documents/deepL/SegNet-Tutorial-master/kitti/data_road/training/image_2_Segnet_GC/'
resultsGC = '/home/shashank/Documents/deepL/SegNet-Tutorial-master/kitti/data_road/training/image_2_SegnetCityS_GC/'

os.system("ls -1 "+rawDataDir+" > raw.txt")
os.system("ls -1 "+deepLDataDir+" > deepL.txt")

with open('raw.txt') as f:
    linesRaw = f.read().splitlines()

with open('deepL.txt') as f:
    linesDeepL = f.read().splitlines()


for x in xrange(0,len(linesRaw)):
	cmd = './driverFunc '+rawDataDir+linesRaw[x]+' '+deepLDataDir+linesDeepL[x]
	print cmd
	os.system(cmd)
	img = cv2.imread('data/segmentation_wi.png')
	imgBase = cv2.imread(rawDataDir+linesRaw[x])
	img = cv2.resize(img,(imgBase.shape[1],imgBase.shape[0]))
	img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	img[img>0] = 255
	# os.system('cp data/segmentation_wi.png '+resultsGC+linesRaw[x])
	s = linesRaw[x].split('_')
	file = '_'.join([s[0],'road',s[1]])
	print file
	cv2.imwrite(resultsGC+file,img)