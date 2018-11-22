import os
import matplotlib.pyplot as plt
import cv2
import numpy as np

base_path = '/home/weizhang/DA-RNN/data/RGBDScene/data/'


for i in xrange(1,len(os.listdir(base_path))+1):
	folder_path = os.path.join(base_path,'scene_{:02d}'.format(i))
	for j in xrange(int(len(os.listdir(folder_path))/4)):
		print i,j
		label_path = os.path.join(folder_path,'{:05d}-label.png'.format(j))
		rgb_path = os.path.join(folder_path,'{:05d}-color.png'.format(j))
		im_label = cv2.imread(label_path,-1)
		im_rgb = cv2.imread(rgb_path,-1)

		idx = np.where(im_label== 9)
		print idx[0].shape
		im_label[idx[0],idx[1]] = 5

		# idx = np.where(im_label == 0)

		# im_rgb[idx[0],idx[1],:] = (0,0,0)

		# plt.imshow(im_rgb)
		# plt.show()

		cv2.imwrite(label_path,im_label)
