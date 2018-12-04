import os
import matplotlib.pyplot as plt
import cv2
import numpy as np

base_path = '/home/weizhang/Documents/domain-adaptation/data/BlenderScene/data/'


for i in xrange(1,len(os.listdir(base_path))+1):
	folder_path = os.path.join(base_path,'scene_{:02d}'.format(i))
	for j in xrange(int(len(os.listdir(folder_path))/4)):
		print i,j
		label_path = os.path.join(folder_path,'{:05d}_label.png'.format(j))
		rgb_path = os.path.join(folder_path,'{:05d}_color.png'.format(j))
		im_label = cv2.imread(label_path,-1)
		im_rgb = cv2.imread(rgb_path,-1)

		idx1 = np.where(im_label== 1)
		idx2 = np.where(im_label== 2)
		idx3 = np.where(im_label== 3)
		idx4 = np.where(im_label== 4)
		idx5 = np.where(im_label== 5)
		idx6 = np.where(im_label== 6)
		idx7 = np.where(im_label== 7)

		# print idx[0].shape
		im_label[idx1[0],idx1[1]] = 4
		im_label[idx2[0],idx2[1]] = 5
		im_label[idx3[0],idx3[1]] = 2
		im_label[idx4[0],idx4[1]] = 7
		im_label[idx5[0],idx5[1]] = 1
		im_label[idx6[0],idx6[1]] = 6
		im_label[idx7[0],idx7[1]] = 3

		# idx = np.where(im_label == 0)

		# im_rgb[idx1[0],idx1[1],:] = (0,0,0)
		# im_rgb[idx2[0],idx2[1],:] = (0,0,0)
		# im_rgb[idx3[0],idx3[1],:] = (0,0,0)
		# im_rgb[idx4[0],idx4[1],:] = (0,0,0)
		# im_rgb[idx5[0],idx5[1],:] = (0,0,0)
		# im_rgb[idx6[0],idx6[1],:] = (0,0,0)
		# im_rgb[idx7[0],idx7[1],:] = (0,0,0)

		# print im_label[288,279]

		# plt.imshow(im_rgb)
		# plt.show()

		# cv2.imwrite(label_path,im_label)
