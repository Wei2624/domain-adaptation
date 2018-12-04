import os
import sys



for i in range(1, 15):
	print i
	base_path = '/home/weizhang/Documents/domain-adaptation/data/BlenderScene/data/scene_{:02d}'.format(i)

	for j in range(int(len(os.listdir(base_path))/4)):
		rgb_path = os.path.join(base_path,'{:05d}-color.png'.format(j))
		rgb_path_new = os.path.join(base_path,'{:05d}_rgba.png'.format(j))

		label_path = os.path.join(base_path,'{:05d}-label.png'.format(j))
		label_path_new = os.path.join(base_path,'{:05d}_label.png'.format(j))


		depth_path = os.path.join(base_path,'{:05d}-depth.png'.format(j))
		depth_path_new = os.path.join(base_path,'{:05d}_depth.png'.format(j))


		meta_path = os.path.join(base_path,'{:05d}-meta.mat'.format(j))
		meta_path_new = os.path.join(base_path,'{:05d}_meta.mat'.format(j))



		os.rename(rgb_path, rgb_path_new)
		os.rename(label_path, label_path_new)
		os.rename(depth_path, depth_path_new)
		os.rename(meta_path, meta_path_new)