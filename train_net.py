#!/usr/bin/env python

import path
from fcn.train import get_training_roidb, train_net
from fcn.config import cfg, cfg_from_file, get_output_dir
from datasets.factory import get_imdb
import argparse
import pprint
import numpy as np
import sys
import os


def parse_args():
	parser = argparse.ArgumentParser(description='Train a network')
	parser.add_argument('--gpu', dest='gpu_id',
						help='GPU device id to use [0]',
						default=0, type=int)
	parser.add_argument('--iters', dest='max_iters',
						help='number of iterations to train',
						default=40000, type=int)
	parser.add_argument('--weights', dest='pretrained_model',
						help='initialize with pretrained model weights',
						default=None, type=str)
	parser.add_argument('--cfg', dest='cfg_file',
						help='optional config file',
						default=None, type=str)
	parser.add_argument('--imdb', dest='imdb_name',
						help='dataset to train on',
						default='shapenet_scene_train', type=str)
	parser.add_argument('--rand', dest='randomize',
						help='randomize (do not use a fixed seed)',
						action='store_true')
	parser.add_argument('--network', dest='network_name',
						help='name of the network',
						default=None, type=str)

	if len(sys.argv) == 1:
		parser.print_help()
		sys.exit(1)

	args = parser.parse_args()
	return args



if __name__ == '__main__':
	args = parse_args()

	# print('Called with args:')
	# print(args)


	if args.cfg_file is not None:
		cfg_from_file(args.cfg_file)

	print('Using config:')
	pprint.pprint(cfg)
	if not args.randomize:
		# fix the random seeds (numpy and caffe) for reproducibility
		np.random.seed(cfg.RNG_SEED)

	imdb = get_imdb(args.imdb_name)
	print 'Loaded dataset `{:s}` for training'.format(imdb.name)
	roidb = get_training_roidb(imdb)

	output_dir = get_output_dir(imdb, None)
	print 'Output will be saved to `{:s}`'.format(output_dir)


	device_name = '/gpu:{:d}'.format(args.gpu_id)
	cfg.GPU_ID = args.gpu_id
	print device_name


	pretrained_model = args.pretrained_model

	from networks.factory import get_network
	network = get_network(args.network_name)

	# import ipdb
	# ipdb.set_trace()

	print 'Use network `{:s}` in training'.format(args.network_name)

	train_net(network, imdb, roidb, output_dir,
				pretrained_model=pretrained_model,
				max_iters=args.max_iters)