#!/usr/bin/env python

import path
from fcn.lab_test import test_net
from fcn.lab_test import test_net_single_frame
# from fcn.lab_test_local import test_net
# from fcn.lab_test_local import test_net_single_frame
from fcn.config import cfg, cfg_from_file
from datasets.factory import get_imdb
import argparse
import pprint
import time, os, sys
import tensorflow as tf
import os.path as osp


def parse_args():
    """
    Parse input arguments
    """
    parser = argparse.ArgumentParser(description='Test a Fast R-CNN network')
    parser.add_argument('--gpu', dest='gpu_id', help='GPU id to use',
                        default=0, type=int)
    parser.add_argument('--weights', dest='pretrained_model',
                        help='pretrained model',
                        default=None, type=str)
    parser.add_argument('--model', dest='model',
                        help='model to test',
                        default=None, type=str)
    parser.add_argument('--cfg', dest='cfg_file',
                        help='optional config file', default=None, type=str)
    parser.add_argument('--wait', dest='wait',
                        help='wait until net file exists',
                        default=True, type=bool)
    parser.add_argument('--imdb', dest='imdb_name',
                        help='dataset to test',
                        default='shapenet_scene_val', type=str)
    parser.add_argument('--network', dest='network_name',
                        help='name of the network',
                        default=None, type=str)
    parser.add_argument('--rig', dest='rig_name',
                        help='name of the camera rig file',
                        default=None, type=str)
    parser.add_argument('--kfusion', dest='kfusion',
                        help='run kinect fusion or not',
                        default=False, type=bool)

    if len(sys.argv) == 1:
        parser.print_help()
        sys.exit(1)

    args = parser.parse_args()
    return args


if __name__ == '__main__':
    args = parse_args()

    print('Called with args:')
    print(args)

    if args.cfg_file is not None:
        cfg_from_file(args.cfg_file)

    print('Using config:')
    pprint.pprint(cfg)

    weights_filename = os.path.splitext(os.path.basename(args.model))[0]

    imdb = get_imdb(args.imdb_name)

    cfg.GPU_ID = args.gpu_id
    device_name = '/gpu:{:d}'.format(args.gpu_id)
    print device_name

    cfg.TRAIN.NUM_STEPS = 1
    cfg.TRAIN.GRID_SIZE = cfg.TEST.GRID_SIZE
    if cfg.NETWORK == 'FCN8VGG':
        path = osp.abspath(osp.join(cfg.ROOT_DIR, args.pretrained_model))
        cfg.TRAIN.MODEL_PATH = path
    cfg.TRAIN.TRAINABLE = False

    from networks.factory import get_network

    network = get_network(args.network_name)
    print 'Use network `{:s}` in training'.format(args.network_name)

    # start a session
    saver = tf.train.Saver()
    if args.kfusion:
        gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=0.2)
        sess = tf.Session(config=tf.ConfigProto(allow_soft_placement=True, gpu_options=gpu_options))
    else:
        sess = tf.Session(config=tf.ConfigProto(allow_soft_placement=True))
    saver.restore(sess, args.model)
    print ('Loading model weights from {:s}').format(args.model)

    print 'test-------------------------------------------------------------------------'

    if cfg.TEST.SINGLE_FRAME:
        test_net_single_frame(sess, network, imdb, weights_filename, args.rig_name, args.kfusion)
    else:
        test_net(sess, network, imdb, weights_filename, args.rig_name, args.kfusion)
