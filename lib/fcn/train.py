from fcn.config import cfg
# from gt_data_layer.layer import GtDataLayer
# from gt_single_data_layer.layer import GtSingleDataLayer
from utils.timer import Timer
import numpy as np
import os
import tensorflow as tf
import sys
import threading

def get_training_roidb(imdb):
    """Returns a roidb (Region of Interest database) for use in training."""
    if cfg.TRAIN.USE_FLIPPED:
        print 'Appending horizontally-flipped training examples...'
        imdb.append_flipped_images()
        print 'done'

    return imdb.roidb