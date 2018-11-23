__sets = {}

import networks.vgg16
import tensorflow as tf
from fcn.config import cfg

# if cfg.TRAIN.SINGLE_FRAME:
#     if cfg.NETWORK == 'VGG16':
#         __sets['vgg16_convs'] = networks.vgg16_convs(cfg.INPUT, cfg.TRAIN.NUM_CLASSES, cfg.TRAIN.NUM_UNITS, cfg.TRAIN.SCALES_BASE, cfg.TRAIN.VERTEX_REG, cfg.TRAIN.TRAINABLE)
#     if cfg.NETWORK == 'RESNET50':
#         __sets['resnet50'] = networks.resnet50(cfg.INPUT, cfg.TRAIN.NUM_CLASSES, cfg.TRAIN.SCALES_BASE)
#     if cfg.NETWORK == 'FCN8VGG':
#         __sets['fcn8_vgg'] = networks.fcn8_vgg(cfg.TRAIN.NUM_CLASSES, cfg.TRAIN.MODEL_PATH)
# else:
#     __sets['vgg16'] = networks.vgg16(cfg.INPUT, cfg.TRAIN.NUM_STEPS, cfg.TRAIN.NUM_CLASSES, cfg.TRAIN.NUM_UNITS, cfg.TRAIN.SCALES_BASE)

__sets['vgg16'] = networks.vgg16(cfg.INPUT, cfg.TRAIN.NUM_STEPS, cfg.TRAIN.NUM_CLASSES, cfg.TRAIN.NUM_UNITS, cfg.TRAIN.SCALES_BASE)


def get_network(name):
    """Get a network by name."""
    if not __sets.has_key(name):
        raise KeyError('Unknown network: {}'.format(name))
    return __sets[name]

def list_networks():
    """List all registered imdbs."""
    return __sets.keys()