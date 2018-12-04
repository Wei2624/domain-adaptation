from .imdb import imdb
from .blender_scene import blender_scene
from .shapenet_scene import shapenet_scene
from . import factory
import os.path as osp
ROOT_DIR = osp.join(osp.dirname(__file__), '..', '..')
