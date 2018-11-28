#!/bin/bash

# This script trains the model on simulated dataset. It has merged 8 classes from original RGBD scene dataset from DARNN. 
# Every frame has a table and chair. Others are randomly picked up. 

set -x
set -e

export PYTHONUNBUFFERED="True"
export CUDA_VISIBLE_DEVICES=$1
#export LD_PRELOAD=/usr/lib/libtcmalloc.so.4

LOG="experiments/logs/blender_scene_multi_rgbd_1.txt.`date +'%Y-%m-%d_%H-%M-%S'`"
exec &> >(tee -a "$LOG")
echo Logging output to "$LOG"

# train FCN for multiple frames
time ./train_net.py --gpu 0 \
 --network vgg16 \
 --weights data/imagenet_models/vgg16_convs.npy \
 --imdb blender_scene_train \
 --cfg experiments/cfgs/blender_scene_1_multi_rgbd.yml \
 --iters 50000

# if [ -f $PWD/output/shapenet_scene/shapenet_scene_val/vgg16_fcn_rgbd_multi_frame_shapenet_scene_iter_40000/segmentations.pkl ]
# then
#   rm $PWD/output/shapenet_scene/shapenet_scene_val/vgg16_fcn_rgbd_multi_frame_shapenet_scene_iter_40000/segmentations.pkl
# fi

 # test FCN for multiple frames
# time ./test_net.py --gpu 0 \
#   --network vgg16 \
#   --model output/blender_scene_1/blender_scene_train/vgg16_fcn_rgbd_multi_frame_shapenet_scene_iter_40000.ckpt \
#   --imdb blender_scene_val \
#   --cfg experiments/cfgs/blender_scene_1_multi_rgbd.yml \
#   --rig data/BlenderScene/camera.json\
#   --kfusion 0

