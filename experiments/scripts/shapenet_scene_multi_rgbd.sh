#!/bin/bash

set -x
set -e

export PYTHONUNBUFFERED="True"
export CUDA_VISIBLE_DEVICES=$1

#LOG="experiments/logs/rgbd_scene_multi_rgbd_test.txt.`date +'%Y-%m-%d_%H-%M-%S'`"
#exec &> >(tee -a "$LOG")
#echo Logging output to "$LOG"
#
#if [ -f $PWD/output/rgbd_scene/rgbd_scene_val/vgg16_fcn_rgbd_multi_frame_rgbd_scene_iter_40000/segmentations.pkl ]
#then
#  rm $PWD/output/rgbd_scene/rgbd_scene_val/vgg16_fcn_rgbd_multi_frame_rgbd_scene_iter_40000/segmentations.pkl
#fi

# test FCN for multiple frames
#time ./tools/lab_test_net.py --gpu 0 \
#  --network vgg16 \
#  --model data/fcn_models/shapenet_scene/vgg16_fcn_rgbd_multi_frame_shapenet_scene_iter_40000.ckpt \
#  --imdb shapenet_scene_val \
#  --cfg experiments/cfgs/shapenet_scene_multi_rgbd.yml \
#  --rig data/LabScene/camera.json \
#  --kfusion 1


time ./test_net_old.py --gpu 0 \
  --network vgg16 \
  --model output/shapenet_scene/shapenet_scene_train/vgg16_fcn_rgbd_multi_frame_shapenet_scene_iter_50000.ckpt \
  --imdb blender_scene_val \
  --cfg experiments/cfgs/shapenet_scene_multi_rgbd.yml \
  --rig data/LabScene/camera.json \
  --kfusion 0

#time ./tools/lab_test_net.py --gpu 0 \
#--network vgg16 \
#--model output/shapenet_scene/shapenet_scene_train/vgg16_fcn_depth_multi_frame_shapenet_scene_iter_30000.ckpt \
#--imdb shapenet_scene_val \
#--cfg experiments/cfgs/shapenet_scene_multi_depth.yml \
#--rig data/LabScene/camera.json \
#--kfusion 1



#time ./tools/lab_test_net.py --gpu 0 \
#  --network vgg16 \
#  --model output/shapenet_scene/shapenet_scene_train/vgg16_fcn_normal_multi_frame_shapenet_scene_iter_30000.ckpt \
#  --imdb shapenet_scene_val \
#  --cfg experiments/cfgs/shapenet_scene_multi_normal.yml \
#  --rig data/LabScene/camera.json \
#  --kfusion 1
