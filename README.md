# Lerobot
## Introduction
This reposity dirives from the official Lerobot commit around [May 25th 2025](https://github.com/huggingface/lerobot/tree/bed90e3a41c43758c619dba66158ddd5798d361a).

### Features
1. It provide script to control follower arm only.
2. A simple IK algorithm by referring to [Analytical IK for SO101](https://vectorwang.notion.site/Analytical-IK-for-SO101-20abb280f59380d288bcdd398ffbfab1). The latest official Lerobot repo also provides an IK module which I think is more general.
3. Merge DepthAnything (DA) feature with ResNet feature in ACT to make prediction better. Many works have demonstrate the effectiveness of 3D infomation. We use DA to compensate the ACT encoder feature.

## Installation

Follow the instruction of [official doc](https://github.com/huggingface/lerobot/tree/bed90e3a41c43758c619dba66158ddd5798d361a).

### What's more
Get submodule of DepthAnything

`git submodule update --init --recursive`

## Train
Modify params in train.sh and run with `sh train.sh`

## Follower Control
Please refer code in `lerobot/scripts/control_follower_robot.py`.

We add `so100_follower` and `so101_follower` in `lerobot/common/robot_devices/robots/configs.py`. Remember to fix the `port` in the follower config.


## TODO
- [ ] Add config to swith ResNet and ResNet+DA
- [ ] Add config to control DA input image size
- [ ] Add experiment results
- [ ] Add introduction of cus

