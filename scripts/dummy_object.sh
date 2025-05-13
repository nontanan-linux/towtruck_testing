#!/bin/bash

ros2 topic pub -1 /simulation/dummy_perception_publisher/object_info dummy_perception_publisher/msg/Object "
header:
  stamp:
    sec: 1741769813
    nanosec: 298234608
  frame_id: map
id:
  uuid:
  - 37
  - 80
  - 117
  - 242
  - 115
  - 14
  - 116
  - 135
  - 187
  - 0
  - 219
  - 251
  - 15
  - 136
  - 94
  - 250
initial_state:
  pose_covariance:
    pose:
      position:
        x: -4.710485458374023
        y: 131.7153778076172
        z: 1.0
      orientation:
        x: 0.0
        y: 0.0
        z: -0.6941514886369103
        w: 0.7198289455302289
    covariance:
    - 0.0008999999845400453
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0008999999845400453
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0008999999845400453
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.007615434937179089
  twist_covariance:
    twist:
      linear:
        x: 0.0
        y: 0.0
        z: 0.0
      angular:
        x: 0.0
        y: 0.0
        z: 0.0
    covariance:
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
  accel_covariance:
    accel:
      linear:
        x: 0.0
        y: 0.0
        z: 0.0
      angular:
        x: 0.0
        y: 0.0
        z: 0.0
    covariance:
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
classification:
  label: 7
  probability: 1.0
shape:
  type: 1
  footprint:
    points: []
  dimensions:
    x: 2.8
    y: 1.3
    z: 2.0
max_velocity: 33.29999923706055
min_velocity: -33.29999923706055
action: 0
"
