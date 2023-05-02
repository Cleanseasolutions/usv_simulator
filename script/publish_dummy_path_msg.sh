#!/bin/bash

rostopic pub /path nav_msgs/Path "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
poses:
- pose:
    position:
      x: 0.0
      y: -5.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
- pose:
    position:
      x: 30.0
      y: -5.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0"