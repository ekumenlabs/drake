#!/bin/bash

cd ../../
build/install/bin/drake-visualizer &
sleep 5
bazel-bin/drake/sdf_sample/sample drake/sdf_sample/models/darpa_box.sdf
cd drake/sdf_sample
# sleep 20
# kill -9 $(pidof drake-visualizer)