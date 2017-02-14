#!/bin/bash

cd ../../
build/install/bin/drake-visualizer &
sleep 5
bazel-bin/drake/sdf_sample/sample drake/sdf_sample/models/darpa.urdf
cd drake/sdf_sample
kill -9 $(pidof drake-visualizer)