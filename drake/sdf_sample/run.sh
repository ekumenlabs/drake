#!/bin/bash

cd ../../
LC_ALL=C build/install/bin/drake-visualizer &
sleep 5
bazel-bin/drake/sdf_sample/sample drake/sdf_sample/models/mcity.sdf
cd drake/sdf_sample