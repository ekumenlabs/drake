# Description

Oneway is an implementation of Maliput's API that allows users to instantiate
a single lane dragway.  Such lane is a straight lane. The ends of the lane
are connected together via a "magical loop" that results in vehicles traveling
on the Oneway's lane instantaneously teleporting from one end of the lane to
the opposite end of the lane.

The lane length and width are all user specifiable.

# How to Run `oneway_to_urdf`

The executable `dragway_to_urdf` allows one create a URDF representation of a
dragway. To run `dragway_to_urdf`, execute:

    bazel run //drake/automotive/maliput/oneway:oneway_to_urdf -- \
          --dirpath=[dirpath] \
          --file_name_root=[file name root] \
          --lane_width=[lane width] \
          --length=[length of road in meters] \
          --num_lanes=[number of lanes] \
          --shoulder_width=[width of shoulder in meters]

For an explanation on what the above-mentioned parameters mean, execute:

    bazel run //drake/automotive/maliput/oneway:oneway_to_urdf -- --help

One the above command is executed, the following files should exist:

  1. `[dirpath]/[file name root].urdf` -- a [URDF](http://wiki.ros.org/urdf)
     description of the oneway.
  2. `[dirpath]/[file name root].obj` -- a
     [Wavefront OBJ](https://en.wikipedia.org/wiki/Wavefront_.obj_file) mesh of
     the oneway.
  3. `[dirpath]/[file name root].mtl` -- a material file that applies textures
     and colors to the above Wavefront OBJ file.
