# Drake Docker images

## Description

In this directory you will find different Drake relative dockerfiles. 

## Available images

You can find the following images:

* ```drake```: it contains all the dependencies to Drake, Ignition Math and Manifold. Also you have system wide installed Ignition Math and Manifold. Drake's repository
is a clone of the Ekumenlabs fork.

## Graphic card drivers

At the moment there are only one graphic card driver and it's for Intel. We will support NVidia in the future.

## Default user

Use ```drake``` user as default. It's a passwordless sudo too.

## Building the image

To build the image:

```
make drake
```

## Running a container

To run a container:

```
./run.sh
``` 
 
