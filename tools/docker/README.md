# Drake's Docker

## Description

This Dockerfile loads all the dependencies of Drake's repository, clones the Github fork of [Ekumen](https://github.com/ekumenlabs) and builds it from **master** branch.

This image also builds drivers for Intel graphic cards. Future work will include NVidia drivers.

## Building the image

Just run:

```
make drake
```

And the Makefile will do it for you.

## Running the container

Just do:

```
./run.sh [CONTAINER_NAME] [IMAGE:TAG]
```

For example:

```
./run.sh drake drake
```
