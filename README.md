This repo is a proof of concept for some heurestic based mapping for a robot wandering around targets.

Given data.json which containts:
1. Time series of some poses for one robot relative to an origin
1. Time series of some poses for loads relative to the robot

Produce time series of load poses relative to origin complying with the conditions:
* The produced data is to be smooth i.e. implement a filtering mechanism
* The given time series from data.json are mismatched in time so they have to be processed before using.
* Given poses of loads relative to robots flip 180 degrees in angle often, the solution would require to provide a correct orientation.


Solution demo:

https://user-images.githubusercontent.com/43698361/182023452-6c452dad-5a40-4414-be3f-16035a98391f.mp4


Commands to build the project:

1. cd heurestic_based_mapping_warehouse_robot_poc
1. bazel build //main:main , this produces the output results which will be used to be plotted.
1. bazel run //object_tracking/test:unit_tests, this target runs the unit tests as the development followed the TDD appraoch
1. bazel run //main:plot, this target runs the plotting process


