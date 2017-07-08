# aerial-mapper

### Overview
- Load camera poses from different formats (PIX4D, COLMAP, etc.)
- Generates a dense point cloud from raw images, camera poses and camera intriniscs
- Generates Digital Surface Models (DSMs) from raw point clouds and exports e.g. to GeoTiff
- Generates (ortho-)mosaics from raw images, camera poses and camera intrinsics


### Package Overview
- [**aerial_mapper:**](https://github.com/ethz-asl/aerial_mapper/tree/master/aerial_mapper) Meta package
- [**aerial_mapper_demos_no_ros:**](https://github.com/ethz-asl/aerial_mapper/tree/master/aerial_mapper_demos_no_ros) Sample executables that do not require ROS.
- [**aerial_mapper_demos_ros:**](https://github.com/ethz-asl/aerial_mapper/tree/master/aerial_mapper_demos_ros) Sample executables that require ROS.
- [**aerial_mapper_dense_pcl:**](https://github.com/ethz-asl/aerial_mapper/tree/master/aerial_mapper_dense_pcl) Dense point cloud generation using planar rectification.
- [**aerial_mapper_dsm:**](https://github.com/ethz-asl/aerial_mapper/tree/master/aerial_mapper_dsm) Digitial Surface Map/Model generation.
- [**aerial_mapper_grid_map:**](https://github.com/ethz-asl/aerial_mapper/tree/master/aerial_mapper_grid_map) Wrapper package for grid_map.
- [**aerial_mapper_io:**](https://github.com/ethz-asl/aerial_mapper/tree/master/aerial_mapper_io) Input/Output handler that reads/writes poses, intrinsics, point clouds, GeoTiffs etc.
- [**aerial_mapper_ortho:**](https://github.com/ethz-asl/aerial_mapper/tree/master/aerial_mapper_ortho) (Ortho-)Mosaic generation.
- [**aerial_mapper_ros:**](https://github.com/ethz-asl/aerial_mapper/tree/master/aerial_mapper_ros) Wrapper for ROS-dependent methods.
- [**aerial_mapper_thirdparty:**](https://github.com/ethz-asl/aerial_mapper/tree/master/aerial_mapper_thirdparty) Package containing thirdparty code.
- [**aerial_mapper_utils:**](https://github.com/ethz-asl/aerial_mapper/tree/master/aerial_mapper_utils) Package for common utility functions.

<hr>

### Publications
If you use this work in an academic context, please cite the following publication(s):

T. Hinzmann, J. L. Schönberger, M. Pollefeys, and R. Siegwart, **"Mapping on the Fly: Real-time 3D Dense Reconstruction, Digital Surface Map and Incremental Orthomosaic Generation for Unmanned Aerial Vehicles"**

```
@INPROCEEDINGS{fsr_hinzmann_2017,
   Author = {T. Hinzmann, J. L. Schönberger, M. Pollefeys, and R. Siegwart},
   Title = {Mapping on the Fly: Real-time 3D Dense Reconstruction, Digital Surface Map and Incremental Orthomosaic Generation for Unmanned Aerial Vehicles},
   Booktitle = {Field and Service Robotics - Results of the 11th International Conference},
   Year = {2017}
}
```
