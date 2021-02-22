# aerial-mapper 


### Overview
- Load camera poses from different formats (such as [PIX4D](https://pix4d.com/), [COLMAP](https://colmap.github.io/))
- Generates a dense point cloud from raw images, camera poses and camera intrinsics
- Generates Digital Surface Models (DSMs) from raw point clouds and exports e.g. to [GeoTiff](http://www.gdal.org/frmt_gtiff.html) format
- Different methods to generate (ortho-)mosaics from raw images, camera poses and camera intrinsics


### Package Overview
- [**aerial_mapper:**](https://github.com/ethz-asl/aerial_mapper/tree/master/aerial_mapper) Meta package
- [**aerial_mapper_demos:**](https://github.com/ethz-asl/aerial_mapper/tree/master/aerial_mapper_demos) Sample executables.
- [**aerial_mapper_dense_pcl:**](https://github.com/ethz-asl/aerial_mapper/tree/master/aerial_mapper_dense_pcl) Dense point cloud generation using planar rectification.
- [**aerial_mapper_dsm:**](https://github.com/ethz-asl/aerial_mapper/tree/master/aerial_mapper_dsm) Digitial Surface Map/Model generation.
- [**aerial_mapper_google_maps_api:**](https://github.com/ethz-asl/aerial_mapper/tree/master/aerial_mapper_google_maps_api) Wrapper package for Google Maps API.
- [**aerial_mapper_grid_map:**](https://github.com/ethz-asl/aerial_mapper/tree/master/aerial_mapper_grid_map) Wrapper package for grid_map.
- [**aerial_mapper_io:**](https://github.com/ethz-asl/aerial_mapper/tree/master/aerial_mapper_io) Input/Output handler that reads/writes poses, intrinsics, point clouds, GeoTiffs etc.
- [**aerial_mapper_ortho:**](https://github.com/ethz-asl/aerial_mapper/tree/master/aerial_mapper_ortho) Different methods for (ortho-)mosaic generation.
- [**aerial_mapper_thirdparty:**](https://github.com/ethz-asl/aerial_mapper/tree/master/aerial_mapper_thirdparty) Package containing thirdparty code.
- [**aerial_mapper_utils:**](https://github.com/ethz-asl/aerial_mapper/tree/master/aerial_mapper_utils) Package for common utility functions.

### Getting started
- [How to install](https://github.com/ethz-asl/aerial_mapper/wiki/How-to-install)
- [How to run (+ sample dataset)](https://github.com/ethz-asl/aerial_mapper/wiki/How-to-run)
- [How to contribute](https://github.com/ethz-asl/aerial_mapper/wiki/How-to-contribute)

### Output samples
Dense point cloud <br> (from virtual stereo pair, 2 images) | Digital Surface Map <br> (DSM, exported as GeoTiff) | (Ortho-)Mosaic <br> (from homography, 249 images) |
:---: | :---: | :---: |
![](https://github.com/ethz-asl/aerial_mapper/wiki/dense_preview.png) | ![](https://github.com/ethz-asl/aerial_mapper/wiki/dsm_preview.png)| ![](https://github.com/ethz-asl/aerial_mapper/wiki/ortho_from_homogr.gif) 

Raw images | Dense point cloud | Digital Surface Map |
:---: | :---: | :---: | 
![](https://github.com/ethz-asl/aerial_mapper/wiki/raw_images.gif) | ![](https://github.com/ethz-asl/aerial_mapper/wiki/dense.gif)  | ![](https://github.com/ethz-asl/aerial_mapper/wiki/dsm.png) 
__Observation Angle <br> (red: nadir)__ | __Grid-based Orthomosaic <br> (Cell resolution: 0.5m)__ | __Textured DSM__ 
![](https://github.com/ethz-asl/aerial_mapper/wiki/observation_angle.gif) | ![](https://github.com/ethz-asl/aerial_mapper/wiki/ortho.gif) | ![](https://github.com/ethz-asl/aerial_mapper/wiki/ortho_elevated.gif) 

<hr>

### Publications
If you use this work in an academic context, please cite the following publication:

T. Hinzmann, J. L. Schönberger, M. Pollefeys, and R. Siegwart, **"Mapping on the Fly: Real-time 3D Dense Reconstruction, Digital Surface Map and Incremental Orthomosaic Generation for Unmanned Aerial Vehicles"** [[PDF]](http://www.timohinzmann.com/publications/fsr_2017_hinzmann.pdf)

```
@INPROCEEDINGS{fsr_hinzmann_2017,
   Author = {T. Hinzmann, J. L. Schönberger, M. Pollefeys, and R. Siegwart},
   Title = {Mapping on the Fly: Real-time 3D Dense Reconstruction, Digital Surface Map and Incremental Orthomosaic Generation for Unmanned Aerial Vehicles},
   Booktitle = {Field and Service Robotics - Results of the 11th International Conference},
   Year = {2017}
}
```
### Acknowledgment
This work was partially funded by the European FP7 project SHERPA (FP7-600958) and the Federal office armasuisse Science and Technology under project number 050-45. Furthermore, the authors wish to thank [Lucas P. Teixeira](https://github.com/weblucas) from the [Vision for Robotics Lab](http://www.iris.ethz.ch/the-institute/vision-for-robotics-lab.html) at ETH Zurich for sharing scripts that bridge the gap between Blender and Gazebo.


