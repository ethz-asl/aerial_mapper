# aerial-mapper

### Overview
- **aerial_mapper_loader: Load input data**
   - Load from PIX4D
   - Load from COLMAP
   - Subscribe to ROS message
- **aerial_mapper_ortho: (Ortho-)Mosaic Generation**
   - Input: Raw images, camera intrinsics, camera poses 
   - Output: (Ortho-)Mosaic as ros message (grid_map) or GeoTiff
   - Available methods:
      - Homography-based mosaic (forward projection)
      - Grid-based orthomosaic (backward projection)
      - Incremental grid-based orthomosaic (backward projection)
- **aerial_mapper_dsm: Digitial Surface Map/Model Generation**
   - Input: 3D-pointcloud
   - Output: Elevation Map as ros message (grid_map) or GeoTiff

<hr>

### Publications
If you use this work in an academic context, please cite the following publication(s):

T. Hinzmann, J. L. Schönberger, M. Pollefeys, and R. Siegwart, "Mapping on the Fly: Real-time 3D Dense Reconstruction, Digital Surface Map and Incremental Orthomosaic Generation for Unmanned Aerial Vehicles"

```
@INPROCEEDINGS{fsr_hinzmann_2017,
   Author = {T. Hinzmann, J. L. Schönberger, M. Pollefeys, and R. Siegwart},
   Title = {Mapping on the Fly: Real-time 3D Dense Reconstruction, Digital Surface Map and Incremental Orthomosaic Generation for Unmanned Aerial Vehicles},
   Booktitle = {Field and Service Robotics - Results of the 11th International Conference},
   Year = {2017}
}
```
