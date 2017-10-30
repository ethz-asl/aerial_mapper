# aerial_mapper_ortho

(Ortho-)Mosaic Generation

- **Input:** Raw images, camera intrinsics, camera poses
- **Output:** (Ortho-)Mosaic as ros message (e.g. for grid_map) or GeoTiff

Available methods:
- Homography-based mosaic (forward projection)
- Grid-based orthomosaic (backward projection)
- Point cloud-based orthomosaic (forward projection)
