# aerial_mapper_ortho

(Ortho-)Mosaic Generation

- **Input:** Raw images, camera intrinsics, camera poses
- **Output:** (Ortho-)Mosaic as ros message (e.g. for grid_map) or GeoTiff

Available methods:
- Homography-based mosaic (forward projection):
  - (+) The complete field of view (FOV) of the images can be used.
  - (-) Planar assumption requires high "flight altitude to terrain elevation" ratio.
  - (-) Generates a mosaic but not a true orthomosaic.
- Point cloud-based orthomosaic (forward projection)
  - (+) No assumption on flight altitude/terrain elevation.
  - (-) Holes in dense point cloud need to be interpolated from nearby points.
  - (-) May result in reduced FOV (depending on the dense reconstruction).
- Grid-based orthomosaic (backward projection)
  - (+) No assumption on flight altitude/terrain elevation.
  - (+) Able to generate true orthomosaic.
  - (+) Full coverage/full usage of FOV.
  - (-) Backward projection is computationally more expensive.

