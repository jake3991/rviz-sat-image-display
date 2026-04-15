# 🛰️ Satellite Image RViz Alignment Tool (ROS 2)

All images must come from the Mapbox Static Images API. This ensures consistent Web Mercator projection, controlled zoom scaling, and reproducible geospatial alignment. Use the Mapbox sandbox here: https://docs.mapbox.com/playground/static/

Static API format:
https://api.mapbox.com/styles/v1/mapbox/satellite-v9/static/<lon>,<lat>,<zoom>/<width>x<height>?access_token=YOUR_TOKEN

Requirements: must use satellite-v9 style, image must be square (e.g. 1024×1024), and you must record latitude, longitude, and zoom level. These values are used to compute meters-per-pixel scaling.

---

## Dependencies

ROS 2 packages: rclpy, sensor_msgs, geometry_msgs, tf2_ros, visualization_msgs, and ros-humble-interactive-markers (or your distro equivalent). Install with:

sudo apt install ros-humble-interactive-markers

Python dependencies: numpy, opencv-python, scipy, gtsam.

---

## Configuration

config.yaml contains image filename, latitude, longitude, and zoom. transform.yaml stores alignment state including x, y, z, yaw, and flip values used for TF initialization and interactive updates.

---

## Running the system

Run with: python3 view_image.py

---

## RViz Setup

Launch RViz2 and set Fixed Frame to map. Add displays for TF, PointCloud2 (/sat_image_cloud), and InteractiveMarkers (/sat_marker/update).

---

## Interactive Marker Control

The system provides an RViz interactive marker for real-time alignment. You can drag the satellite image in the XY plane, rotate around yaw, and optionally adjust position depending on configuration. This directly updates the TF transform (map → sat_frame) and the point cloud alignment.

---

## System Model

Mapbox Image → OpenCV → PointCloud2 → RViz → Interactive Marker → TF Transform

---

## Controls

Translation is done by dragging in the plane (X/Y movement). Rotation adjusts yaw. Optional Z-axis movement can be enabled via additional controls.

---

## Notes

The point cloud is generated at Z = 0 plane. Scaling is derived from latitude and zoom level using Web Mercator approximation. Large images may be slow due to dense point cloud generation.

---

## Limitations

This is a manual alignment tool, not an automatic optimization system. RViz is required for interaction. High resolution images may be computationally expensive. The system assumes a local planar approximation of the Earth.

---

## Recommended Improvements

Downsample images before cloud generation, add GPU acceleration for raster to point cloud conversion, integrate ICP-based automatic alignment, add GTSAM optimization loop, and support multi-resolution Mapbox tiles.

---

## Future Work

Snap-to-LiDAR-ground mode, automatic yaw initialization, error metric visualization in RViz, and multi-frame georeferenced alignment system.

---

## Key Idea

This tool treats satellite imagery as a manipulable 3D object in ROS space, enabling intuitive geospatial alignment using RViz and TF rather than manual coordinate tuning.