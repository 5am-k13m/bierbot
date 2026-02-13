# [WIP] Beverage Issuing and Extraction Robot (BIERBot)
### This project seeks to develop a personal home beverage attendant, able to navigate home environments, deliver beverages to residents, and deposit empty beverage receptacles in recycling bins.
#### Created by Sam Klemic

Further project goals include utilizing low-cost hardware, developing a dynamic computer vision platform adaptable to any room, and incorporating more advanced computer vision techniques in the future (i.e. Object vs Person classification, Beverage Receptacle location estimation, etc.).

## Room Sensing

3D point cloud images of a given room is captured using two Xbox 360 Kinects. Kinects are interfaced with using [PyKinect](https://github.com/microsoft/ptvs/wiki/PyKinect). Infrared depth images from individual Kinects provide distance data for each pixel in frame relative to the sensor. Using the Kinect camera's intrinsic matrix, we transform these distance pixels to point clouds, with the Kinect positioned as the origin.

Depth Heatmap from Cameras 1 and 2:

| Depth Heatmap | 3D Point Cloud |
| ----------- | ----------- |
| ![Camera 1 Depth heatmap](src\example_viz\cam_1_depth_heatmap.png) | ![Camera 1 Point Cloud](src\example_viz\cam_1_pointcloud.gif) |
| ![Camera 2 Depth heatmap](src\example_viz\cam_2_depth_heatmap.png) | ![Camera 2 Point Cloud](src\example_viz\cam_2_pointcloud.gif) |

We calibrate the coordinate spaces of the two camera point clouds by identifying three non-colinear corresponding points within the room. Using the Kabsch-Umeyama algorithm, we then compute the rigid transformation, in the form of a rotation matrix and a translation vector. This transformation is applied to one of the point clouds to align it with the other, enabling unified 3D sensing of the space.

![Camera coordinate space alignment](src\example_viz\sequenced_calib.gif)

By selecting the three calibration points to all lie on the floor of the room, we can also identify a plane within the space that corresponds to the floor. With this floor plane determined, we can easily idenify which points correspond with obstacles. The strategy implemented makes this discrimination by checking if the orthogonal distance between a given point and the floor plane falls within a user-determined threshold.

## Obstacle Detection

Idenfiying the points that are within a given threshold from the floor is our first step in identifying the navigable space in the room. In practice, the Kinect IR Sensor can be noisy, leading to inaccuracy in depth data. In order to discriminate between noise and an actual obstacle, we employ a cluster detection algorithm, namely DBSCAN. DBSCAN was selected because it is lightweight enough to run in real time and can accurately differentiable between points belonging to obstacles and noise from the Kinect sensor.

![Floor and cluster detection](src\example_viz\top_view_clusters.gif)

With these two techniques employed we can identify obstacles within the room, and more importantly, identify where in the room our robot can actually navigate.