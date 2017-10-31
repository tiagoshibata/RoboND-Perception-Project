## Project: Perception Pick & Place

---

# Steps:
1. Extract features and train an SVM model on new objects.

Features were extracted and three models were trained. The `capture_features.py` script was changed to receive an argument that sets the list of labels to be used. The lists were taken from `pick_list_*.yaml` in `/pr2_robot/config/`:

```diff
diff --git a/Exercise-3/sensor_stick/scripts/capture_features.py b/Exercise-3/sensor_stick/scripts/capture_features.py
index a04164b..81ee745 100755
--- a/Exercise-3/sensor_stick/scripts/capture_features.py
+++ b/Exercise-3/sensor_stick/scripts/capture_features.py
@@ -2,6 +2,7 @@
 import numpy as np
 import pickle
 import rospy
+import sys

 from sensor_stick.pcl_helper import *
 from sensor_stick.training_helper import spawn_model
@@ -20,17 +21,54 @@ def get_normals(cloud):
     return get_normals_prox(cloud).cluster


+def get_labels(world_number):
+    if world_number == 0:
+        return [
+            'beer',
+            'bowl',
+            'create',
+            'disk_part',
+            'hammer',
+            'plastic_cup',
+            'soda_can',
+         ]
+    elif world_number == 1:
+        return [
+            'biscuits',
+            'soap',
+            'soap2',
+         ]
+    elif world_number == 2:
+        return [
+            'biscuits',
+            'soap',
+            'book',
+            'soap2',
+            'glue',
+         ]
+    elif world_number == 3:
+        return [
+            'sticky_notes',
+            'book',
+            'snacks',
+            'biscuits',
+            'eraser',
+            'soap2',
+            'soap',
+            'glue',
+         ]
+
+
 if __name__ == '__main__':
     rospy.init_node('capture_node')

-    models = [\
-       'beer',
-       'bowl',
-       'create',
-       'disk_part',
-       'hammer',
-       'plastic_cup',
-       'soda_can']
+    assert(len(sys.argv) <= 2)
+    if len(sys.argv) < 2:
+        world_number = 0
+    else:
+        world_number = int(sys.argv[1])
+
+    models = get_labels(world_number)

     # Disable gravity and delete the ground plane
     initial_setup()
```

The training sets were generated just like in exercise X.

2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

# Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---
### Writeup / README

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

Exercise 1 was completed in repository [RoboND-Perception-Exercises](https://github.com/tiagoshibata/RoboND-Perception-Exercises/blob/master/Exercise-1/RANSAC.py). Screenshots:

Original point cloud:

![Original point cloud](Writeup_images/Exercise-1/Original.png)

Voxel grid filtered:

![Voxel grid filtered](Writeup_images/Exercise-1/Downsampled.png)

Pass through filtered:

![Pass through filtered](Writeup_images/Exercise-1/Pass-through.png)

RANSAC plane fitting inliers and outliers:

![RANSAC inliers](Writeup_images/Exercise-1/RANSAC-inliers.png)
![RANSAC outliers](Writeup_images/Exercise-1/RANSAC-outliers.png)

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.

#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.
Here is an example of how to include an image in your writeup.

![demo-1](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.
