# DenseMap
Dense mapping using trajectory files from ORB_SLAM2.

### 
-`Import trajectory.txt to /bin.`
-`Import dataset to /bin.`

### To build
-`mkdir build && cd build`

-`cmake ..`

-`make`

### To run
-`cd ~/bin`

-`./dense_mapping dataset_folder trajectory_file.txt max_frames denseMapp.pcd `
