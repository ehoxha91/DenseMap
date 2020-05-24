# DenseMap
Dense mapping using trajectory files from ORB_SLAM2.

### Clone the repository:

-`git clone https://github.com/ehoxha91/DenseMap`


### Build
-`cd DenseMap`

-`mkdir build && cd build`

-`cmake ..`

-`make`

### Run

-`cd ~/bin`

-`./dense_mapping dataset_folder trajectory_file.txt max_frames denseMapp.pcd `

### Dataset

As input this program takes trajectory.txt file and dataset folder which contains `rgb` and `depth` folders inside.
Images IDs should be on the trajectory file --> id, tx, ty, tz, qx, qy, qz, qw

-`Import trajectory.txt to /bin.` 

-`Import dataset to /bin.`

Output of this repository is a point cloud map. To convert this .pcd file to an octomap you can use this repository:

-`https://github.com/ehoxha91/pcl2octomap`
