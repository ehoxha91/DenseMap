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

-`./dense_mapping dataset_folder trajectory_file.txt max_frames distance_threshold denseMapp.pcd `

Where max_frames is the number of frames to be used on this map, and distance_threshold is up to what depth for each frame.

### Dataset

As input this program takes trajectory.txt file and dataset folder which contains `rgb` and `depth` folders inside.
Trajectory file line format: `id, tx, ty, tz, qx, qy, qz, qw` where id is the id of the frame. Program uses this id to read rgb (r+id.png) and depth (d+id.png) images.

-`Import trajectory.txt to /bin.` 

-`Import dataset to /bin.`

Output of this repository is a point cloud map. To convert this .pcd file to an octomap you can use this repository:

-`https://github.com/ehoxha91/pcl2octomap`
