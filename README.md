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

-`Import trajectory.txt to /bin.`

-`Import dataset to /bin.`

-`./dense_mapping dataset_folder trajectory_file.txt max_frames denseMapp.pcd `

Output of this repository is a point cloud map.
