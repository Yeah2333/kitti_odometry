# KITTI Odometry Benchmark
KITTI Odometry result for GICP, ICP-P2P, NDT, ICP-P2Pl

## Required
pcl

boost


## Run
```
	cout << "Odometry method: ";
    if(flag == 0){
        cout<< "icp_nolinear"<< endl;
    }
    else if(flag == 1){
        cout<< "icp_p2p"<< endl;
    }
    else if (flag == 2){
        cout<< "gicp"<< endl;
    }
    else if (flag == 3){
        cout<< "icp_p2pl"<< endl;
    }
    else if (flag == 4){
        cout<< "ndt"<< endl;
    }
```

example
```
./kitti_odometry 1 /home/wangzhiyong/DataDriver/kitti/ /home/wangzhiyong/DataDriver/kitti_result/ 
```