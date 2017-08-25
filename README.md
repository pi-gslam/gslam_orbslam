# GSLAM-ORBSLAM

## 1. Introduction

This code is the [ORB_SLAM](https://github.com/raulmur/ORB_SLAM) plugin implementation base on [GSLAM](https://github.com/pi-gslam/GSLAM).

## 2. Build and Install
### 2.1. Build and Install GSLAM

Please follow: https://github.com/pi-gslam/GSLAM

### 2.2. Build and Install GSLAM-ORBSLAM

```
mkdir build;
cd build;
cmake ..;
make;
sudo make install
```

## 3. Run ORBSLAM with gslam

```
gslam conf=Default.cfg Dataset=your_dataset 
```
