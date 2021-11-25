# POINT CLOUD PROCESSING

[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)





## Realsense Camera
Running Realsense viewer to check avalability of the camera

```realsense-viewer```

Running Realsense with ROS

```roslaunch realsense2_camera rs_camera.launch```


## Open 3D Install from Source.
http://www.open3d.org/docs/release/compilation.html

1.  Using wget to install Open3D App
https://github.com/isl-org/Open3D/releases


```
wget -c https://github.com/isl-org/Open3D/releases/download/v0.13.0/open3d-app-0.13.0-Ubuntu_18.04.deb

sudo apt-get install ./open3d-app-0.13.0-Ubuntu_18.04.deb
```
2. Update Cmake 3.19++ Version ( https://apt.kitware.com/)

For Ubuntu Bionic Beaver (18.04) and newer:
```
sudo apt-get update
sudo apt-get install gpg wget
```

Obtain a copy of our signing key:
```
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null

echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ bionic main' | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null

sudo apt-get update
```

Install the kitware-archive-keyring package to ensure that your keyring stays up to date as we rotate our keys:

```
sudo rm /usr/share/keyrings/kitware-archive-keyring.gpg
sudo apt-get install kitware-archive-keyring
```

Additional step, you can add our release candidate repository to your sources
```
echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ bionic-rc main' | sudo tee -a /etc/apt/sources.list.d/kitware.list >/dev/null

sudo apt-get update
```
Install Cmake
```
sudo apt-get install cmake
cmake --version
```



### Step 1: Installing Open3D from source (http://www.open3d.org/docs/latest/compilation.html)

 - Clonning the Open3D
 
 ```
 git clone --recursive https://github.com/isl-org/Open3D

# You can also update the submodule manually
git submodule update --init --recursive

 ```  

- Install dependencies 
```
util/install_deps_ubuntu.sh
```
- Config

```
mkdir build
cd build
cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=${HOME}/open3d_install ..
cmake --build . --config Release --parallel 12 --target install
```

- Build

```
make -j$(nproc)
```
- Install

 To install Open3D C++ library:
```
make install
```
### Step 2: Use Open3D in this example project

```

git clone https://github.com/intel-isl/open3d-cmake-find-package.git
cd open3d-cmake-find-package
mkdir build
cd build
cmake -DOpen3D_ROOT=${HOME}/open3d_install ..
cmake --build . --config Release
./Draw
```

## Open 3D and ROS perception package



## References
https://github.com/ros-perception/perception_open3d

https://github.com/SeungBack/open3d-ros-helper