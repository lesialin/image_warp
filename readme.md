# Image Warp

This repo is implementation image warp in c function, there are floating and fixed function for warping image. 

I reference the python function in  https://github.com/eborboihuc/rotate_3d . And you may refine the  cal_trans_matrix input in main.cpp to get the warping matrix.

```C++
// the input rotation order, roll, pitch, yaw in degree
cal_trans_matrix(0.0, 0.0,10, image_width, image_height, trans_matrix);
```

**usage**

```shell
mkdir build
cd build
cmake ..
cmake --build .
```

and there will be prebuilt/image_warp.exe

**run**

```shell
./image_warp.exe ../lean.png
```

**result**

```shell
--------------------------
Image warp in float operation:
Time measured: 0.014680 seconds.
--------------------------
Image warp in fixed-point operation:
Time measured: 0.008530 seconds.
```

rotate yaw 10 degree

![test_fixed](image/readme/test_fixed.png)