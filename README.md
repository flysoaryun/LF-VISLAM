### LF-VISLAM: A SLAM Framework for Large Field-of-View Cameras with Negative Imaging Plane on Mobile Agents [[PDF]](https://arxiv.org/pdf/2209.05167.pdf), IEEE Transactions on Automation Science and Engineering (T-ASE)
Ze Wang, [Kailun Yang](https://yangkailun.com/), Hao Shi, [Peng Li](https://person.zju.edu.cn/en/lipeng), [Fei Gao](http://zju-fast.com/fei-gao/), [Jian Bai](https://person.zju.edu.cn/en/baijian), [Kaiwei Wang](http://wangkaiwei.org/indexeg.html).

## Wiki 
The VIO system [**LF-VIO**](https://github.com/flysoaryun/LF-VIO#readme) and [**LF-VIO Wiki**](https://github.com/flysoaryun/LF-VIO/wiki) is **constantly being updated......**

The Loop closure of LF-VISLAM will also be updated later.

If you find this work useful or interesting, please kindly give us a star :star:, **thanks!**:grinning:

## How to run LF-VISLAM
1、Build LF-VIO on ROS
Clone the repository and catkin_make:
```
    cd ~/catkin_ws/src
    git clone https://github.com/flysoaryun/LF-VISLAM.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```
---

2、Run
```
    roslaunch vins_estimator mindvision.launch
```

## Ourdoor test




https://github.com/flysoaryun/LF-VISLAM/assets/36565779/e35c2a3f-d5d4-42be-9f72-a4fb0d046107





**Green:** LF-VIO

**Blue:** LF-VISLAM

**Red:** Groundtruth


## Download PALVIO Dataset

![picture](https://user-images.githubusercontent.com/36565779/219860393-943833dd-c83d-4ea8-b23c-7bf0fd171a47.png)

  ID01, ID06, ID10: [**Google Drive**](https://drive.google.com/drive/folders/1RdnUtMulDuhWBfAgq_CLp18EgDvTrZ89?usp=sharing)
  
  ID01~ID10: [**Baidu Yun**](https://pan.baidu.com/s/1o6TgcDwfcDIFl6n9dzsysA), Code: d7wq 
  
  IDL01~IDL02: [**Baidu Yun**](https://pan.baidu.com/s/1FZZkPCdR3odSatULG772og), Code: khw2 

## Dataset parameters
ID01~ID10: [**LF-VIO**](https://github.com/flysoaryun/LF-VIO)

IDL01~IDL02: 

Pal_camera:
```
Fov: 360°x(40°~120°)

Resolution ratio: 1280x720

Lens: Designed by Hangzhou HuanJun Technology.

Sensor: mynteye module.

Frequency: 30Hz
```

Pal_camera:
```
model_type: scaramuzza
camera_name: pal
image_width: 1280
image_height: 720
poly_parameters:
   p0: -2.463825e+02
   p1: 0.000000e+00
   p2: 2.001880e-03
   p3: -3.074460e-06
   p4: 6.138370e-09
inv_poly_parameters:
   p0: 378.706255
   p1: 248.013259
   p2: 9.758682
   p3: 14.691543
   p4: 22.412646
   p5: 8.404425
   p6: -1.668530
   p7: 3.279720
   p8: 2.380985
   p9: -0.599038
   p10: 0.525474
   p11: 1.018587
   p12: 0.289189
   p13: 0.0
   p14: 0.0
   p15: 0.0
   p16: 0.0
   p17: 0.0
   p18: 0.0 
   p19: 0.0 
affine_parameters:
   ac: 0.999993
   ad: -0.000002
   ae: 0.000025
   cx: 605.933287
   cy: 362.393961
```


IMU(mynteye module):
```
Frequency: 200Hz
acc_n: 0.02          # accelerometer measurement noise standard deviation.
gyr_n: 0.04         # gyroscope measurement noise standard deviation.    
acc_w: 0.04         # accelerometer bias random work noise standard deviation.  
gyr_w: 0.002      # gyroscope bias random work noise standard deviation.    
```

The extrinsic parameter between IMU and pal Camera
```
extrinsicRotation: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [-0.953122, 0.172069, 0.248899,
           0.134667, 0.977837, -0.160311,
           -0.270968, -0.119278, -0.95517]
extrinsicTranslation: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [-0.0324005, -0.00943501, 0.039072]
```

## Publication
If you find this work useful, please consider referencing the following paper:

**LF-VISLAM: A SLAM Framework for Large Field-of-View Cameras With Negative Imaging Plane on Mobile Agents**

Z. Wang, K. Yang, H. Shi, P. Li, F. Gao, J. Bai, K. Wang.

```
@article{LF-VISLAM,
  title={LF-VISLAM: A SLAM Framework for Large Field-of-View Cameras With Negative Imaging Plane on Mobile Agents},
  author={Wang, Ze and Yang, Kailun and Shi, Hao and Li, Peng and Gao, Fei and Bai, Jian and Wang, Kaiwei},
  journal={IEEE Transactions on Automation Science and Engineering},
  year={2023},
  publisher={IEEE}
}
```
