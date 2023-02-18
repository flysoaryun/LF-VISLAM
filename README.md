### LF-SLAM: A SLAM Framework for Large Field-of-View Cameras with Negative Imaging Plane on Mobile Agents 
Ze Wang, [Kailun Yang](https://yangkailun.com/), Hao Shi, [Peng Li](https://person.zju.edu.cn/en/lipeng), [Fei Gao](http://zju-fast.com/fei-gao/), Jian Bai, [Kaiwei Wang](http://wangkaiwei.org/indexeg.html).


## Download PALVIO Dataset
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

**LF-SLAM: A SLAM Framework for Large Field-of-View Cameras with Negative Imaging Plane on Mobile Agents.**
Z. Wang, K. Yang, H. Shi, P. Li, F. Gao, J. Bai, K. Wang.

```

```
