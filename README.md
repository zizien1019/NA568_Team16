# EECS568 Team16, Winter 2024

### Repository Information
* [android_app_team16](https://github.com/zizien1019/NA568_Team16/tree/main/android_app_team16) containing the the project code for building Android app using Java. The back-end codes are implemented [here](https://github.com/zizien1019/NA568_Team16/tree/main/android_app_team16/app/src/main/java/com/mbsbahru/na568Teamproject_MohammedAlanUsmanBahru). 
* [Data_visualization](https://github.com/zizien1019/NA568_Team16/tree/main/Data_visualization) containing the code and data for examining the testing.
* [python_code](https://github.com/zizien1019/NA568_Team16/tree/main/python_code) containing the code on implementing Kalman and Particle Filter.
* [figures](https://github.com/zizien1019/NA568_Team16/tree/main/figures) containing the figures for documentation purpose.
 
## Pose Perfect - An Android Pose Estimation & Guidance App

This project implements an Android application for pose estimation in 6-DoF ($x$, $y$, $z$, $\phi$, $\theta$, $\psi$) of the camera coordinate toward the object frame, designed to guide users towards capturing images from a specific angle.

### Features

* Uses computer vision Perspective-n-Points (PnP) techniques to etimates the positions of the camera toward the object frame.
* Provides real-time feedback on user positioning for achieving the desired pose. It uses simple instructions: forward/backward right/left, up/down, and clockwise/counterclockwise

### Installation
* To develop and install the application via Android Studio please [read through this](android_app_team16/README.md).
* To install the app more instantly you follow through this step:
  - Download through your Android device, the generated '.apk' file [here](https://drive.google.com/file/d/1H8T5yAWxWS_5-SAM8eWqe3GmNhLwfa4T/view?usp=sharing).
  - Install the package by the Android built-in package installer or by using third-party APK installer from the Google Play.
  - Grant Permission for the application to access the camera by go through:\
    **Settings &rarr; App &rarr; 'Team16_NA568' (app name) &rarr; Permissions &rarr; Camera &rarr; Allow**
  - Open the app.



### Application Interaction and Usage
The app has three menus:
 - *HSV Object Segmentation*
 - *Reference Input*
 - *Pose Perfect Action*

1. **HSV Object Segmentation**\
In this instant the user can first segment the wanted color by **touching** the object in the display. If further adjustment is needed, there are 5 buttons provided with its corresponding sliders:
    1. **H**: Adjust ***Hue*** thresholding. The *Top* slider for the *Minimum* threshold and the *Bottom* slider for the *Maximum* threshold adjuster.
    2. **S**: Adjust ***Saturation*** thresholding. The *Top* slider for the *Minimum* threshold and the *Bottom* slider for the *Maximum* threshold adjuster.
    3. **V**: Adjust ***Value*** thresholding. The *Top* slider for the *Minimum* threshold and the *Bottom* slider for the *Maximum* threshold adjuster.
    4. **ED**: Adjust morphological operator ***Erode*** and ***Dilate*** mask size. The *Top* slider is for the *Erode* size and the *Bottom* for the *Dilate* size. This button also serve as ***toggle*** to display either the ***RGB*** or ***Binary*** color space. It can be used for removing unwanted contour.
    5. **MG**: Adjust the blurring operator ***MedianBlur*** and ***GaussianBlur*** mask size. The *Top* slider is for the *Median* size and the *Bottom* for the *Gaussian* size. This button also serve as ***toggle*** to display either the ***RGB*** or ***Binary*** color space. It can be used for smoothing out or removing unwanted contour.
   
>> If you prefer not to use the "touch" feature, you can adjust the segmentation just based on the HSV threshold. For doing this, first make all the HSV sliders ranged at their maximum (top slider to the most left and bottom slider to the most right). Then, adjust the minimum and maximum slider sequentially from **H** &rarr; **S** &rarr; **V**.

![HSV Object Segmentation](https://github.com/zizien1019/NA568_Team16/assets/113637971/11ba64ab-c299-4b62-961f-ab09b91a8cc4)


2. **Reference Input**\
This is used to update the object dimensions, and manually input the desired reference image using the states (euclidean distance $r$, $\phi$, $\theta$, and $\psi$). The default object size is 34.5 cm wide by 14.5 cm tall. Please change this according to your object.

![Reference Input](https://github.com/zizien1019/NA568_Team16/assets/113637971/2da23e17-7d1d-43de-9681-cebac0307cc1)


3. **Pose Perfect Action**\
This part of the app guides the user on how to achieve the correct pose. It gives the following instructions: **forward** / **back**, **up** / **down**, **right** / **left**, and **clockwise** / **counterclockwise**.

![Pose Perfect Action1](https://github.com/zizien1019/NA568_Team16/assets/113637971/9b9f44c5-65a4-45b0-a7e1-a3ac6d25ded0)


The user can automatically selsect the new desired pose (reference pose) by clicking this button

![Pose Perfect Action2](https://github.com/zizien1019/NA568_Team16/assets/113637971/d05ad614-f165-4de1-b05b-6d332cc06893)

The user can reset the desired or reference pose by clicking this button

![Pose Perfect Action3](https://github.com/zizien1019/NA568_Team16/assets/113637971/0c3dc39a-9ad3-49a8-84f9-744aa0af1ff7)


### Disclaimer
For correct pose estimation, the object dimension (which can be adjusted in the app) and the camera intrinsics should be defined. In this project, we use our calibrated Google Pixel 7 which has camera intrinsic parameters as:

$$
K_{android\_gp7} = \begin{bmatrix}
    f_x & s & c_x\\
    0 & f_y & c_y\\
    0 & 0 & 1
\end{bmatrix}
= \begin{bmatrix}
    2998 & 8 & 1148\\
    0 & 2971 & 2049\\
    0 & 0 & 1
\end{bmatrix} \\
$$

$$
k_{android\_gp7} = \begin{bmatrix}
    k_1\\
    k_2\\
    p_1\\
    p_2\\
    k_3
\end{bmatrix}
= \begin{bmatrix}
    0.2585\\
    -1.6159\\
    -0.0051\\
    -0.0008\\
    4.2888
\end{bmatrix}
$$

If you have your intrinsic camera parameters and want to build the app via Android Studio, you can change those parameters in [these lines](https://github.com/zizien1019/NA568_Team16/blob/main/android_app_team16/app/src/main/java/com/mbsbahru/na568Teamproject_MohammedAlanUsmanBahru/MainActivity.java#L113C5-L122C96).


* **Authors:** Muhammad Bahru Sholahuddin, Mohammed Buhlaigah, Zih-En Tseng, Usman Shahzad


