# Dynamic Obstacle Avoidance

This docker container will detect the LEDs of other duckiebots

## Prerequisites

You should have already completed the camera and wheel calibrations


## How to use it

### 1. Clone this repository

    $ git clone https://github.com/Janst1000/donald.git

### 2. Get your intrinsics file

Download your camera intrinsics calibration yaml file. We called ours donald_intrinsics.yaml and copy it into
the /src directoy. Please keep in mind that these are name sensitive and you either need to change the name of yours or change the code.


### 3. Build the container on your duckiebot

Exchange donald.local with your duckiebots name

    $ dts devel build -f -H donald.local


### 4. Run the container 

Exchange donald.local with your duckiebots name

    $ dts devel run -H donald.local

### 5. Inspect the results

Open a new terminal and open rqt_image_view and inspect the image topics that are being published

    $ dts start_gui_tools donald.local
    $ rosrun rqt_image_view rqt_image_view

### 6. Tuning (optional)

You can tune the HoughCircles parameters by modifying some rosparameters.
The following parameters are available with it's default values:

    $ circles/param1  25
    $ circles/param2  25
    $ circles/min     0
    $ circles/max     15
    $ circles/blur    5

You can tune them by using:

    $ rosparam set /circles/<parameter> <value>

## How it works

### Preprocessing

We subscribe to the /donald/camera_node/image/compressed image and get the raw pictures from the camera.
We then correct the image using the undistort.rectify() method. This method was taken from the duckietown utils
library but we had to slightly modify it to read the donald_intrinsics.yaml file correctly.

Once we have the corrected image we convert it to grayscale and and darken it a bit.
After that we threshold the image and convert it back to grayscale.

We the use opencv's HoughCircles to get all of the circles in the image
Then the circles are drawn onto the corrected image and can be displayed.