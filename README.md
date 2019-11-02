# darknet_det

0. Prerequisites 

        Ubuntu 
        CUDA
        OpenCV
        gstreamer 
        ROS (gscam)

1. Clone or copy darknet_det code into a ROS worksapce. Suppose the directory tree is as below: 

            [ros_ws]/src/darknet_det/ 

    from:   https://github.com/maoxuli/darknet_det

2. Clone or copy darknet code to some directory. For example, inside darknet_det directory, or side by side, as shown below: 

            [ros_ws]/src/darknet_det/darknet/ 

        or  [ros_ws]/src/darknet/ 

        or  in other directory 

    from:   https://github.com/maoxuli/darknet
    
        or  https://github.com/pjreddie/darknet  

3. Build darknet

        cd [ros_ws]/src/darknet_det/darknet     (according to above step)

        edit [ros_ws]/src/darknet_det/darknet/Makefile as below (or according to your situation): 

            GPU=1
            CUDNN=1
            OPENCV=1
            OPENMP=1
            DEBUG=0

        make 

4. Build darknet_det 

        set darknet path in [ros_ws]/src/darknet_det/CMakeLists.txt, according to above steps: 

            set(DARKNET_PATH ${CMAKE_CURRENT_SOURCE_DIR}/darknet)

        cd [ros_ws] 

        catkin_make 

5. Download pre-trained YOLO networks (cfg file, weights file, and names file) to directory shown as below: 

            [ros_ws]/src/darket_det/models/yolov3.cfg 
            [ros_ws]/src/darket_det/models/yolov3.weights 
            [ros_ws]/src/darket_det/models/coco.names 

    from:   https://pjreddie.com/darknet/yolo

        edit darknet detection settings accordingly in [ros_ws]/src/darknet_det/launch/darknet_det.launch, e.g.:

            <arg name="cfg_file" default="$(find darknet_det)/models/yolov3.cfg" />
            <arg name="weights_file" default="$(find darknet_det)/models/yolov3.weights" />
            <arg name="names_file" default="$(find darknet_det)/models/coco.names" />

        set target object classes for detection as below: 

            <rosparam>class_ids: [ 0, 3, 14 ]</rosparam>

        the number is the class ID defined by dataset used for network training, e.g. in COCO dataset, ID 0 is for person. 

6. Test with CSI camera on Jetson Xavier/TX2/Nano or Raspberry PI 

        cd [ros_ws]

        source devel/setup.bash 

        edit camera settings in [ros_ws]/src/darknet_det/launch/csi_cam_det.launch 

        edit darknet detection settings in [ros_ws]/src/darknet_det/launch/darknet_det.launch 

        rolsuanch darknet_det csi_cam_det.launch 

7. Understanding detection result 

darknet_det publish deteciton result on topic [camera_name]/det. Please refer to [ros_ws]/src/darknet_det/msg for defined message type. Please note that the bounding box in current implementation is denoted with center point (x, y) and size (w, h), normalized to image size. 
