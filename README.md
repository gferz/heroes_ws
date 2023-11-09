# HEROES ROS Workspace

### Installation
1. [Install ros noetic](https://wiki.ros.org/noetic/Installation/Ubuntu).
1. Install some additional ros packages. 
    ```
    sudo apt install ros-noetic-teleop-joy
    sudo apt install ros-noetic-rosserial
    ```
1. Create python virtual environtment for yolov8.
    ```
    cd
    python3 -m venv ultralytics
    source ultralytics/bin/activate
    pip install ultralytics
    pip install onnxruntime
    pip install onnx
    pip install rospkg
    deactivate
    ```
1. Create directory for the yolo model
    ```
    mkdir model
    ```
    Place your yolov8 model inside the model directory.
1. Clone the repository.
    ```
    git clone https://github.com/gferz/heroes_ws
    ```
1. Build the ros packages.
    ```
    cd heroes_ws
    catkin_make
    ```
