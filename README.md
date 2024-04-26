### 0. Setting
```
git clone https://github.com/dongjineee/plane_graph_slam.git
cd your_workspaces/plane_graph_slam
catkin_make
```

### 0. Setting
```
git clone https://github.com/dongjineee/plane_graph_slam.git
cd your_workspaces/plane_graph_slam
catkin_make
```

### 1. environment using Gazebo
```
<p align = "center">
<img src="https://github.com/dongjineee/plane_graph_slam/assets/150753899/41c6cba6-1a39-47df-920f-ac39f5558641" width="700" height="400"/>
</p>
```

## Project Tree

```
plane_graph_slam
├─ .gitignore
├─ CMakeLists.txt
├─ README.md
├─ include
│  └─ qp_slam
│     ├─ backend.h
│     ├─ keyframe.h
│     └─ tracker_cloud.h
├─ msg
│  ├─ key_m.msg
│  ├─ plane.msg
│  ├─ planes.msg
│  └─ pose.msg
├─ package.xml
└─ src
   ├─ backend.cpp
   ├─ front_plane.cpp
   └─ keyframe.cpp
```


## Installation

1. Clone the repository:

    ```bash
    git clone https://github.com/your_username/your_repository.git
    ```

2. Install dependencies:

    ```bash
    pip install -r requirements.txt
    ```

## Example

To run the system on sample data included in this repository:

```bash
python main.py --input sample_data --output results

