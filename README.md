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

