## Project Tree
```
24-project-base
├─ README.md
└─ src
   ├─ 24-project-localization
   │  ├─ FAST-LOCALIZATION
   │  └─ ...
   ├─ 24-project-mapping
   │  ├─ custom_factor
   │  ├─ plane_segmantation_ransac
   │  └─ ...
   ├─ 24-project-navigation
   │  └─ ...
   ├─ 24-project-robot-ros
   │  ├─ scout_ros
   │  └─ unitree_ros
   └─ 24-project-simulation
      ├─ scout_sim
      └─ unitree_sim


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

