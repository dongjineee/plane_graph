## Features

- **Plane Extraction:** using ransac
- **Graph Representation:** pose & plane graph
- **Graph Optimization:** LM optimize using GTSAM
- **Visualization:** Visualize the resulting trajectory and map.

## Installation

1. Clone the repository:

    ```bash
    git clone https://github.com/your_username/your_repository.git
    ```

2. Install dependencies:

    ```bash
    pip install -r requirements.txt
    ```

## Usage

1. Prepare sensor data in a suitable format (e.g., images or point clouds with plane information).
2. Run the main script:

    ```bash
    python main.py --input <path_to_sensor_data> --output <output_directory>
    ```

3. View the results in the specified output directory.

## Example

To run the system on sample data included in this repository:

```bash
python main.py --input sample_data --output results

