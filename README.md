# Vision-Based Tactile Information Extraction and Localization for Dexterous Grasping

## Introduction
This repository is dedicated to the research project "Vision-Based Tactile Information Extraction and Localization for Dexterous Grasping," focusing on leveraging vision-based systems to infer tactile properties for advanced robotic manipulation tasks.

## Repository Structure
- `0.Texture_Extraction_and_Analysis`: Scripts and documentation for texture feature extraction and analysis.
- `1.RHBSetup`: Setup files for the robot hand base.
- `2.Initial_pcd`: Initial point cloud data collected from the RealSense camera.
- `3.Filtered_pcd`: Processed point cloud data after filtering.
- `4.Texture_Regions_Examples`: Examples of identified texture regions in the point cloud.
- `5.Images_Examples`: Sample images showing the results of texture analysis.
- `pcd_1 for each object`: Point cloud data for individual objects, first set.
- `pcd_2 for each object`: Point cloud data for individual objects, second set.
- `Parameter Values and Correspondences`: Documentation for parameter values and their correspondences used in analysis.
- `initial_example.png`: An initial example image showing the point cloud data visualization.
- `LICENSE`: The license file for the project.

## Scripts Description
- `get_pointcloud.py`: Captures point cloud data from a RealSense camera and processes it into a usable format.
- `texture_analysis.py`: Analyzes texture features within the point cloud data and separates regions based on texture.
- `get_pointcloud2.py`, `iros_texture_analysis.py`, `openpcd.py`: Additional scripts for point cloud data handling and analysis.

## Dependencies
To run the scripts in this repository, you need the following dependencies installed:
- Python 3.x
- NumPy
- Open3D
- Pyrealsense2
- SciPy

## Installation and Usage
Please refer to the `0.Texture_Extraction_and_Analysis` directory for detailed instructions on setting up the environment and running the scripts for texture extraction and analysis.

## Contributing
Contributions to this project are welcome. Please refer to the contributing guidelines for more information.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
