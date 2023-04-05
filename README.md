# Point Cloud Visualization and Cropping

This Python script allows you to visualize a 3D point cloud, interactively crop it using a bounding box, and compute the convex hull and its volume. The script uses Open3D and OpenCV libraries for processing and visualization.

## Dependencies

- Python 3.6+
- Open3D
- OpenCV
- NumPy

You can install these dependencies using the following command:

```bash
pip install open3d opencv-python numpy
```


## Usage

To run the script, use the following command:

```bash
python pcd_volume.py <path_to_point_cloud>
```

## How it works
- The script reads the point cloud from the provided file.
- A visualization window is created to display the point cloud and a separate window to control the bounding box size.
- You can adjust the bounding box size using trackbars in the "Bounding Box Size" window.
- Press 'q' to finish the cropping process.
- The script will then compute the convex hull of the cropped point cloud and display its volume.
- Finally, the cropped point cloud, the convex hull, and the bounding box will be displayed in a new visualization window.
