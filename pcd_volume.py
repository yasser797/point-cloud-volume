import sys
import numpy as np
import open3d as o3d
import cv2

if len(sys.argv) != 2:
    print("Usage: python point_cloud_visualization.py <path_to_point_cloud>")
    exit()

point_cloud_path = sys.argv[1]

pcd = o3d.io.read_point_cloud(point_cloud_path)

pcd_center = pcd.get_center()

bbox = pcd.get_axis_aligned_bounding_box()
bbox_center = bbox.get_center()

translation = bbox_center - pcd_center
pcd.translate(translation, relative=False)


def update_bbox_size(*args):
    global bbox, points, colors

    min_corner = pcd_center - np.array([cv2.getTrackbarPos("X Min", "Bounding Box Size"),
                                        cv2.getTrackbarPos("Y Min", "Bounding Box Size"),
                                        cv2.getTrackbarPos("Z Min", "Bounding Box Size")]) * 0.01

    max_corner = pcd_center + np.array([cv2.getTrackbarPos("X Max", "Bounding Box Size"),
                                        cv2.getTrackbarPos("Y Max", "Bounding Box Size"),
                                        cv2.getTrackbarPos("Z Max", "Bounding Box Size")]) * 0.01

    if np.all(min_corner < max_corner):
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_corner, max_corner)
        bbox.color = (1, 0, 0)

        cropped_pcd = pcd.crop(bbox)
        points = cropped_pcd.points
        colors = cropped_pcd.colors

        vis.clear_geometries()

        new_pcd = o3d.cpu.pybind.geometry.PointCloud()

        new_pcd.points = o3d.utility.Vector3dVector(points)
        new_pcd.colors = o3d.utility.Vector3dVector(colors)

        vis.add_geometry(new_pcd)
        vis.add_geometry(bbox)


# Create visualization window
vis = o3d.visualization.Visualizer()
vis.create_window(width=640, height=480)

# Add point cloud to visualization window
vis.add_geometry(pcd)

WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 500

cv2.namedWindow("Bounding Box Size", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Bounding Box Size", WINDOW_WIDTH, WINDOW_HEIGHT)

cv2.createTrackbar("X Min", "Bounding Box Size", 0, 200, update_bbox_size)
cv2.createTrackbar("X Max", "Bounding Box Size", 100, 200, update_bbox_size)

cv2.createTrackbar("Y Min", "Bounding Box Size", 0, 200, update_bbox_size)
cv2.createTrackbar("Y Max", "Bounding Box Size", 100, 200, update_bbox_size)

cv2.createTrackbar("Z Min", "Bounding Box Size", 0, 200, update_bbox_size)
cv2.createTrackbar("Z Max", "Bounding Box Size", 100, 200, update_bbox_size)

update_bbox_size()

# Run visualization loop
while True:
    vis.poll_events()
    vis.update_renderer()
    cv2.imshow("Bounding Box Size", np.zeros((1, 1)))
    key = cv2.waitKey(1)
    if key == ord("q"):
        cropped_pcd_points = points
        break

# Close visualization window
vis.destroy_window()
cv2.destroyAllWindows()

cropped_pcd = o3d.geometry.PointCloud(cropped_pcd_points)
cropped_pcd.colors = o3d.utility.Vector3dVector(colors)

# Get volume of the cropped cloud using convex hull
hull, _ = cropped_pcd.compute_convex_hull()
hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
hull_ls.paint_uniform_color((1, 0, 0))

cropped_bbox = cropped_pcd.get_axis_aligned_bounding_box()
cropped_bbox.color = (1, 0, 0)

volume = hull.get_volume()
print(hull.get_volume())

o3d.visualization.draw_geometries([cropped_pcd, hull_ls, cropped_bbox])