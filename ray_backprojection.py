import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def visualize_ray(origin, direction, length):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    t = np.linspace(0, length, 100)
    ray_points = origin[:, np.newaxis] + direction[:, np.newaxis] * t
    
    ax.plot(ray_points[0], ray_points[1], ray_points[2], 'b-')
    ax.scatter(origin[0], origin[1], origin[2], c='r', marker='o')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    plt.show()
    
    
origin = np.array([0, 0, 0])  # 시작점
direction = np.array([1, 1, 1])  # 방향벡터
length = 10  # 광선의 길이

visualize_ray(origin, direction, length)


#####################################
import open3d as o3d
import numpy as np

# Camera parameters
width = 640
height = 480
fx = 525.0
fy = 525.0
cx = 319.5
cy = 239.5

# Image pixel coordinates
u = 320  # Example pixel coordinate
v = 240  # Example pixel coordinate

# Backproject the ray
ray_origin = np.zeros(3)
ray_direction = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
ray_direction /= np.linalg.norm(ray_direction)  # Normalize ray direction

# Create Open3D point cloud with ray origin
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.expand_dims(ray_origin, axis=0))
pcd.colors = o3d.utility.Vector3dVector(np.array([[1.0, 0.0, 0.0]]))  # Assign color to the point based on world coordinates

# Create Open3D line set representing the ray
line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(np.vstack([ray_origin, ray_origin + ray_direction]))
line_set.lines = o3d.utility.Vector2iVector([[0, 1]])
line_set.colors = o3d.utility.Vector3dVector(np.array([[1.0, 0.0, 0.0]]))  # Red color for the ray

# Create Open3D coordinate frame
coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)

# Create Open3D visualizer
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(pcd)
vis.add_geometry(line_set)
vis.add_geometry(coordinate_frame)  # Add coordinate frame to the visualizer

# Run the visualizer
vis.run()
vis.destroy_window()
