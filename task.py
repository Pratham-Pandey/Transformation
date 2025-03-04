import os.path
import cv2
import numpy as np
import cv2.aruco as aruco
import glob
import open3d as o3d
from sklearn.cluster import DBSCAN

class Camera:
    def __init__(self):
        # Camera parameters (intrinsic matrix & distortion coefficients)
        self.camera_matrix = np.array([[617.54, 0, 318.655], [0, 617.5625, 244.10], [0, 0, 1]])
        self.dist_coeffs = np.array([[0, 0, 0, 0, 0]], dtype=np.float32)

        self.squareLength = 0.1      # Length of a single square in the checker board

        # ArUco dictionary & ChArUco board specification for pose estimation
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        self.board = aruco.CharucoBoard_create(squaresX=9, squaresY=9, squareLength=self.squareLength, markerLength=0.075,
                                          dictionary=self.aruco_dict)


    def marker_detection_pose_estimation(self, image_path):
        # Read image and convert to grayscale
        image = cv2.imread(image_path)
        img_cpy1 = image.copy()

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict)

        if ids is not None:
            # Refine marker detection to get ChArUco corners
            _, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(corners, ids, gray, self.board)

            if charuco_corners is not None and len(charuco_corners) > 4:
                # Estimate ChArUco board pose
                success, rvec, tvec = aruco.estimatePoseCharucoBoard(charuco_corners, charuco_ids, self.board, self.camera_matrix,
                                                                     self.dist_coeffs, None, None)


                # For bringing axis to the center
                R, _ = cv2.Rodrigues(rvec)
                board_center_offset = np.array([[4.5 * self.squareLength], [4.5 * self.squareLength], [0]], dtype=np.float32)
                offset_camera_frame = np.dot(R, board_center_offset)
                tvec_adjusted = tvec + offset_camera_frame

                if success:
                    aruco.drawDetectedMarkers(img_cpy1, corners, ids)
                    cv2.imshow("Detected Aruco Markers", img_cpy1)

                    aruco.drawDetectedCornersCharuco(image, charuco_corners, charuco_ids)
                    aruco.drawAxis(image, self.camera_matrix, self.dist_coeffs, rvec, tvec_adjusted, 0.1)
                    cv2.imshow("Detected Checker Corners and Frame", image)

                    cv2.waitKey(0)
                    cv2.destroyAllWindows()

                    return cv2.Rodrigues(rvec.ravel())[0], tvec.ravel()

        print("Unable to Estimate Pose! Skipping.")
        return np.zeros((3, 3)), np.zeros(3)

class Lidar:
    def __init__(self):
        self.percentile_threshold = 99.8            # Threshold for filtering of lidar points
        self.ransac_distance_threshold = 0.05       # Distance to the nearby points for being considered an inlier

    def retroreflective_tape_detection(self, file_path):
        data_fixed = np.load(file_path)
        percentile = np.percentile(data_fixed[:, 6], self.percentile_threshold)

        # To filter points with low intensity
        data_mid = data_fixed[:, 6] > percentile
        filterd_data = data_fixed[data_mid]

        # DBSCAN Filtering
        filterd_data = self.DBSCAN_clustering(filterd_data)

        # Plain Fitting(RANSAC)
        plane_model, filterd_data = self.fit_plane_ransac(filterd_data, self.ransac_distance_threshold)

        # Projecting points to plain
        projected_points = self.project_points_to_plane(filterd_data, plane_model)

        # Converts 3D points to 2D plane coordinates using a local plane basis.
        points_2d, u, v, plane_origin = self.convert_3d_to_2d(projected_points, plane_model)

        # Fit circle
        cx, cy, r = self.fit_circle_least_squares(points_2d)

        # Reconstruct 3D Circle
        circle_3d_points = self.reconstruct_3d_circle(cx, cy, r, plane_origin, u, v)

        # Visualization:
        # Filtered Points
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(filterd_data[:, :3])  # Take only XYZ and not the intensity values
        pcd.paint_uniform_color([0, 0, 1])  # Blue for detected tape

        # Estimated Circle
        circle_pcd = o3d.geometry.PointCloud()
        circle_pcd.points = o3d.utility.Vector3dVector(circle_3d_points)
        circle_pcd.paint_uniform_color([1, 0, 0])  # Red for estimated circle

        o3d.visualization.draw_geometries([pcd, circle_pcd])

        return r, (plane_origin + cx * u + cy * v), plane_model


    def DBSCAN_clustering(self, filterd_data):
        """
        Filters outlier points.
        """
        # Clustering
        xyz_points = filterd_data[:, :3]
        clustering = DBSCAN(eps=3.2, min_samples=40).fit(xyz_points)
        labels = clustering.labels_

        # Filtering Points which belongs to cluster 0. -1 is cluster of noise.
        filterd_data = filterd_data[(labels == 0)]  # Filtering those points having cluster label '0'.

        return filterd_data

    def fit_plane_ransac(self, point_cloud, threshold=0.05):
        """
        Fits a plain to the point cloud.
        """
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(point_cloud[:, :3])

        # Perform RANSAC plane fitting
        plane_model, inliers = pcd.segment_plane(distance_threshold=threshold,
                                                 ransac_n=3,
                                                 num_iterations=1000)

        # Extract inlier points
        inlier_points = point_cloud[inliers]

        return plane_model, inlier_points

    def project_points_to_plane(self, inlier_points, plane_model):
        """
        Projects points on to a plain.
        """
        a, b, c, d = plane_model
        normal = np.array([a, b, c])

        denominator = (a ** 2 + b ** 2 + c ** 2)

        projected_points = []
        for P in inlier_points[:, :3]:
            distance = (np.dot(normal, P) + d) / denominator

            P_proj = P - distance * normal
            projected_points.append(P_proj)

        return np.array(projected_points)

    def convert_3d_to_2d(self, projected_points, plane_model):
        """
        Converts 3D points to 2D plane coordinates using a local plane basis.
        """
        a, b, c, _ = plane_model
        normal = np.array([a, b, c])  # Plane normal vector

        # Step 1: Find two perpendicular basis vectors (u, v) in the plane
        arbitrary_vector = np.array([1, 0, 0]) if abs(a) < abs(c) else np.array([0, 0, 1])
        u = np.cross(normal, arbitrary_vector)  # Compute first basis vector
        u = u / np.linalg.norm(u)  # Normalize it

        v = np.cross(normal, u)  # Compute second basis vector
        v = v / np.linalg.norm(v)  # Normalize it

        # Step 2: Compute centroid as the local origin
        plane_origin = np.mean(projected_points, axis=0)

        # Step 3: Convert each 3D point into 2D (u, v) coordinates
        points_2d = np.array([
            [np.dot(p - plane_origin, u), np.dot(p - plane_origin, v)]
            for p in projected_points
        ])

        return points_2d, u, v, plane_origin

    def fit_circle_least_squares(self, points_2d):
        """
            Fits a circle to given 2D points using the Least Squares method.
        """
        x = points_2d[:, 0]
        y = points_2d[:, 1]

        # Form the design matrix and the right-hand side vector
        A = np.c_[2 * x, 2 * y, np.ones_like(x)]
        b = x ** 2 + y ** 2

        # Solve for (cx, cy, r) using the least squares
        c, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        cx, cy = c[0], c[1]
        r = np.sqrt(c[2] + cx ** 2 + cy ** 2)

        return cx, cy, r

    def reconstruct_3d_circle(self, cx, cy, r, plane_origin, u, v, num_points=100):
        """
        Generates a set of 3D points forming the fitted circle.
        """
        theta = np.linspace(0, 2 * np.pi, num_points)  # Generate angles
        circle_2d = np.array([cx + r * np.cos(theta), cy + r * np.sin(theta)]).T  # Circle in 2D

        # Convert each 2D circle point back to 3D
        circle_3d = np.array([plane_origin + p[0] * u + p[1] * v for p in circle_2d])

        return circle_3d

class Transformation:
    def __init__(self, rmat, tvec, circle_center, plane_model):

        # Charuco board pose in camera frame(from Problem 1)
        self.R_board_cam = rmat
        self.t_board_cam = tvec

        # Charuco board pose in LiDAR frame(from Problem 2)
        self.t_board_lidar = circle_center
        self.plane_normal_lidar = plane_model

    def cam_to_lidar(self):
        # Step 1: Normalize the board normal (Z-axis) in LiDAR frame
        z_lidar = self.plane_normal_lidar / np.linalg.norm(self.plane_normal_lidar)
        z_cam = self.R_board_cam[:, 2]  # Z-axis from camera frame rotation matrix

        # Step 2: Compute rotation to align Z-axes
        rotation_axis = np.cross(z_lidar, z_cam)  # Cross product(gives axis of rotation)
        rotation_axis /= np.linalg.norm(rotation_axis)
        rotation_angle = np.arccos(
            np.clip(np.dot(z_lidar, z_cam), -1.0, 1.0))  # Dot product(used to calculate angular offset)
        R_align_z, _ = cv2.Rodrigues(
            rotation_axis * rotation_angle)  # converting from Rodrigues format to normal rotation matrix.

        # Step 3: Align X-axis
        y_cam = self.R_board_cam[:, 1]  # Y-axis from camera frame
        y_ref = R_align_z @ y_cam  # Rotate Y-axis reference
        x_lidar = np.cross(z_lidar, y_ref)  # Compute new X-axis
        x_lidar /= np.linalg.norm(x_lidar)  # Normalize
        x_cam = self.R_board_cam[:, 0]  # X-axis from camera frame

        rotation_axis_x = np.cross(x_lidar, x_cam)
        rotation_axis_x /= np.linalg.norm(rotation_axis_x)
        rotation_angle_x = np.arccos(np.clip(np.dot(x_lidar, x_cam), -1.0, 1.0))
        R_align_xy, _ = cv2.Rodrigues(rotation_axis_x * rotation_angle_x)

        # Final Rotation Matrix
        R_lidar_to_camera = R_align_xy @ R_align_z

        # Step 4: Compute translation
        transformed_lidar_translation = R_align_z @ self.t_board_lidar
        t_lidar_to_camera = transformed_lidar_translation - self.t_board_cam

        # Final Transformation Matrix
        T_lidar_to_camera = np.eye(4)
        T_lidar_to_camera[:3, :3] = R_lidar_to_camera
        T_lidar_to_camera[:3, 3] = t_lidar_to_camera

        return R_lidar_to_camera, t_lidar_to_camera, T_lidar_to_camera


lidar_data_dir = "/home/pratham/Desktop/documents/Tasks/Ati_motors/pc_save/"
camera_data_dir = "/home/pratham/Desktop/documents/Tasks/Ati_motors/cv_assignment/"

lidar_data_files = glob.glob(lidar_data_dir + "*.npy")

for i in lidar_data_files:
    lidar_file_path = i
    camera_file_path = camera_data_dir + "cam_" + str(i.split("/")[-1].split(".")[0].split("_")[-1]) + ".png"

    print("Current Image: ", camera_file_path)
    if (os.path.exists(lidar_file_path) and os.path.exists(camera_file_path)):
        # Problem 1: Charuco Marker Detection and Pose Estimation using camera data
        camera = Camera()
        rmat, tvec = camera.marker_detection_pose_estimation(camera_file_path)

        if (rmat[0,0] == 0 and tvec[0] == 0):
            continue

        print("\nProblem 1 Solution: ")
        print("Rotation Matrix: ")
        print(rmat)
        print("\nTranslation Vector: ")
        print(tvec)

        # Problem 2: Retroreflective Tape Detection using Lidar data
        lidar = Lidar()
        radius, circle_center, plane_model = lidar.retroreflective_tape_detection(lidar_file_path)

        print("\n\nProblem 2 Solution: ")
        print("\tCircle Center: ", circle_center)
        print("\tCircle Radius: ", radius)
        print("\tPlain Model: ", plane_model)

        # Problem 3: Compute Transformation Between Camera and Lidar
        transformation = Transformation(rmat, tvec, circle_center, plane_model[:3])
        R_lidar_to_camera, t_lidar_to_camera, T_lidar_to_camera = transformation.cam_to_lidar()

        print("\n\nProblem 3 Solution: ")

        print("Rotation: ")
        print(R_lidar_to_camera)

        print("\nTranslation: ")
        print(t_lidar_to_camera)

        print("\nCombined: ")
        print(T_lidar_to_camera)

    print("\n\n\n\n")
