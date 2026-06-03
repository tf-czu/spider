# -*- coding: UTF-8 -*-

import numpy as np

from osgar.node import Node

from lib.pointcloud_interpreter import PointCloudInterpreter
from lib.lidar_to_pointcloud import LidarToPointCloud


class CloudMapper(Node):
    """
    OSGAR node responsible for lidar-based terrain mapping.

    The Mapper receives raw lidar data from the OSGAR bus,
    converts the scans into point clouds and forwards them
    to PointCloudInterpreter for higher-level terrain analysis.

    Input streams:
        - lidar_metadata
        - lidar_scan3d
        - lidar_reflectivity

    Internal processing pipeline:

        lidar scan
            |
        LidarToPointCloud
            |
        point cloud (N x 3)
            |
        PointCloudInterpreter
            |
        terrain maps

    Expected output maps:
        - obstacle map
        - slope map
        - traversability map
        - height map
        - angular obstacle map
    """

    def __init__(self, config, bus):
        """
        Initializes the Mapper node.

        Args:
            config (dict):
                OSGAR node configuration.

            bus:
                OSGAR communication bus.
        """

        super().__init__(config, bus)

        # register a stream to be published
        # bus.register('pose3d')

        # super-class Node sets this to `True` if --verbose parameter is applied
        self.verbose = False

        # interprets accumulated point clouds into terrain maps
        self.interpreter = PointCloudInterpreter(output_frequency=1.0)

        # lidar scan -> point cloud converter
        # initialized after lidar metadata are received
        self.l2pc = None

        # data for draw()
        self.draw_timestamps = []
        self.draw_min_maps = []
        self.draw_max_maps = []
        self.draw_dif_maps = []
        self.draw_slope_maps = []
        self.draw_angular_distances = []
        self.draw_angular_counts = []
        self.draw_hole_fuzzy_maps = []

    def on_lidar_metadata(self, data):
        """
        Processes lidar geometry metadata.

        This callback is expected to be processed before
        lidar scans are received.

        The metadata are used to initialize:
            - sensor geometry
            - XYZ lookup table
            - lidar scan to point cloud conversion

        Args:
            data (str):
                JSON string containing lidar metadata.
        """

        # initialize lidar converter only once
        if self.l2pc is None:
            self.l2pc = LidarToPointCloud(data)

    def on_lidar_scan3d(self, data):
        """
        Processes one lidar distance scan.

        The incoming range image is converted into
        a 3D point cloud and passed to PointCloudInterpreter.

        Args:
            data (numpy.array):
                Lidar range scan of size H x W.
        """

        # ignore scans until lidar metadata are initialized
        if self.l2pc is not None:

            # convert lidar scan to Nx3 point cloud
            points = self.l2pc.convert(data)

            # update terrain interpretation pipeline
            output = self.interpreter.update(self.time, points)

            if self.verbose and output is not None:
                self.draw_timestamps.append(output["timestamp"])
                #self.draw_min_maps.append(output["min_map"])
                #self.draw_max_maps.append(output["max_map"])
                self.draw_dif_maps.append(output["dif_map"])
                #self.draw_slope_maps.append(output["slope_map"])
                self.draw_angular_distances.append(output["angular_distances"])
                #self.draw_angular_counts.append(output["angular_counts"])
                self.draw_hole_fuzzy_maps.append(output["hole_fuzzy_mask"])
                print(len(self.draw_timestamps), "...", self.interpreter.get_execution_times())

    def on_lidar_reflectivity(self, data):
        """
        Processes one lidar reflectivity scan.

        Reflectivity information may later be used for:
            - vegetation detection
            - terrain classification
            - surface material estimation

        Args:
            data (numpy.array):
                Lidar reflectivity scan of size H x W.
        """

        if self.l2pc is not None:
            # print("REFLECTIVITY:", data.shape)
            pass

    def draw(self):
        """
        Draws debug visualization.

        If verbose mode is enabled, this method shows three maps at once:
            - grid-based terrain map
            - angular obstacle-distance map
            - fuzzy hole-candidate map

        Keyboard controls:
            right arrow ... next frame
            left arrow  ... previous frame
        """
        if not self.verbose:
            return

        import matplotlib.pyplot as plt

        grid_maps = self.draw_dif_maps
        angular_maps = self.draw_angular_distances
        hole_maps = self.draw_hole_fuzzy_maps
        timestamps = self.draw_timestamps

        if not grid_maps or not angular_maps or not hole_maps:
            print("No maps to draw.")
            return

        fig, (ax_grid, ax_angular, ax_holes) = plt.subplots(1, 3, figsize=(18, 6))
        idx = 0

        grid_img = ax_grid.imshow(
            grid_maps[idx].T,
            origin="lower",
            aspect="equal",
            vmin=0.0,
            vmax=1.0,
        )
        plt.colorbar(grid_img, ax=ax_grid, label="Height difference [m]")
        ax_grid.set_title("Grid map")

        angular_distances = angular_maps[idx]
        angles = np.linspace(0.0, 2.0 * np.pi, len(angular_distances), endpoint=False)
        valid = np.isfinite(angular_distances)
        angular_x = angular_distances[valid] * np.cos(angles[valid])
        angular_y = angular_distances[valid] * np.sin(angles[valid])

        angular_scatter = ax_angular.scatter(angular_x, angular_y, s=8)
        center_scatter = ax_angular.scatter([0.0], [0.0], s=60, marker="+")

        max_range = self.interpreter.angular_mapper.max_distance
        ax_angular.set_xlim(-max_range, max_range)
        ax_angular.set_ylim(-max_range, max_range)
        ax_angular.set_aspect("equal")
        ax_angular.grid(True)
        ax_angular.set_xlabel("X [m]")
        ax_angular.set_ylabel("Y [m]")
        ax_angular.set_title("Angular obstacle map")

        hole_img = ax_holes.imshow(
            hole_maps[idx].T,
            origin="lower",
            aspect="equal",
            vmin=0.0,
            vmax=1.0,
        )
        plt.colorbar(hole_img, ax=ax_holes, label="Hole fuzzy mask [-]")
        ax_holes.set_title("Hole candidates")

        title = fig.suptitle(f"Frame {idx + 1}/{len(grid_maps)}  t={timestamps[idx]}")

        def draw_update_image():
            grid_img.set_data(grid_maps[idx].T)

            angular_distances = angular_maps[idx]
            valid = np.isfinite(angular_distances)
            angular_x = angular_distances[valid] * np.cos(angles[valid])
            angular_y = angular_distances[valid] * np.sin(angles[valid])
            angular_scatter.set_offsets(np.column_stack([angular_x, angular_y]))

            hole_img.set_data(hole_maps[idx].T)

            title.set_text(f"Frame {idx + 1}/{len(grid_maps)}  t={timestamps[idx]}")
            fig.canvas.draw_idle()

        def draw_on_key(event):
            nonlocal idx

            if event.key == "right":
                idx = min(idx + 1, len(grid_maps) - 1)
                draw_update_image()

            elif event.key == "left":
                idx = max(idx - 1, 0)
                draw_update_image()

        fig.canvas.mpl_connect("key_press_event", draw_on_key)
        plt.show()
