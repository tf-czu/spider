# -*- coding: UTF-8 -*-

import numpy as np

from osgar.node import Node

from lib.pointcloud_interpreter import PointCloudInterpreter
from lib.lidar_to_pointcloud import LidarToPointCloud


class Mapper(Node):
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
        self.interpreter = PointCloudInterpreter(output_frequency = 1.0)

        # lidar scan -> point cloud converter
        # initialized after lidar metadata are received
        self.l2pc = None

        # data for draw()
        self.draw_min_maps = []
        self.draw_max_maps = []
        self.draw_dif_maps = []
        self.draw_timestamps = []

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
                self.draw_min_maps.append(output["min_map"])
                self.draw_max_maps.append(output["max_map"])
                self.draw_dif_maps.append(output["dif_map"])
                print(len(self.draw_timestamps))

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
        if self.verbose:
            import matplotlib.pyplot as plt

            if not self.draw_dif_maps:
                print("No maps to draw.")
                return

            maps = self.draw_dif_maps
            timestamps = self.draw_timestamps

            fig, ax = plt.subplots()
            idx = 0

            img = ax.imshow(
                maps[idx].T,
                origin="lower",
                aspect="equal",
                vmin=0.0,
                vmax=1.0,
            )

            cbar = plt.colorbar(img, ax=ax, label="Height difference [m]")

            title = ax.set_title(f"Frame {idx + 1}/{len(maps)}  t={timestamps[idx]}")

            def draw_update_image():
                img.set_data(maps[idx].T)
                title.set_text(f"Frame {idx + 1}/{len(maps)}  t={timestamps[idx]}")
                fig.canvas.draw_idle()

            def draw_on_key(event):
                nonlocal idx

                if event.key == "right":
                    idx = min(idx + 1, len(maps) - 1)
                    draw_update_image()

                elif event.key == "left":
                    idx = max(idx - 1, 0)
                    draw_update_image()

            fig.canvas.mpl_connect("key_press_event", draw_on_key)

            plt.show()
