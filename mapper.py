# -*- coding: UTF-8 -*-

import numpy as np

from osgar.node import Node

from lib.lidar_interpreter import LidarInterpreter


class Mapper(Node):
    """
    OSGAR node responsible for scan-based lidar interpretation.

    The Mapper receives raw lidar range scans from the OSGAR bus and forwards
    them directly to LidarInterpreter. No point cloud is created in this branch.

    Input streams:
        - lidar_metadata
        - lidar_scan3d
        - lidar_reflectivity

    Internal processing pipeline:

        lidar scan
            |
        LidarInterpreter
            |
        angular obstacle and hole maps
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

        self.verbose = False

        self.interpreter = LidarInterpreter(output_frequency=1.0)

        self.draw_timestamps = []
        self.draw_obstacle_distances = []
        self.draw_hole_distances = []
        self.draw_obstacle_pixel_masks = []
        self.draw_hole_pixel_masks = []

    def on_lidar_metadata(self, data):
        """
        Processes lidar metadata.

        The scan-based interpreter does not need lidar geometry metadata yet,
        but the callback is kept so the OSGAR stream can be connected.

        Args:
            data (str):
                JSON string containing lidar metadata.
        """
        pass

    def on_lidar_scan3d(self, data):
        """
        Processes one lidar range scan.

        Args:
            data (numpy.array):
                Lidar range scan of size H x W.
        """

        output = self.interpreter.update(self.time, data)

        if self.verbose and output is not None:
            self.draw_timestamps.append(output["timestamp"])
            self.draw_obstacle_distances.append(output["obstacle_distances"])
            self.draw_hole_distances.append(output["hole_distances"])
            self.draw_obstacle_pixel_masks.append(output["obstacle_pixel_mask"])
            self.draw_hole_pixel_masks.append(output["hole_pixel_mask"])
            print(len(self.draw_timestamps), "...", self.interpreter.get_execution_times())

    def on_lidar_reflectivity(self, data):
        """
        Processes one lidar reflectivity scan.

        Args:
            data (numpy.array):
                Lidar reflectivity scan of size H x W.
        """
        pass

    def draw(self):
        """
        Draws debug visualization.

        If verbose mode is enabled, this method shows three maps:
            - angular obstacle-distance map
            - angular hole-distance map
            - pixel masks of the latest interpreted scan

        Keyboard controls:
            right arrow ... next frame
            left arrow  ... previous frame
        """
        if not self.verbose:
            return

        import matplotlib.pyplot as plt

        obstacle_maps = self.draw_obstacle_distances
        hole_maps = self.draw_hole_distances
        obstacle_pixel_masks = self.draw_obstacle_pixel_masks
        hole_pixel_masks = self.draw_hole_pixel_masks
        timestamps = self.draw_timestamps

        if not obstacle_maps or not hole_maps:
            print("No maps to draw.")
            return

        #fig, (ax_obstacles, ax_holes, ax_pixels) = plt.subplots(1, 3, figsize=(18, 6))

        fig, (ax_obstacles, ax_holes, ax_pixels) = plt.subplots(
            1,
            3,
            figsize=(30, 6),
            gridspec_kw={"width_ratios": [1, 1, 5]},
        )

        idx = 0

        angles = np.linspace(
            0.0,
            2.0 * np.pi,
            self.interpreter.number_of_bins,
            endpoint=False,
        )

        obstacle_distances = obstacle_maps[idx]
        obstacle_valid = np.isfinite(obstacle_distances)
        obstacle_x = obstacle_distances[obstacle_valid] * np.cos(angles[obstacle_valid])
        obstacle_y = obstacle_distances[obstacle_valid] * np.sin(angles[obstacle_valid])

        obstacle_scatter = ax_obstacles.scatter(obstacle_x, obstacle_y, s=8)
        ax_obstacles.scatter([0.0], [0.0], s=60, marker="+")
        ax_obstacles.set_title("Scan obstacle map")

        hole_distances = hole_maps[idx]
        hole_valid = np.isfinite(hole_distances)
        hole_x = hole_distances[hole_valid] * np.cos(angles[hole_valid])
        hole_y = hole_distances[hole_valid] * np.sin(angles[hole_valid])

        hole_scatter = ax_holes.scatter(hole_x, hole_y, s=8)
        ax_holes.scatter([0.0], [0.0], s=60, marker="+")
        ax_holes.set_title("Scan hole map")

        for ax in (ax_obstacles, ax_holes):
            max_range = self.interpreter.max_range
            ax.set_xlim(-max_range, max_range)
            ax.set_ylim(-max_range, max_range)
            ax.set_aspect("equal")
            ax.grid(True)
            ax.set_xlabel("X [m]")
            ax.set_ylabel("Y [m]")

        obstacle_mask = obstacle_pixel_masks[idx]
        hole_mask = hole_pixel_masks[idx]
        pixel_show = np.zeros(obstacle_mask.shape, dtype=float)
        pixel_show[obstacle_mask] = 1.0
        if hole_mask.shape[0] == pixel_show.shape[0] - 1:
            pixel_show[:-1, :][hole_mask] = 2.0

        #pixel_img = ax_pixels.imshow(
        #    pixel_show,
        #    origin="lower",
        #    aspect="auto",
        #    vmin=0.0,
        #    vmax=2.0,
        #)
        pixel_img = ax_pixels.imshow(
            pixel_show,
            origin="lower",
            aspect="auto",
            interpolation="nearest",
            vmin=0.0,
            vmax=2.0,
        )
        plt.colorbar(pixel_img, ax=ax_pixels, label="0 none, 1 obstacle, 2 hole")
        ax_pixels.set_title("Scan pixel detections")
        ax_pixels.set_xlabel("Azimuth column")
        ax_pixels.set_ylabel("Lidar channel")

        title = fig.suptitle(f"Frame {idx + 1}/{len(obstacle_maps)}  t={timestamps[idx]}")

        def draw_update_image():
            obstacle_distances = obstacle_maps[idx]
            obstacle_valid = np.isfinite(obstacle_distances)
            obstacle_x = obstacle_distances[obstacle_valid] * np.cos(angles[obstacle_valid])
            obstacle_y = obstacle_distances[obstacle_valid] * np.sin(angles[obstacle_valid])
            obstacle_scatter.set_offsets(np.column_stack([obstacle_x, obstacle_y]))

            hole_distances = hole_maps[idx]
            hole_valid = np.isfinite(hole_distances)
            hole_x = hole_distances[hole_valid] * np.cos(angles[hole_valid])
            hole_y = hole_distances[hole_valid] * np.sin(angles[hole_valid])
            hole_scatter.set_offsets(np.column_stack([hole_x, hole_y]))

            obstacle_mask = obstacle_pixel_masks[idx]
            hole_mask = hole_pixel_masks[idx]
            pixel_show = np.zeros(obstacle_mask.shape, dtype=float)
            pixel_show[obstacle_mask] = 1.0
            if hole_mask.shape[0] == pixel_show.shape[0] - 1:
                pixel_show[:-1, :][hole_mask] = 2.0
            pixel_img.set_data(pixel_show)

            title.set_text(f"Frame {idx + 1}/{len(obstacle_maps)}  t={timestamps[idx]}")
            fig.canvas.draw_idle()

        def draw_on_key(event):
            nonlocal idx

            if event.key == "right":
                idx = min(idx + 1, len(obstacle_maps) - 1)
                draw_update_image()

            elif event.key == "left":
                idx = max(idx - 1, 0)
                draw_update_image()

        fig.canvas.mpl_connect("key_press_event", draw_on_key)
        plt.show()
