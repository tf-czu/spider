"""
  Log video stream provided by OpenCV camera
"""

import cv2
from threading import Thread

from ouster.sdk import open_source
from ouster.sdk.core import XYZLut, destagger, ChanField

class LidarOuster:
    def __init__(self, config, bus):
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.bus = bus
        bus.register('3dscan')
        inputfile_json = config["inputfile"] + ".json"
        inputfile_pcap = config["inputfile"] + ".pcap"

        print(" --- inputfile_json", inputfile_json)
        print(" --- inputfile_pcap", inputfile_pcap)

        self.source = open_source(inputfile_pcap, meta=[inputfile_json], sensor_idx=0, collate=False)
        self.info = self.source.sensor_info[0] # geometrie senzoru
        self.xyz_lut = XYZLut(self.info) # lookup tabulka pro prevod ranges -> xyz
        self.counter = 0
        self.singler = iter(self.source.single(0))

    def start(self):
        self.input_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)

    def run_input(self):
        while self.bus.is_alive():
            # dalsi frame z lidaru
            try:
                scan_tuple = next(self.singler)
                lidar_scan = scan_tuple[0]
                # staggered range data (H×W)
                ranges = lidar_scan.field(ChanField.RANGE)
                # prevod na xyz; (H×W×3); xyz[h, w] = [x, y, z]
                xyz = self.xyz_lut(lidar_scan)
                # souradnice v metrech
                # x = xyz[:, :, 0]
                # y = xyz[:, :, 1]
                # z = xyz[:, :, 2]
                # x = dopředu, y = doleva, z = nahoru (pravotočivý systém Ouster)

                # Převod na klasický point cloud (Nx3) ... Většina knihoven (Open3D, PCL, atd.) chce Nx3
                points = xyz.reshape(-1, 3) # Nx3 body
                # odstranění neplatných bodů ... body s range == 0 nejsou validní
                ranges_flat = ranges.reshape(-1)
                points = points[ranges_flat > 0]
                # vkladam to na bus jako point cloud = seznam 3D souradnic
                self.bus.publish('3dscan', points)
                print(self.counter, points.shape)
                self.counter += 1
            except StopIteration:
                break

            # Capture frame-by-frame
            #ret, frame = self.cap.read()
            #if ret:
            #    retval, data = cv2.imencode('*.jpeg', frame)
            #    if len(data) > 0:
            #        self.bus.publish('raw', data.tobytes())
            #    if self.sleep is not None:
            #        self.bus.sleep(self.sleep)
        #self.cap.release()

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4
