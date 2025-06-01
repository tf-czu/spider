import argparse

from unittest.mock import MagicMock

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize
from osgar.bus import Bus

from localization import Localization

def main():
    arg_parser = argparse.ArgumentParser(description = 'Applies LeastSquaresLocalization to an OSGAR log; plots the results')
    arg_parser.add_argument("input", help = "path to input log file")
    args = arg_parser.parse_args()
    path_log = args.input

    bus = Bus(MagicMock())
    config = {
        "algorithm": "lsqr",
        "enc scale": 0.00218,
        "window": 20,
        "prune": 20,
        "post window": 20,
        "initial angle": -75,
        "initial scale":  1.0,
        "initial window": 3.0,
        "initial dumb distance": 0.01,
        "gps err": [2, 2, 6],
        "imu err": [4, 4, 100]
    }

    id_ori = lookup_stream_id(path_log, "imu.orientation")
    id_gps = lookup_stream_id(path_log, "gps.nmea_data")
    #id_p2d = lookup_stream_id(path_log, "spider.pose2d")
    id_enc = lookup_stream_id(path_log, "spider.encoders")

    #list_of_stream_ids = [id_ori, id_gps, id_p2d, id_enc]
    list_of_stream_ids = [id_ori, id_gps, id_enc]

    localization = Localization(config, bus.handle('abc'))
    localization.verbose = True

    with LogReader(path_log, only_stream_id = list_of_stream_ids) as log:
        #counter = 0
        for timestamp, stream_id, data_raw in log:
            localization.time = timestamp
            if stream_id == id_ori:
                localization.on_orientation(deserialize(data_raw))
            elif stream_id == id_gps:
                localization.on_nmea(deserialize(data_raw))
            #elif stream_id == id_p2d:
            #    localization.on_pose2d(deserialize(data_raw))
            elif stream_id == id_enc:
                localization.on_encoders(deserialize(data_raw))
            #counter += 1
            #if counter % 1000 == 0:
            #    print(counter)

    localization.draw()

if __name__ == "__main__":
    main()
