import argparse

from unittest.mock import MagicMock

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize
from osgar.bus import Bus

from lsqr_localization import LeastSquaresLocalization

def main():
    #arg_parser = argparse.ArgumentParser(description = 'Plots GPS and odometry data from OSGAR log. Fits the odometry trajectory to the GPS trajectory.')
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("input", help = "path to input log file")
    #arg_parser.add_argument("-p", help = "post-process window size in meters")
    #arg_parser.add_argument("-w", help = "window size in meters")
    #arg_parser.add_argument("--prune", help = "pruning of the input data")
    args = arg_parser.parse_args()
    path_log = args.input

    #if args.p:
    #    post_window_in_meters = int(args.p)
    #else:
    #    post_window_in_meters = None
    #if args.w:
    #    window_in_meters = int(args.w)
    #else:
    #    window_in_meters = None
    #if args.prune:
    #    prune = int(args.prune)
    #else:
    #    prune = 1

    bus = Bus(MagicMock())
    config = {
        'window': 5,
        'post_window': 5,
        'prune': 1,   
        'initial_scale':  1.0,   
        'initial_window': 1.0,   
        'initial_angle': -75,   
    }

    id_ori = lookup_stream_id(path_log, "imu.orientation")
    id_gps = lookup_stream_id(path_log, "gps.nmea_data")
    id_p2d = lookup_stream_id(path_log, "spider.pose2d")
    id_enc = lookup_stream_id(path_log, "spider.encoders")

    list_of_stream_ids = [id_ori, id_gps, id_p2d, id_enc]
    list_of_stream_ids = [id_ori, id_gps, id_p2d]

    #localization = LeastSquaresLocalization(config, bus.handle('abc'))

    with LogReader(path_log, list_of_stream_ids) as log:
        counter = 0
        for timestamp, stream_id, data_raw in log:
            #print("timestamp:", timestamp, type(timestamp))
            #if stream_id == only_stream_odo:
            #    localization.add_odo(timestamp, deserialize(data_raw))
            #elif stream_id == only_stream_ori:
            #    localization.add_ori(timestamp, deserialize(data_raw))
            #elif stream_id == only_stream_gps:
            #    localization.add_gps(timestamp, deserialize(data_raw))
            #localization.time = timestamp
            if stream_id == id_ori:
                #print("ORI", deserialize(data_raw))
                #localization.on_orientation(deserialize(data_raw))
                pass
            elif stream_id == id_gps:
                #print("GPS", deserialize(data_raw))
                #localization.on_nmea(deserialize(data_raw))
                pass
            elif stream_id == id_p2d:
                #print("P2D", deserialize(data_raw))
                #localization.on_pose2d(deserialize(data_raw))
                pass
            #elif stream_id == id_enc:
            #    #print("ENC", deserialize(data_raw))
            #    #localization.on_encoders(deserialize(data_raw))
            #    pass
            counter += 1
            if counter % 1000 == 0:
                print(counter)

    #localization.draw()

if __name__ == "__main__":
    main()
