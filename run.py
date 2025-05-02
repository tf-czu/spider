import argparse

from unittest.mock import MagicMock

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize
from osgar.bus import Bus

from lsqr_localization import LeastSquaresLocalization

def main():
    arg_parser = argparse.ArgumentParser(description = 'Applies LeastSquaresLocalization to an OSGAR log; plots the results')
    arg_parser.add_argument("input", help = "path to input log file")
    arg_parser.add_argument("-p", help = "post-process window size in meters")
    arg_parser.add_argument("-w", help = "window size in meters")
    arg_parser.add_argument("--prune", help = "pruning of the input data")
    arg_parser.add_argument("--init-win", help = "initial window [m]")
    arg_parser.add_argument("--init-sca", help = "initial scale")
    arg_parser.add_argument("--init-ang", help = "initial angle [deg]")
    args = arg_parser.parse_args()
    path_log = args.input

    window = 5
    post_window = 5
    prune = 20
    initial_scale = 1
    initial_window = 1
    initial_angle = -75

    if args.w:
        window = float(args.w)
    if args.p:
        post_window = float(args.p)
    if args.prune:
        prune = int(args.prune)
    if args.init_win:
        init_win = float(args.init_win)
    if args.init_sca:
        init_sca = float(args.init_sca)
    if args.init_ang:
        init_ang = float(args.init_ang)

    bus = Bus(MagicMock())
    config = {
        'window':          window,
        'post_window':     post_window,
        'prune':           prune,
        'initial_scale':   initial_scale,
        'initial_window':  initial_window,
        'initial_angle':   initial_angle,
    }

    id_ori = lookup_stream_id(path_log, "imu.orientation")
    id_gps = lookup_stream_id(path_log, "gps.nmea_data")
    id_p2d = lookup_stream_id(path_log, "spider.pose2d")
    id_enc = lookup_stream_id(path_log, "spider.encoders")

    list_of_stream_ids = [id_ori, id_gps, id_p2d, id_enc]

    localization = LeastSquaresLocalization(config, bus.handle('abc'))

    with LogReader(path_log, only_stream_id = list_of_stream_ids) as log:
        counter = 0
        for timestamp, stream_id, data_raw in log:
            localization.time = timestamp
            if stream_id == id_ori:
                localization.on_orientation(deserialize(data_raw))
            elif stream_id == id_gps:
                localization.on_nmea(deserialize(data_raw))
            elif stream_id == id_p2d:
                localization.on_pose2d(deserialize(data_raw))
            elif stream_id == id_enc:
                localization.on_encoders(deserialize(data_raw))
            counter += 1
            if counter % 1000 == 0:
                print(counter)

    localization.draw()

if __name__ == "__main__":
    main()
