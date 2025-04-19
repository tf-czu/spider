import time
import threading
import zmq

from flask import Flask, render_template, Response, jsonify

from osgar.lib.serialize import serialize, deserialize


app = Flask(__name__)
frame_lock = threading.Lock()

class Zmq:
    def __init__(self):
        self.latest_frames = {}  # {"cam1": <jpeg>, "cam2": <jpeg>, ...}
        self.points = []

        # Another context for PUSH TODO
        context = zmq.Context.instance()
        self.socket_push = context.socket(zmq.PUSH)
        self.socket_push.setsockopt(zmq.LINGER, 100)  # milliseconds
        self.socket_push.bind("tcp://*:5561")

    def pull_msg(self):
        context = zmq.Context.instance()
        socket = context.socket(zmq.PULL)
        # https://stackoverflow.com/questions/7538988/zeromq-how-to-prevent-infinite-wait
        socket.RCVTIMEO = 100  # milliseconds
        socket.LINGER = 100
        socket.bind("tcp://*:5560")

        while True:
            try:
                channel, raw = socket.recv_multipart()
                data = deserialize(raw)
                # print(channel, message)
                channel = channel.decode('ascii')
                with frame_lock:
                    if channel == "pose3d":
                        assert len(data) == 2, data
                        (x, y, z), quat = data
                        self.points.append([x, y])
                    else:
                        self.latest_frames[channel] = data

            except zmq.ZMQError as e: # zmq.error.Again:
                # print(e)
                pass

    def push_msg(self, msg):
        try:
            channel, data = msg
            raw = serialize(data)
            self.socket_push.send_multipart([bytes(channel, 'ascii'), raw])
        except zmq.ZMQError as e:
            pass


def generate_frames(cam_id):
    while True:
        with frame_lock:
            frame = connection.latest_frames.get(cam_id, None)

        if frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.05)

@app.route('/')
def index():
    with frame_lock:
        cameras = sorted(connection.latest_frames.keys())
    default_cams = cameras[:3] + ["" for _ in range(3 - len(cameras))]  # add empty strings if no cameras
    return render_template('index.html', cameras=cameras, default_cams=default_cams)

@app.route('/video_feed/<cam_id>')
def video_feed(cam_id):
    return Response(generate_frames(cam_id),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/points')
def get_points():
    with frame_lock:
        return jsonify(connection.points)

if __name__ == '__main__':
    connection = Zmq()
    t = threading.Thread(target=connection.pull_msg, daemon=True)
    t.start()
    app.run(debug=True, use_reloader=False)
