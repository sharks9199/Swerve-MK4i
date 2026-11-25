"""SharksPredict local read-only NT4 viewer. Python 3.11+; demo needs no packages."""
import argparse
import json
import math
import threading
import time
import uuid
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parent
TOPIC = '/SharksPredict/Snapshot'


def demo_frame(speed, acceleration, height, target):
    mass, moving, base, support, g, margin, stop = 29.3, 5., .30, .2667, 9.81, .70, 1.
    cg, future = base + moving/mass*height, base + moving/mass*target
    worst = max(cg, future)
    theoretical = g*support/worst
    limit = theoretical*margin
    braking = speed*speed/(2*stop)
    demand = max(acceleration, braking)
    return {'schema': 1, 'session': 'demo', 'sequence': 1, 'time': 0., 'mode': 'DEMO', 'robotMode': 'TELEOP',
            'observationOnly': True, 'action': 'EXTEND_ELEVATOR_L4', 'requestId': 1,
            'decision': 'WOULD_ALLOW' if demand <= limit else 'WOULD_BLOCK',
            'reason': 'Cenário fictício. Nenhum robô conectado.', 'speed': speed, 'vx': speed, 'vy': 0., 'omega': 0.,
            'acceleration': acceleration, 'battery': None, 'x': None, 'y': None, 'heading': None,
            'height': height, 'target': target, 'actualTarget': target, 'elevatorState': 'EXTENDING', 'preset': 'L4',
            'intakeState': 'DESLIGADO', 'intakeAngle': 0., 'active': [{'id':1,'name':'SwerveTeleop (exemplo)','startedAt':0.,'subsystems':['Drive']}],
            'events': [{'id':1,'time':0.,'type':'REQUEST','detail':'EXTEND_ELEVATOR_L4 (exemplo)'}],
            'model': {'mass':mass,'movingMass':moving,'baseCg':base,'support':support,'gravity':g,'margin':margin,'stopDistance':stop,'calibrated':False},
            'physics': {'currentCgM':cg,'futureCgM':future,'theoreticalAccel':theoretical,'modelAccel':limit,'brakingDemand':braking,
                        'combinedDemand':demand,'tippingTorque':mass*demand*worst,'restoringTorque':mass*g*support,
                        'utilization':demand/limit,'insideModel':demand<=limit}, 'calculationMs':None}


class NTSource:
    def __init__(self, host):
        import ntcore
        self.instance = ntcore.NetworkTableInstance.getDefault()
        self.instance.startClient4('SharksPredict-' + uuid.uuid4().hex[:8])
        self.instance.setServer(host, 5810)
        self.subscriber = self.instance.getStringTopic(TOPIC).subscribe('')
        self.host = host
        self.last_identity = None
        self.last_change = 0.
        self.lock = threading.Lock()

    def read(self):
        with self.lock:
            connected = self.instance.isConnected()
            if not connected:
                # Retain last identity so a cached pre-disconnect frame cannot look live.
                self.last_change = 0.
                return {'status':'disconnected','host':self.host,'frame':None}
            raw = self.subscriber.get()
            try:
                frame = json.loads(raw)
                if not isinstance(frame, dict) or frame.get('schema') != 1:
                    raise ValueError('schema desconhecido')
                identity = (frame['session'], frame['sequence'])
                if self.last_identity is None:
                    # Require a second distinct frame; NT may deliver a cached last value.
                    self.last_identity = identity
                    return {'status':'waiting','host':self.host,'frame':None}
                if identity != self.last_identity:
                    self.last_identity, self.last_change = identity, time.monotonic()
                if self.last_change == 0. or time.monotonic()-self.last_change > 1.5:
                    return {'status':'stale','host':self.host,'frame':None}
                return {'status':'live','host':self.host,'frame':frame}
            except (ValueError, KeyError, TypeError):
                return {'status':'waiting','host':self.host,'frame':None}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', default='127.0.0.1', help='IP da roboRIO ou simulador')
    parser.add_argument('--port', type=int, default=9199)
    parser.add_argument('--demo', action='store_true')
    parser.add_argument('--no-browser', action='store_true')
    args = parser.parse_args()
    try:
        source = None if args.demo else NTSource(args.robot)
    except ImportError:
        parser.exit(1, 'Instale: python -m pip install -r requirements.txt\nOu use --demo.\n')

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path == '/api/state':
                data = {'status':'demo','host':'local','frame':demo_frame(3.,3.,.30,1.6)} if args.demo else source.read()
                body = json.dumps(data, allow_nan=False).encode()
                mime = 'application/json; charset=utf-8'
            elif path in ('/', '/index.html', '/style.css', '/app.js'):
                name = 'index.html' if path == '/' else path[1:]
                body = (ROOT/name).read_bytes()
                mime = {'index.html':'text/html','style.css':'text/css','app.js':'text/javascript'}[name]+'; charset=utf-8'
            else:
                self.send_error(404); return
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            self.wfile.write(body)
        def log_message(self, fmt, *args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    url = f'http://127.0.0.1:{server.server_port}'
    print(f'SharksPredict 1.0 | {url} | {"DEMO" if args.demo else args.robot}', flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if source:
            source.instance.stopClient()

if __name__ == '__main__':
    main()
