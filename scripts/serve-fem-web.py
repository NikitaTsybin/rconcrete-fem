"""Local FEM server with shared-memory isolation for the optional thread pool."""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

class Handler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Cross-Origin-Opener-Policy', 'same-origin')
        self.send_header('Cross-Origin-Embedder-Policy', 'require-corp')
        super().end_headers()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8766)
    args = parser.parse_args()
    directory = Path(__file__).resolve().parents[1] / 'fem_web'
    ThreadingHTTPServer(('127.0.0.1', args.port), partial(Handler, directory=str(directory))).serve_forever()
