from http.server import BaseHTTPRequestHandler, HTTPServer

class Handler(BaseHTTPRequestHandler):

    def do_POST(self):
        length = int(self.headers['Content-Length'])
        data = self.rfile.read(length)

        print("\n--- POST RECEIVED ---")
        print("Path:", self.path)
        print("Headers:\n", self.headers)
        print("Body:", data.decode())
        print("---------------------\n")

        self.send_response(200)
        self.end_headers()
        self.wfile.write(b"OK")

server = HTTPServer(("0.0.0.0", 8080), Handler)

print("Listening on port 8080...")
server.serve_forever()