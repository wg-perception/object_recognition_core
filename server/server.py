#!/usr/bin/env python
import CGIHTTPServer
import BaseHTTPServer

if __name__ == '__main__':
    try:
        PORT = 8080
        Handler = CGIHTTPServer.CGIHTTPRequestHandler
        httpd = BaseHTTPServer.HTTPServer(("", PORT), Handler)
        httpd.serve_forever()
    finally:
        httpd.socket.close()