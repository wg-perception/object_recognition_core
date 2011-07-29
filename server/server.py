#!/usr/bin/env python
from SimpleHTTPServer import SimpleHTTPRequestHandler
import SocketServer
import cgi
def bag_upload(handler):
    global rootnode
    print "Bag upload"
    ctype,pdict = cgi.parse_header(handler.headers.getheader('Content-type'))
    if ctype == 'multipart/form-data':
        form = cgi.FieldStorage(handler.rfile,handler.headers)
        print form.getvalue('object_name')
    handler.send_response(301)
    handler.end_headers()
    handler.wfile.write('Post!')

class RecognitionServer(SimpleHTTPRequestHandler):
    get_requests = {}
    post_requests = { '/upload/bag': bag_upload}
    def do_GET(self):
        if self.path in self.get_requests.keys():
            pass
        else:
            SimpleHTTPRequestHandler.do_GET(self)

    def do_POST(self):
        print 'POST:',self.path
        if self.path in self.post_requests.keys():
            self.post_requests[self.path](self)

def main():
    try:
        PORT = 8080
        Handler = RecognitionServer
        httpd = SocketServer.TCPServer(("", PORT), Handler)
        httpd.serve_forever()
    except Exception,e:
        httpd.socket.close()
if __name__ == '__main__':
    main()

