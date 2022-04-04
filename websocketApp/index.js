// Importing the required modules
const WebSocketServer = require('ws');
var http = require("http");
var fs = require("fs");

const wss = new WebSocketServer.Server({ port: 8080 })

var dataHolder = "nan"


let sockets = [];
wss.on("connection", ws => {
    console.log("new client connected");
    sockets.push(ws);

    ws.on("message", data => {
        console.log(`Client has sent us: ${data}`)
        dataHolder = String(data)
        sockets.forEach(s => {
            s.send(dataHolder);
        });
    });
    // handling what to do when clients disconnects from server
    ws.on("close", () => {
        console.log("the client has disconnected");
    });
    // handling client connection error
    ws.onerror = function () {
        console.log("Some Error occurred")
    }
});
console.log("The WebSocket server is running on port 8080");

var server = http.createServer(function(request, response) {
    response.writeHead(200, {
        'Content-Type': 'text/html'
    });
    fs.readFile('./index.html', null, function (error, data) {
        if (error) {
            response.writeHead(404);
            respone.write('Whoops! File not found!');
        } else {
            response.write(data);
        }
        response.end();
    });
});
server.listen(8082);
