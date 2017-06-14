const express = require('express');
const http = require('http');
const WebSocket = require('ws');

const app = express();
app.use(express.static('public'));
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });

var clients = {};

wss.on('connection', function connection(ws) {
  ws.on('message', (msg) => {
    if(msg.match(/connect:web/)) {
      clients.web = ws;
      return;
    }

    if(msg.match(/connect:device/)) {
      clients.device = ws;
      return;
    }

    clients.web && clients.web.send(msg);

  });
});

server.listen(8080, () => console.log('Listening on http://localhost:8080'));
