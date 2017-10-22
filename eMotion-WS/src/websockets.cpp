#include <WebSocketsClient.h>
#include <Hash.h>

WebSocketsClient client;

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {

    case WStype_DISCONNECTED:
    Serial.println("WebSockets disconnected");
    break;

    case WStype_CONNECTED: {
      Serial.println("WebSockets connected");
  		// client->sendTXT(num, "Connected");
      break;
    }

    case WStype_TEXT:
    Serial.printf("[%u] get Text: %s\n", length, payload);
    // send message to client
    client.sendTXT("message here");
    // send data to all connected clients
    // webSocket.broadcastTXT("message here");
    break;

    case WStype_BIN:
    // Serial.printf("[%u] get binary lenght: %u\n", num, lenght);
    hexdump(payload, length);
    // send message to client
    // webSocket.sendBIN(num, payload, lenght);
    break;
  }
}

void wsSend(String message) {
  client.sendTXT(message);
}

void wsSetup() {
  client.begin("192.168.88.20", 8080);
  client.onEvent(webSocketEvent);
}

void wsLoop() {
  client.loop();
}
