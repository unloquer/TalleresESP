/*
  This a simple example of the aREST UI Library for the ESP8266.
  See the README file for more details.
  Written in 2014-2016 by Marco Schwartz under a GPL license.
*/

// Import required libraries
#include <ESP8266WiFi.h>
#include <aREST.h>
#include <aREST_UI.h>
#include <DHT.h>

#define DHTPIN 12     // what pin we're connected to

// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11

// Initialize DHT sensor for normal 16mhz Arduino
DHT dht(DHTPIN, DHTTYPE);

// Create aREST instance
aREST_UI rest = aREST_UI();

// WiFi parameters
const char* ssid = "K5";
const char* password = "seracambiar9";

// The port to listen for incoming TCP connections
#define LISTEN_PORT           80

// Create an instance of the server
WiFiServer server(LISTEN_PORT);

// Variables to be exposed to the API
float temperature;
float humidity;

void setup(void) {
  // Start Serial
  Serial.begin(115200);

  // Set the title
  rest.title("aREST UI Demo");

  // Init variables and expose them to REST API
  temperature = 0;
  humidity = 0;
  rest.variable("temperature", &temperature);
  rest.variable("humidity", &humidity);

  // Labels
  rest.label("temperature");
  rest.label("humidity");

  // Give name and ID to device
  rest.set_id("1");
  rest.set_name("esp8266");

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  // Start the server
  server.begin();
  Serial.println("Server started");

  // Print the IP address
  Serial.println(WiFi.localIP());

}

void loop() {
  // Handle REST calls
  WiFiClient client = server.available();
  
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();

  //Serial.print("temperature ");Serial.println(temperature);
  //Serial.print("humidity ");Serial.println(humidity);

  if (!client) {
    return;
  }
  while (!client.available()) {
    delay(1);
  }
  rest.handle(client);
}
