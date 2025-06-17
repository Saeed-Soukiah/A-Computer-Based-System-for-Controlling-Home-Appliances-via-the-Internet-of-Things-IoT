#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>

// Wi-Fi credentials
const char* ssid = "Saeed Network";
const char* password = "123456789";

// LED and serial
const int led = 2;
SoftwareSerial swSerial(D5, D6, false, 128); // GPIO14 -> D5, GPIO12 -> D6

WiFiServer server(80);

void setup() {
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  swSerial.begin(9600);
  Serial.begin(115200);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected to Wi-Fi.");
  server.begin();
}

void sendHTML(WiFiClient& client) {
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: text/html"));
  client.println();
  client.println(F("<!DOCTYPE html><html><head><title>Smart Home</title>"));
  client.println(F("<meta name='viewport' content='width=device-width, initial-scale=1'>"));
  client.println(F("<link href='https://cdn.jsdelivr.net/npm/bootstrap@5.2.2/dist/css/bootstrap.min.css' rel='stylesheet'>"));
  client.println(F("</head><body style='text-align:center;'>"));
  client.println(F("<h1>Home Automation Control</h1>"));
  
  // Example control section
  client.println(F("<h3>Light Control</h3>"));
  client.println(F("<a href='/ON' class='btn btn-success'>ON</a>"));
  client.println(F("<a href='/OFF' class='btn btn-danger'>OFF</a>"));

  // Add more control groups here (fan, lock, etc.)

  client.println(F("</body></html>"));
}

void handleRequest(const String& request) {
  if (request.indexOf("/ON") != -1)        swSerial.println("E");
  else if (request.indexOf("/OFF") != -1)   swSerial.println("F");
  else if (request.indexOf("/Open") != -1)  swSerial.println("O");
  else if (request.indexOf("/Close") != -1) swSerial.println("P");
  else if (request.indexOf("/Speed-1") != -1) swSerial.println("I");
  else if (request.indexOf("/Speed-2") != -1) swSerial.println("J");
  else if (request.indexOf("/Fan-OFF") != -1) swSerial.println("K");
  else if (request.indexOf("/Dim") != -1)   swSerial.println("L");
  else if (request.indexOf("/Light") != -1) swSerial.println("M");
  else if (request.indexOf("/Wifi-Lock") != -1) swSerial.println("C");
  else if (request.indexOf("/Wifi-UnLock") != -1) swSerial.println("D");
  else if (request.indexOf("/Wifi-Light-Off") != -1) swSerial.println("A");
  else if (request.indexOf("/Wifi-Light-on") != -1) swSerial.println("B");
}

void loop() {
  WiFiClient client = server.available();
  if (!client) return;

  while (!client.available()) delay(1);
  String request = client.readStringUntil('\r');
  client.flush();

  Serial.println("Client Request: " + request);

  sendHTML(client);
  handleRequest(request);

  delay(1);
  client.stop();
}
