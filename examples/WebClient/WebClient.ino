#include <DHT.h>
#include <UIPEthernet.h>

EthernetClient client;
//server address
char server[] = "192.168.1.28";
//server port
int serverPort = 80;

//my address
uint8_t mac[6] = {0x00,0x01,0x02,0x03,0x04,0x05};
IPAddress myIP(192,168,1,63);

//Pin DHT
int inputPin = 4;
#define DHTTYPE DHT11
DHT dht(inputPin, DHTTYPE);

void setup() {
  Serial.begin(9600);
  pinMode(inputPin, INPUT);
  
  dht.begin();
}

void loop() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  doGet(String(t)); //seng GET request  
  while(client.connected()){
    if(client.available()){
      char c = client.read();
      Serial.print(c);  
    }
  }
}

// http get request
void doGet(String param) {
    Ethernet.begin(mac,myIP);
    if (client.connect(server,serverPort)){
      Serial.println("Connected to server");
      client.print("GET /");
      client.print(param);
      client.println(" HTTP/1.1");
      client.print("Host: ");
      client.println(server);
      client.println();
  }else{
      Serial.println("Connection to server failed");
  }
}
