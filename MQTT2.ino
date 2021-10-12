/**********
  IFG Campus Goiânia
  Autor:    Carlos Silveira
  Modificações: Thallys H.
  Objetivo: Estabelece comunicação via MQTT Broker e envia tópicos  dos sensores
            de temperatura e umidade do sensor DHT11 a cada cinco segundos            
  Origem:  https://randomnerdtutorials.com/esp8266-nodemcu-mqtt-publish-dht11-dht22-arduino/
     
*********/

#include <ESP8266WiFi.h>  // Inclui biblioteca WiFi
#include <Adafruit_Sensor.h> // Inclui bibliotecas do DHT
#include <DHT.h>
#include <AsyncMqttClient.h> // Inclui biblitecas de comunicação
#include <Ticker.h>

// Define as constantes de conexão de rede WiFi  
const char* REDE_WIFI = "Sua rede";  // Insere o nome da Rede WiFi
const char* SENHA_WIFI = "*******";  // Insere a senha da Rede WiFi

// configura a porta do servidor para número 80 (padrão)
WiFiServer server(80);

// Configurações DHT
#define DHTTIPO DHT11     // Define o modelo DHT 11, DHT21, DHT22
#define DHTPIN D1         // Define o pino de conexão do sensor no ESP32
DHT dht(DHTPIN, DHTTIPO); // DHT (pino,tipo)

// Confiogurações do broker MQTT, insere o domínio e porta
#define MQTT_HOST  "seubroker.com"
#define MQTT_PORT 1883

// Define os nomes variáveis do sensor DHT para o MQQT broker
#define MQTT_PUB_TEMP "esp8266/dht/temperatura"
#define MQTT_PUB_HUM  "esp8266/dht/umidade"

// Define as variáveis para temperatura e umidade
float temp;
float um;

// Cria um cliente MQTT
AsyncMqttClient mqttClient;
// 
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

// Variáveis para registrar o intervalo de repetição de leitura
unsigned long tempoAtual;
unsigned long tempoAnterior = 0;   
const long intevalo = 5000;  

// Rotina que faz a conexão com o Wi-Fi
void conectaAoWifi();

// Rotina que faz a conexão com o MQTT Broker
void aoConectarAoWifi(const WiFiEventStationModeGotIP& event);

// Rotina que faz a desconexão com o Wi-Fi
void aoDesconectarDoWifi(const WiFiEventStationModeDisconnected& event);

// Rotina de conexão do cliente com o MQTT
void conectaAoMqtt();

// Rotina que sinaliza estabelecimento de seção MQTT
void aoConectarAoMqtt(bool sessionPresent);

// Rotina que sinaliza a desconexão com o MQTT
void aoDesconectarDoMqtt(AsyncMqttClientDisconnectReason reason);

// Rotina que sinaliza o endereço do pacote MQTT
void onMqttPublish(uint16_t packetId);


void setup() {
  Serial.begin(115200);  // Inicializa a comunicação serial
  Serial.println();
  dht.begin(); // Inicializa o sensor DHT
  
  wifiConnectHandler = WiFi.onStationModeGotIP(aoConectarAoWifi);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(aoDesconectarDoWifi);

  mqttClient.onConnect(aoConectarAoMqtt);
  mqttClient.onDisconnect(aoDesconectarDoMqtt);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);   
  
  conectaAoWifi();
}

void loop() {
  tempoAtual = millis();    // A cada intervalo de tempo realiza a leitura do DHT
  if (tempoAtual - tempoAnterior >= intevalo) {
    tempoAnterior = tempoAtual; // Atualiza o valor de milisegundos anterior 
    um = dht.readHumidity(); // Realiza uma nova leitura do sensor de umidade
    temp = dht.readTemperature(); // Realiza uma nova leitura do sensor de temperatura, em Celsius
    
    // Publica no MQTT mensagem do tópico esp8266/dht/temperatura
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(temp).c_str());                            
    Serial.printf("Publicando no tópico %s em QoS 1, ID do pacote: %i ", MQTT_PUB_TEMP, packetIdPub1);
    Serial.printf("Mensagem: %.2f \n", temp);

    // Publica no MQTT mensagem do tópico esp8266/dht/umidade
    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM, 1, true, String(um).c_str());                            
    Serial.printf("Publicando no tópico %s em QoS 1, ID do pacote %i: ", MQTT_PUB_HUM, packetIdPub2);
    Serial.printf("Mensagem: %.2f \n", um);
  } 
}


// Rotina que faz a conexão com o Wi-Fi
void conectaAoWifi() {
  Serial.println("Conectando ao Wi-Fi...");
  WiFi.begin(REDE_WIFI, SENHA_WIFI);
}

// Rotina que faz a conexão com o MQTT Broker
void aoConectarAoWifi(const WiFiEventStationModeGotIP& event) {
  Serial.println("Conectado ao Wi-Fi.");
  conectaAoMqtt();
}

// Rotina que faz a desconexão com o Wi-Fi
void aoDesconectarDoWifi(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Desconectado do Wi-Fi.");
  mqttReconnectTimer.detach(); // Garante que não reconectemos ao MQTT enquanto tentamos recontectar com o WI-FI
  wifiReconnectTimer.once(2, conectaAoWifi);
}

// Rotina de conexão do cliente com o MQTT
void conectaAoMqtt() {
  Serial.println("Conectando ao MQTT...");
  mqttClient.connect();
}

// Rotina que sinaliza estabelecimento de seção MQTT
void aoConectarAoMqtt(bool sessionPresent) {
  Serial.println("Conectado ao MQTT.");
  Serial.print("Sessão presente: ");
  Serial.println(sessionPresent);
}

// Rotina que sinaliza a desconexão com o MQTT
void aoDesconectarDoMqtt(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Descontecado do MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, conectaAoMqtt);
  }
}

// Rotina que sinaliza o endereço do pacote MQTT
void onMqttPublish(uint16_t packetId) {
  Serial.print("Publicação reconhecida.");
  Serial.print("  ID do pacote: ");
  Serial.println(packetId);
}
