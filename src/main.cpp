#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <TinyGPSPlus.h>


// SX1278 pins
#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND    868E6 // Europe freq

TinyGPSPlus gps;
HardwareSerial GPSSerial(1); 

unsigned long lastAction = 0;
int actionInterval = 3000;

uint8_t deviceAddress = 0xA1;
uint8_t echoAddress = 0xEE;
const uint8_t broadcastAddress = 0xFF; 

int msgCount = 0;
int echoCount = 0;
int packetLossCounter = 0;
int lastPacketId = msgCount;

// prototypes
void initLora();
void sendMessage(uint8_t to, String msg);
void onReceive(int packetSize);
void printError(String msg);


void setup()
{
    // Sanity check delay
    delay(2000);

    Serial.begin(115200);
    while(!Serial);
    Serial.println();
    Serial.println("[init] Serial... Ok");
    GPSSerial.begin(9600, SERIAL_8N1, 34, 12);

    initLora();

    LoRa.receive();

    Serial.println("[init] done.");    
}

void loop()
{
    if(millis() - lastAction > actionInterval) {
        lastAction = millis(); 

        if(GPSSerial.available())
            gps.encode(GPSSerial.read());

        sendMessage(echoAddress, "abc");   
    }

    int packetSize = LoRa.parsePacket();
    if(packetSize)
        onReceive(packetSize);

    // Serial.print("Latitude  : ");
    // Serial.println(gps.location.lat(), 5);
    // Serial.print("Longitude : ");
    // Serial.println(gps.location.lng(), 4);
    // Serial.print("Satellites: ");
    // Serial.println(gps.satellites.value());
    // Serial.print("Altitude  : ");
    // Serial.print(gps.altitude.feet() / 3.2808);
    // Serial.println("M");
    // Serial.print("Time      : ");
    // Serial.print(gps.time.hour());
    // Serial.print(":");
    // Serial.print(gps.time.minute());
    // Serial.print(":");
    // Serial.println(gps.time.second());
    // Serial.println("**********************");

    if(millis() > 5000 && gps.charsProcessed() < 10)
        Serial.println(F("No GPS data received: check wiring"));
}

/**
 * 
 */
void initLora()
{
    Serial.print("[init] LoRa... ");
    SPI.begin(SCK, MISO, MOSI, SS);
    LoRa.setPins(SS, RST, DI0);

    if (!LoRa.begin(BAND)) {
        Serial.println("failed!");
        while(1);
    }
    Serial.println("Ok");
}

/**
 * 
 */
void sendMessage(uint8_t to, String msg)
{
    double latitude = gps.location.lat();
    double longitude = gps.location.lng();
    uint8_t buffer[16]; // 8 bytes for latitude, 8 bytes for longitude
    memcpy(buffer, &latitude, sizeof(double));
    memcpy(buffer + 8, &longitude, sizeof(double));

    LoRa.beginPacket();         // start packet

    LoRa.write(to);                     // add destination address
    LoRa.write(deviceAddress);          // add sender address
    LoRa.write(buffer, sizeof(buffer)); // add location (latitude and longitude)
    LoRa.write(msgCount);               // add message ID
    LoRa.write(msg.length());           // add payload length
    LoRa.print(msg);                    // add payload
    
    LoRa.endPacket(true);       // finish packet and send it 

    Serial.println("[sendMessage] to 0x"+ String(to, HEX) +" - msgId: "+ msgCount);
    Serial.println("[sendMessage] packet loss: "+ String(msgCount - echoCount));

    msgCount++;
    LoRa.receive();
}

/**
 * 
 */
void onReceive(int packetSize)
{
    if (packetSize == 0) return;          // if there's no packet, return

    // packet structure
    // from address
    // to address
    // location lat (from latitude)
    // location lng (from longitude)
    // message id
    // message length - required since cb is used
    // message

    // read incoming packet
    // header
    uint8_t toAdr = LoRa.read();
    
    // return as early as possible
    if (toAdr != deviceAddress && toAdr != broadcastAddress)
        return;

    uint8_t fromAdr = LoRa.read();
    
    double latitude, longitude;
    LoRa.readBytes((uint8_t*)&latitude, sizeof(latitude));
    LoRa.readBytes((uint8_t*)&longitude, sizeof(longitude));

    uint8_t msgId = LoRa.read();
    uint8_t msgLength = LoRa.read();
    // payload
    String msg = "";
    while(LoRa.available()) {
        msg += (char)LoRa.read();
    }

    if(msgLength != msg.length())
        printError("message length does not match.");

    // if(msgId > lastPacketId+1)
        // packetLossCounter += msgId - lastPacketId - 1; // get number of lost packets

    // lastPacketId = msgId;
    echoCount++;


    // if message is for this device, or broadcast, print details:
    Serial.println("Received from: 0x"+ String(fromAdr, HEX));
    Serial.println("Sent to: 0x"+ String(toAdr, HEX));
    Serial.println("Latitude: "+ String(latitude, 6));
    Serial.println("Longitude: "+ String(longitude, 6));
    Serial.println("Message ID: "+ String(msgId));
    Serial.println("Message length: "+ String(msgLength));
    Serial.println("Message: "+ msg);
    Serial.println("RSSI: "+ String(LoRa.packetRssi()));
    Serial.println("Snr: "+ String(LoRa.packetSnr()));
    Serial.println();
}

/**
 * 
 */
void printError(String msg)
{
    Serial.println("[error] "+ msg);
}