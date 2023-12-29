#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>

// SX1278 pins
#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND    868E6 // Europe freq

unsigned long lastAction = 0;
int actionInterval = 3000;

uint8_t deviceAddress = 0xA1;
uint8_t echoAddress = 0xEE;
const uint8_t broadcastAddress = 0xFF; 

int msgCount = 0;

// prototypes
void initLora();
void onReceive(int packetSize);
void sendMessage(uint8_t to, uint8_t msgId, String msg);
void printError(String msg);


void setup()
{
    // Sanity check delay
    delay(2000);

    Serial.begin(115200);
    while(!Serial);
    Serial.println();
    Serial.println("[init] Serial... Ok");

    initLora();

    LoRa.receive();

    Serial.println("[init] done.");    
}

void loop()
{
    if(millis() - lastAction > actionInterval) {
        lastAction = millis();

        sendMessage(echoAddress, msgCount, "abc");
    }

    int packetSize = LoRa.parsePacket();
    if(packetSize)
        onReceive(packetSize);

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
void onReceive(int packetSize)
{
    if (packetSize == 0) return;          // if there's no packet, return

    // packet structure
    // from address
    // to address
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
    uint8_t msgId = LoRa.read();
    uint8_t msgLength = LoRa.read();
    // payload
    String msg = "";
    while(LoRa.available()) {
        msg += (char)LoRa.read();
    }

    if(msgLength != msg.length())
        printError("message length does not match.");

    // if message is for this device, or broadcast, print details:
    Serial.println("Received from: 0x" + String(fromAdr, HEX));
    Serial.println("Sent to: 0x" + String(toAdr, HEX));
    Serial.println("Message ID: " + String(msgId));
    Serial.println("Message length: " + String(msgLength));
    Serial.println("Message: " + msg);
    Serial.println("RSSI: " + String(LoRa.packetRssi()));
    Serial.println("Snr: " + String(LoRa.packetSnr()));
    Serial.println();
}

/**
 * 
 */
void sendMessage(uint8_t to, uint8_t msgId, String msg)
{
    LoRa.beginPacket();         // start packet

    LoRa.write(to);             // add destination address
    LoRa.write(deviceAddress);  // add sender address
    LoRa.write(msgId);          // add message ID
    LoRa.write(msg.length());   // add payload length
    LoRa.print(msg);            // add payload
    
    LoRa.endPacket(true);       // finish packet and send it 

    msgCount++;
    Serial.println("[sendMessage] to 0x"+ String(to, HEX));
    LoRa.receive();
}

/**
 * 
 */
void printError(String msg)
{
    Serial.println("[error] "+ msg);
}