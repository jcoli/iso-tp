// demo: CAN-BUS Shield, send data
// loovee@seeed.cc

#include <mcp_can.h>
#include <SPI.h>
#include <mcp_can_dfs.h>
#include <iso-tp.h>

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;
#define MCP_INT 2

bool MILflag = 0; // Flag: 0 = MIL Off / 1 = MIL On
bool DTCflag = 0; // Flag indicating that there is at least one DTC in memory

int dtcNumbers = 0; // Flag indicating that there is at least one DTC in memory

bool DTC0flag = LOW; // Flag indicating that there is at least one DTC in memory
bool DTC1flag = LOW; // Flag indicating that there is at least one DTC in memory
bool DTC2flag = LOW; // Flag indicating that there is at least one DTC in memory
bool DTC3flag = LOW; // Flag indicating that there is at least one DTC in memory
bool DTC4flag = LOW; // Flag indicating that there is at least one DTC in memory
bool DTC5flag = LOW; // Flag indicating that there is at least one DTC in memory
bool DTC6flag = LOW; // Flag indicating that there is at least one DTC in memory
bool DTC7flag = LOW; // Flag indicating that there is at least one DTC in memory

unsigned long start = millis();

unsigned char len = 0; // Length of the data received
unsigned char buf[8]; // Message storage buffer OBD-II
String BuildMessage = ""; // String to print the received message
int MODE = 0; // OBD-II operating mode
int PID = 0; // PParameters IDs of Mode 1
int operMode = 0; // operating mode 0=normal(read analog inputs) 1=auto random 2=simulating normal car operation



MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin
IsoTp isotp(&CAN, MCP_INT);

struct Message_t TxMsg;
struct Message_t RxMsg;
uint8_t sf_test[] = { 0x00, 0x01 };
uint8_t mf_test[] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, \
                      0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F
                    };
uint32_t can_id = 0x7E8;
uint32_t rx_id = 0x705;



void setup()
{
  Serial.begin(115200);

  while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
  {
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
    delay(100);
  }
  Serial.println("CAN BUS Shield init ok!");

  TxMsg.Buffer = (uint8_t *)calloc(MAX_MSGBUF, sizeof(uint8_t));
  RxMsg.Buffer = (uint8_t *)calloc(MAX_MSGBUF, sizeof(uint8_t));

  RxMsg.tx_id = 0;
}

unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};

unsigned char stmp2[7] = {0, 0, 0, 0, 0, 0, 0};
void loop()
{
  if (millis() - start > 500000) {
    start = millis();
    // send data:  id = 0x00, standrad frame, data len = 8, stmp: data buf
    stmp[7] = stmp[7] + 1;
    if (stmp[7] == 100)
    {
      stmp[7] = 0;
      stmp[6] = stmp[6] + 1;

      if (stmp[6] == 100)
      {
        stmp[6] = 0;
        stmp[5] = stmp[6] + 1;
      }
    }
    stmp2[6] = stmp2[6] + 1;
    if (stmp2[6] == 100)
    {
      stmp2[6] = 0;
      stmp2[5] = stmp2[5] + 1;

      if (stmp2[5] == 100)
      {
        stmp2[5] = 0;
        stmp2[6] = 0;
      }
    }
    Serial.println("send");
    TxMsg.len = sizeof(stmp2);
    TxMsg.tx_id = can_id;
    TxMsg.rx_id = can_id + 0x20;
    //memcpy(TxMsg.Buffer, sf_test, sizeof(sf_test));
    memcpy(TxMsg.Buffer, stmp2, sizeof(stmp2));
    Serial.println(F("Send..."));
    isotp.send(&TxMsg);

  }
  //delay(100);
  //Serial.println(F("Send..."));
  //Serial.println(stmp)
  //CAN.sendMsgBuf(0x09, 0, 8, stmp);
  //delay(2000);                       // send data per 100ms


  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    MODE = 0;
    PID = 0;

    readMessage();

    Serial.print("Mode :");
    Serial.println(MODE);
    Serial.print("PID :");
    Serial.println(PID);
  }
}

String readMessage() {


  Serial.println("ReadMessage");

  RxMsg.tx_id = can_id;
  RxMsg.rx_id = rx_id;
  Serial.println(F("Receive..."));
  isotp.receive(&RxMsg);
  
  len=RxMsg.len;
  
  //can_id = RxMsg.rx_id; // Message identifier
  Serial.print("Id: ");
  Serial.println(RxMsg.rx_id);
  Serial.print("Len: ");
  Serial.println(RxMsg.len);
  //Serial.println("Buffer: ");
  //Serial.println(RxMsg.Buffer);  

  String Message = "";
  //CAN.readMsgBuf(&len, buf); // We store the OBD-II message in buf and its length in len
  // Print the address Id
  
  //Message= RxMsg.Buffer;
  Serial.print("CanId: ");
  Serial.println(can_id);
  Serial.print("Len: ");
  Serial.println(len);
  isotp.print_buffer(RxMsg.rx_id, RxMsg.Buffer, RxMsg.len);
  

  // Build the message and print it
  for (int i = 0; i < len; i++)
  {
    if ( i == len - 1 ) {
      Message = Message + RxMsg.Buffer[i];
      Serial.println(RxMsg.Buffer[i]);
    }
    else {
      Serial.println(RxMsg.Buffer[i]);
      Message = Message + RxMsg.Buffer[i] + ",";

    }
  }
  Serial.println("Message: " + Message);
  // We keep the mode and the PID required
  if ( RxMsg.Buffer != 0 ) { // We check that the message is not empty
    MODE = RxMsg.Buffer[0];
    Serial.print("Mode: "); Serial.println(MODE);
    // If you work in mode one we keep the PID
    if (MODE == 1) {
      PID = RxMsg.Buffer[1];
      Serial.print("PID: "); Serial.println(PID);
    }
  }
  return Message;
}

void replyMode03() {
  // Predefined OBD-II message to respond in mode 1
  // {len data, 03+40hex, num DTCs, DTC1H, DTC1L, DTC2H, DTC2L}
  // {len data, 03+40hex, num DTCs, DTC1H, DTC1L, DTC2H, DTC2L, DTC3H, DTC3L }


  unsigned char OBDIImsg[7] = {6, 67, 0, 0, 0, 0, 0}; // Inicializamos a 0 no hay DTCs
  if (DTCflag) {
    if (dtcNumbers <= 2) {
      if (DTC0flag) {

        OBDIImsg[2] = dtcNumbers; // amount of DTCs
        OBDIImsg[3] = 2; // P0217 Engine overheating HIGH
        OBDIImsg[4] = 23; // P0217 Engine overheating LOW
        //CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg);
        Serial.println("Mode 3 flag 0");
      }
      if (DTC1flag ) {

        OBDIImsg[2] = dtcNumbers; // amount of DTCs
        OBDIImsg[5] = 1; // P0217 Engine overheating HIGH
        OBDIImsg[6] = 22; // P0217 Engine overheating LOW
        //CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg);
        Serial.println("Mode 3 flag 1: ");


      }
      if (DTC2flag ) {

        OBDIImsg[2] = dtcNumbers; // amount of DTCs
        OBDIImsg[5] = 2; // P0217 Engine overheating HIGH
        OBDIImsg[6] = 2; // P0217 Engine overheating LOW
        //CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg);
        //Serial.println("Mode 3 flag 2");

      }
      //Serial.print("Mode 3: ");


      //Serial.println("=========SEND============");
      Serial.println(CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg));
    }
  }  else  {
    //Serial.println("=========SEND============");
    //Serial.println(CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg));
  }

  //  for (int i = 0; i < 10; i++) {
  //    Serial.print(OBDIImsg[i]);
  //    Serial.print(":");
  //  }
  //  Serial.println("");
  //  Serial.println("=======");



}


// END FILE
