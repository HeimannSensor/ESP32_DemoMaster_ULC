/*** PROGRAMM INFO***************************************************************************************
  source code for ESP32 and HTPAd Application Shield as demo master for 8x8dULC
  name:           ESP32_8x8dULC_Demomaster.ino
  version/date:   1.0 / 08 Apr 2024
  programmer:     Heimann Sensor GmbH / written by Dennis Pauer & Christoph Kohlmann (pauer@heimannsensor.com, kohlmann@heimannsensor.com)
*********************************************************************************************************/

/*** MODES **********************************************************************************************
  The source code includes three ways to interact with the sensor:
  - via WIFI you can stream thermal images in our GUI (#define WIFIMODE)
  - via the SERIAL monitor you can observe the sensor data as text output (#define SERIALMODE)
  - via ACCESSPOINT the ESP32 creates the wifi network. You have to connect your computer to this network
    to stream thermal images in the GUI (#define ACCESSPOINT)
  Both modes are contained in the same code and you can activate one or both by activate the matching define.
*********************************************************************************************************/
//#define WIFIMODE
#define SERIALMODE
#define ACCESSPOINT

#include "Wire.h"
#include "def.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <pins_arduino.h>
#include "uTimerLib.h"

/*** NETWORK INFORMATION ********************************************************************************
  If you want to use the WIFI function, you have to change ssid and pass.
*********************************************************************************************************/
int8_t ssid[] = "WifiAppsetNetwork";
int8_t pass[] = "heimannsensor";

// WIFI
int32_t status = WL_IDLE_STATUS;
uint32_t localPort = 30444;
char packetBuffer[1024];  // CK added for 32x32 changed from 256 to 1024
uint8_t mac[6];
uint8_t ip_partner[4];
uint8_t device_bind;
WiFiUDP udp;
uint8_t wifi_on = 0;

// LEDs
uint8_t pinLEDred = 33;
uint8_t pinLEDgreen = 25;
uint8_t pinLEDblue = 32;

// read out semaphores
bool readingRoutineEnable = true;
bool checkForNewData = true;
bool sendOnlyActPixel = false;
uint16_t noEOCcounter = 0;

// timer
uint16_t timert = 0;

// sensor data
uint16_t pixel_data[PIXEL_PER_COLUMN * PIXEL_PER_ROW + 2]; // +2 for sensor Tamb
#ifdef HTPA_8x8d
uint8_t pixel_mask[2][PIXEL_PER_COLUMN] = {{0, 0, 0, 0, 0, 0, 0, 0}, {0x01, 0x03, 0x06, 0x0C, 0x18, 0x30, 0x60, 0xC0}}; // [0][..] contains mask read from sensor module, [1][..] contains correct mask on master side
uint8_t nrOfActPixel = 15;
#endif
#ifdef HTPA_16x16d
uint16_t pixel_mask[2][PIXEL_PER_COLUMN] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0x0001, 0x0003, 0x0006, 0x000C, 0x0018, 0x0030, 0x0060, 0x00C0, 0x0100, 0x0300, 0x0600, 0x0C00, 0x1800, 0x3000, 0x6000, 0xC000}}; // [0][..] contains mask read from sensor module, [1][..] contains correct mask on master side
uint16_t nrOfActPixel = 30;
#endif
#ifdef HTPA_32x32d  // CK added for 32x32
uint32_t pixel_mask[2][PIXEL_PER_COLUMN] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                            {0x00000001, 0x00000003, 0x00000006, 0x0000000C, 0x00000018, 0x00000030, 0x00000060, 0x000000C0, 0x00000100, 0x00000300, 0x00000600, 0x00000C00, 0x00001800, 0x00003000, 0x00006000, 0x0000C000,
                                             0x00010000, 0x00030000, 0x00060000, 0x000C0000, 0x00180000, 0x00300000, 0x00600000, 0x00C00000, 0x01000000, 0x03000000, 0x06000000, 0x0C000000, 0x18000000, 0x30000000, 0x60000000, 0xC0000000}}; // [0][..] contains mask read from sensor module, [1][..] contains correct mask on master side
uint16_t nrOfActPixel = 60;
#endif
uint32_t sensorID = 0;
uint8_t epsilon = 100;
uint16_t ta = 0;

// PROGRAMM CONTROL
uint8_t send_data = 0;
uint8_t data_ready = 0;
uint8_t state = 0;
uint8_t print_state = 0;
char serial_input = 'm';

/********************************************************************
   Function:        ISR()
   Description:     interrupt service routine; called by timer periodically to set
                    semaphore to read new data
 *******************************************************************/
void IRAM_ATTR ISR(void) {
  // set semaphore to read new data if not busy via serial interface or already reading data
  if (readingRoutineEnable)
    checkForNewData = true;
    
  /* HINT: this interrupt service routine sets a flag called checkForNewData.
     This flag will be checked in the main loop. If this flag is set, the main loop will call
     the function to read the new data and reset this flag as well as the timer. I go that way
     because the ESP32 cannot read I2C data directly in the ISR. If your µC can handle I2C in
     an interrupt, please read the new sensor volatges direclty in the ISR. */

  return;
}

/********************************************************************
   Function:      Calc_Timert()
   Description:   calculate the duration of the timer which reads the module's data output
 *******************************************************************/
uint16_t Calc_Timert(void) {
  float a = 0.0;
  uint16_t calculated_timer_duration = 0;
  float fclk_float = 12000000.0 / 63.0 * (float)CLK + 1000000.0;  // calc clk in Hz

  a = 32.0 * ((float)pow(2, (uint8_t)(MBIT & 0b00001111)) + 4.0) / fclk_float;  
  a *= ((float)NR_OF_BLOCKS);
  a=0.002;  //Simply ask every two millisec
  calculated_timer_duration = (uint16_t)(0.98 * a * 1000000);     // a in s | timer_duration in µs (including 2% safety factor for fastet readout)

  return calculated_timer_duration;
}

/********************************************************************
   Function:      print_menu()
   Description:   prints serial monitor command options to serial monitor
 *******************************************************************/
void Print_Menu(void) {
  Serial.println("\n\n\n***************************************************");
  Serial.println("16x16dULC Demomaster v1.0               /_/eimann");
  Serial.println("for ESP32-DevkitC-32D                  / /   Sensor");
  Serial.println("\nYou can choose one of these options by sending the \ncharacter\n ");
  Serial.println("  i... get sensor ID");
  Serial.println("  d... set module to DEEPSLEEP");
  Serial.println("  s... set module to SLEEP");
  Serial.println("  w... wake up module");
  Serial.println("  e... read emission");  
  Serial.println("  1... read actual pixel mask");
  Serial.println("  3... set emission to 80%");
  Serial.println("  4... set emission to 100%");
  Serial.println("  5... read end of conversion (EOC)");
  Serial.println("  6... read actual frame");
  Serial.println("  7... set diagonal pixel mask");
  Serial.println("  8... reset pixel mask (all pixel)");
  Serial.println("\t\t\t\t\tver1.0 (ck)");
  Serial.println("***************************************************\n\n\n");

  return;
}

/********************************************************************
   Function:      Set_LED(uint8_t red, uint8_t green, uint8_t blue)
   Description:   switch different colors of LED on/off
 *******************************************************************/
void Set_LED(uint8_t red, uint8_t green, uint8_t blue) {

  digitalWrite(pinLEDred, !red);
  digitalWrite(pinLEDgreen, !green);
  digitalWrite(pinLEDblue, !blue);

  return;
}

/********************************************************************
   Function:        setup()
   Description:     initialize serial, WIFI and 8x8dULC module
 *******************************************************************/
void setup() {

  // serial initialization
  Serial.begin(SERIAL_BAUDRATE);
  while (!Serial);

  // set LED pins as output
  pinMode(pinLEDred, OUTPUT);
  pinMode(pinLEDgreen, OUTPUT);
  pinMode(pinLEDblue, OUTPUT);
  digitalWrite(pinLEDred, HIGH);
  digitalWrite(pinLEDgreen, HIGH);
  digitalWrite(pinLEDblue, HIGH);

  // WIFI initialization
#ifdef WIFIMODE
  wifi_on = 1;
#endif
#ifdef ACCESSPOINT
  wifi_on = 2;
#endif
  if(wifi_on){
    Set_LED(0, 0, 1);
    InitWIFI();
    udp.begin(localPort);
    Set_LED(0, 0, 0);
  }
  
  Wire.begin();
//  Wire.setTimeOut(80);
  Wire.setClock(CLOCK_EEPROM); 

  // timer initialization
  timert = Calc_Timert();
  TimerLib.setInterval_us(ISR, timert);

  // clear pixel data buffer
  memset(pixel_data, 0, sizeof(pixel_data));

  // initialize module
  Startup_Routine_Module();

  // print the menu for the first time
#ifdef SERIALMODE
  Print_Menu();
#endif

  return;
}


/********************************************************************
  Function:        loop()
  Description:     main loop serving communication via WIFI, serial and 8x8dULC module
*******************************************************************/
void loop() {
  // check if the timer interrupt set the checkForNewData flag. If so read the new raw pixel data via I2C
  if (checkForNewData){
    Read_Temp_Frame();
    checkForNewData = false;
  }

  // check for serial input
#ifdef SERIALMODE
  CheckSerial();
#endif

  // check for wifi input
  if(wifi_on)
    CheckUDP();

  // wifi output
  if((send_data)&&(data_ready))
    // send the thermal image as UDP packets to GUI
    SortUDPpacket();

  return;
}

/********************************************************************
   Function:        Write_Module(uint8_t cmd, uint8_t input, uint16_t nrBytes)
   Description:     write nrBytes to sensor module
 *******************************************************************/
void Write_Module(uint8_t cmd, uint8_t* pData, uint16_t nrBytes) {
  uint16_t i = 0;

  Wire.beginTransmission((uint16_t)MODULE_ADDRESS);
  Wire.write(cmd);
  for (i = 0; i < nrBytes; i++)
    Wire.write(*pData++);
  Wire.endTransmission();

  return;
}

/********************************************************************
   Function:        Read_Module(uint8_t cmd, uint8_t* pDest, uint16_t nrBytes)
   Description:     read nrBytes from sensor module
 *******************************************************************/
void Read_Module(uint8_t cmd, uint8_t* pBuffer, uint16_t nrBytes) {
  Wire.beginTransmission((uint16_t)MODULE_ADDRESS);
  Wire.write(cmd);
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)MODULE_ADDRESS, nrBytes, true);
  while (Wire.available()){
    *pBuffer++ = Wire.read();
  }
  return;
}

/********************************************************************
   Function:        Read_Temp_Frame()
   Description:     read all active pixel temperatures as well as sensor's ambient temperature
 *******************************************************************/
void Read_Temp_Frame(void) {
  uint8_t eocByte = 0;
  uint16_t i,j,temp;
  uint8_t setting = 0;
  uint16_t dummyBuffer[PIXEL_PER_COLUMN * PIXEL_PER_ROW + 2]; // +2 for sensor Tamb
  uint16_t nrPixel = 0;
  static uint16_t calls=0;

  readingRoutineEnable = false;
  TimerLib.clearTimer();
  noEOCcounter = 0;
  memset(dummyBuffer, 0, sizeof(dummyBuffer));
  
  // wait for end of conversion bit
  Read_Module((uint8_t)CMD_READEOC, (uint8_t*)&eocByte, 1);
  /*while((bitRead(eocByte, 0) == 0) && (noEOCcounter <= NR_OF_MAX_POLLS)){
    Read_Module(CMD_READEOC, (uint8_t*)&eocByte, 1);
    noEOCcounter++;
  }
  // wake up module in case of missing answer (most likely module is in sleep)
  if(noEOCcounter >= NR_OF_MAX_POLLS){
    Write_Module((uint8_t)CMD_WAKEUP, (uint8_t*)&setting, 0);
    delay(10);
  }
  else{  */
  if(!bitRead(eocByte, 0)){
    calls++;
    if(calls>NR_OF_MAX_POLLS){
      Write_Module((uint8_t)CMD_WAKEUP, (uint8_t*)&setting, 0);
      delay(10);      
      }
    }
else{   
    // read temperature frame from module   
    Read_Module((uint8_t)CMD_READDATA, (uint8_t*)&dummyBuffer, (uint16_t)(nrOfActPixel * 2 + 2)); // +2 for sensor Tamb
    calls=0;
    // sort according to current active-pixel-mask
    for(i = 0; i < PIXEL_PER_COLUMN; i++){
      for(j = 0; j < PIXEL_PER_ROW; j++){
        if((pixel_mask[0][i]>>j)&0x1){
          pixel_data[i*PIXEL_PER_COLUMN+j] = dummyBuffer[nrPixel];
          nrPixel++;
        }
        else
          pixel_data[i*PIXEL_PER_COLUMN+j] = 0;
      }
    }  
    // extract ambient temperature from last transmission
    ta = dummyBuffer[nrOfActPixel];
    data_ready = 1;
  }

  TimerLib.setInterval_us(ISR, timert);
  readingRoutineEnable = true;

  return;
}

/********************************************************************
   Function:        Startup_Routine_Module()
   Description:     initialize 8x8dULC module
 *******************************************************************/
void Startup_Routine_Module(void) {
  uint8_t setting = 0;

  // wake up module
  Write_Module((uint8_t)CMD_WAKEUP, (uint8_t*)&setting, 1);
  delay(10);

  // read active pixel mask from module's nvm
  Read_Module((uint8_t)CMD_READMASK, (uint8_t*)&pixel_mask[0][0], PIXEL_PER_COLUMN * sizeof(pixel_mask[0][0]));
  Read_Pixelmask();
  delay(10);

  // read currently used emissivity
  Read_Module((uint8_t)CMD_READEMI, (uint8_t*)&epsilon, 1);
  if ((epsilon <= 0) || (epsilon > 100)) {
    epsilon = 100;
    Write_Module((uint8_t)CMD_WRITEEMI, (uint8_t*)&epsilon, 1);
  }
  delay(10);

  // read sensor ID
  Read_Module((uint8_t)CMD_READID, (uint8_t*)&sensorID, 4);
  if ((sensorID == 0) || (sensorID >= 0xFFFFFFFF))
    Set_LED(1, 0, 0);
  delay(10);

  return;
}

/********************************************************************
   Function:        Write_Pixelmask()
   Description:     write local active-pixel-mask to 8x8dULD module
 *******************************************************************/
void Write_Pixelmask(void) {
  // write new mask to module (bit-wise)
  Write_Module((uint8_t)CMD_WRITEMASK, (uint8_t*)&pixel_mask[1][0], PIXEL_PER_COLUMN * sizeof(pixel_mask[1][0]));
  delay(500);
  return;
}


/********************************************************************
   Function:        Read_Pixelmask()
   Description:     get the current pixel mask from the slave
 *******************************************************************/
void Read_Pixelmask(void) {  
    unsigned char i,k;
    uint8_t temp;
    
    Read_Module((uint8_t)CMD_READMASK, (uint8_t*)&pixel_mask[0][0], PIXEL_PER_COLUMN * sizeof(pixel_mask[0][0]));
    nrOfActPixel=0;

    for(i=0;i<PIXEL_PER_COLUMN;i++){
      for(k=0;k<PIXEL_PER_COLUMN;k++){
        Serial.print((pixel_mask[0][i]>>((PIXEL_PER_COLUMN - 1)-k))&0x1);
        if((pixel_mask[0][i]>>((PIXEL_PER_COLUMN - 1)-k))&0x1)
          nrOfActPixel++;
      }
      Serial.print("\r\n");
    }
    Serial.print("\r\nActivePixel: ");
    Serial.print(nrOfActPixel);
    Serial.print("\r\n");    
}

/********************************************************************
   Function:        void Set_Pixel_Mask()
   Description:     get new active-pixel-mask via udp
 *******************************************************************/
void Set_Pixel_Mask(void) {
  uint8_t i = 0;
  uint8_t j = 0;

  nrOfActPixel = 0;
  send_data = 0;
  TimerLib.clearTimer();

  for (i = 0; i < PIXEL_PER_COLUMN; i++)
    #ifdef HTPA_8x8d
    pixel_mask[1][i] = (uint8_t)(packetBuffer[(PIXEL_PER_COLUMN * 2 + 1) + 2 * i] - '0'); // expecting comma separated bytes reprenting pixels per bit (1 = activ, 0 = inactive pixel)
    #endif
    #ifdef HTPA_16x16d
    pixel_mask[1][i] = (uint16_t)(packetBuffer[(PIXEL_PER_COLUMN * 2 + 1) + 2 * i] - '0'); // expecting comma separated bytes reprenting pixels per bit (1 = activ, 0 = inactive pixel)
    #endif
    #ifdef HTPA_32x32d    // CK added for 32x32
    pixel_mask[1][i] = (uint32_t)(packetBuffer[(PIXEL_PER_COLUMN * 2 + 1) + 2 * i] - '0'); // expecting comma separated bytes reprenting pixels per bit (1 = activ, 0 = inactive pixel)
    #endif

  // retransform bit-wise encoding to local byte-wise handling
  for (i = 0; i < PIXEL_PER_COLUMN; i++) {
    if (pixel_mask[1][i]){
        for(j = 0; j < PIXEL_PER_COLUMN; j++){
          // summing up new amount of active pixels
          nrOfActPixel += ((pixel_mask[1][i] >> j) & 0x1);
        }
    }
  }
  // write new mask to module (bit-wise)
  Write_Module((uint8_t)CMD_WRITEMASK, (uint8_t*)&pixel_mask[1][0], PIXEL_PER_COLUMN * sizeof(pixel_mask[1][0]));

  delay(1000);
  TimerLib.setInterval_us(ISR, timert);

  return;
}

/********************************************************************
   Function:        SortUDPpacket()
 *******************************************************************/
void SortUDPpacket(void) {
  uint8_t packet1[UDP_PACKET_LENGTH];
  int32_t bytei = 0, packetnumber = 0;
  uint16_t i,k;


  for (i = 0; i < PIXEL_PER_COLUMN * PIXEL_PER_ROW; i++) {
    packet1[bytei++] = (pixel_data[i] & 0x00ff);
    packet1[bytei++] = (pixel_data[i] & 0xff00) >> 8;

    if(bytei == UDP_PACKET_LENGTH){
      udp.beginPacket(ip_partner, udp.remotePort());
      udp.write(packet1, UDP_PACKET_LENGTH);
      udp.endPacket();
      bytei = 0;
      packetnumber++;
    }
  }

  // HS GUI expects these data, since usually elOffs are submitted
  for (i = 0; i < (PIXEL_PER_COLUMN * PIXEL_PER_ROW / NR_OF_BLOCKS); i++) {
    packet1[bytei++] = 0;
    packet1[bytei++] = 0;
    
    if(bytei == UDP_PACKET_LENGTH){
      udp.beginPacket(ip_partner, udp.remotePort());
      udp.write(packet1, UDP_PACKET_LENGTH);
      udp.endPacket();
      bytei = 1;
      packetnumber++;
    }
  }

  packet1[bytei++] = 0;                         //VDD
  packet1[bytei++] = 0;

  packet1[bytei++] = (ta & 0x00ff);             // TA
  packet1[bytei++] = (ta & 0xff00) >> 8;

  for (i = 0; i < NR_OF_BLOCKS * 2; i++) {      // PTAT
    packet1[bytei++] = 0;
    packet1[bytei++] = 0;
  }

  udp.beginPacket(ip_partner, udp.remotePort());
  udp.write(packet1, LAST_UDP_PACKET_LENGTH);
  udp.endPacket();
  data_ready=0;

  return;
}

/********************************************************************
   Function:        InitWIFI()
   Description:     initialize wifi interface
 *******************************************************************/
void InitWIFI(void) {
#ifdef WIFIMODE
  while (status != WL_CONNECTED) {
    Serial.print("\nstart connection to network:");
    Serial.print(ssid);
    WiFi.mode(WIFI_STA);
    status = WiFi.begin(ssid, pass);
    delay(5000);
  }
  Serial.println("\n\nWIFI setup:");
  Serial.print("network:\t\t");
  Serial.println(WiFi.SSID());
  Serial.print("IP Address:\t\t");
  Serial.println(WiFi.localIP());
  Serial.print("signal strength :\t");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
#endif

#ifdef ACCESSPOINT
  const char* ssid1     = ACCESSPOINTNAME;
  const char* password1 = ACCESSPOINTKEY;

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid1, password1);;
  Serial.println("\n\naccess point setup:");
  Serial.print("name:\t\t");
  Serial.print(ACCESSPOINTNAME);
  Serial.print("\nIP Address:\t");
  Serial.println(WiFi.softAPIP());
#endif

  return;
}

/********************************************************************
   Function:        checkUDP()
   Description:     check udp input and call according function as response
 *******************************************************************/
void CheckUDP(void) {
  int32_t packetSize = udp.parsePacket();
  uint8_t packetChangeEPSILON[] = {"Set Emission to "};
  uint8_t packetChangePIXMASK[] = {"Set Pixelmask to "};
  bool change_epsilon = true;
  bool set_mask = true;
  int32_t len = 0;
  uint8_t i = 0;

  if (packetSize) {
    if ((ip_partner[0] == udp.remoteIP()[0] &&
         ip_partner[1] == udp.remoteIP()[1] &&
         ip_partner[2] == udp.remoteIP()[2] &&
         ip_partner[3] == udp.remoteIP()[3]) || device_bind == 0) {
      len = udp.read(packetBuffer, 255);
      if (len > 0)
        packetBuffer[len] = 0;
    }
    else {
      udp.read(packetBuffer, 255);
      return;
    }

    // HTPA RESPONSED
    if (strcmp(packetBuffer, "Calling HTPA series devices") == 0)
      Udp_Respond_GUI_Calling();

    // SEND IP AND MAC (HW FILTER)
    else if (strcmp(packetBuffer, "Bind HTPA series device") == 0)
      Udp_Respond_GUI_Binding();

    // USER SETTING
    else if (strcmp(packetBuffer, "G") == 0)
      Udp_Respond_GUI_User_Settings();

    // SEND DATA (TEMPS)
    else if (strcmp(packetBuffer, "K") == 0)
      send_data = 1;

    // STOP SENDING
    else if (strcmp(packetBuffer, "x") == 0)
      send_data = 0;

    // HW RELEASED
    else if (strcmp(packetBuffer, "x Release HTPA series device") == 0)
      Udp_Respond_GUI_Releasing();

    // CHANGE EPSILON
    else if (packetSize > sizeof(packetChangeEPSILON)) { // use smaller one of sizeof(packetChangeEPSILON) and sizeof(packetChangePIXMASK)
      for (i = 0; i < sizeof(packetChangeEPSILON); i++) {
        if (packetBuffer[i] != packetChangeEPSILON[i]) {
          change_epsilon = false;
          break;
        }
      }
      if (change_epsilon)
        Udp_Respond_GUI_ChangeEmissivity();
      else {
        for (i = 0; i < sizeof(packetChangePIXMASK); i++) {
          if (packetBuffer[i] != packetChangePIXMASK[i]) {
            set_mask = false;
            break;
          }
        }
        if (set_mask)
          Set_Pixel_Mask();
      }
    }
  }

  return;
}

/********************************************************************
   Function:        void Udp_Respond_GUI_Calling(void)
 *******************************************************************/
void Udp_Respond_GUI_Calling(void) {
  uint8_t i = 0;

  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  #ifdef HTPA_8x8d
  udp.print("HTPA series responsed! I am Arraytype 00");
  #endif
  #ifdef HTPA_16x16d
  udp.print("HTPA series responsed! I am Arraytype 01");
  #endif  
  #ifdef HTPA_32x32d  // BF added for 32x32
  udp.print("HTPA series responsed! I am Arraytype 10");
  #endif    
  udp.print(" MODTYPE 005\r\nADC: ");
  udp.print( (MBIT & 15) + 4);    // calc ADC resolution
  udp.print("\r\n");
  #ifdef HTPA_8x8d
  udp.print("HTPA8x8d v.2.00 Heimann Sensor GmbH; written by D. Pauer 2021-01-13\r\n");
  #endif
  #ifdef HTPA_16x16d
  udp.print("HTPA16x16d v.2.00 Heimann Sensor GmbH; written by D. Pauer 2021-01-13\r\n");
  #endif
  #ifdef HTPA_32x32d  // CK added for 32x32
  udp.print("HTPA32x32d v.2.00 Heimann Sensor GmbH; written by D. Pauer 2021-01-13\r\n");
  #endif
  udp.print("I am running on ");
  float clk_float = 12000000 / 63 * CLK + 1000000;    // calc clk in MHz
  udp.print(clk_float / 1000, 1); // print clk in kHz
  udp.print(" kHz\r\n");

  udp.print("MAC-ID: ");
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 0x10)
      udp.print("0");
    udp.print(mac[i], HEX);
    if (i < 5)
      udp.print(".");
  }

  udp.print(" IP: ");
  for (i = 0; i < 4; i++) {
    if (WiFi.localIP()[i] < 10)
      udp.print("00");
    else if (WiFi.localIP()[i] < 100)
      udp.print("0");
    udp.print(WiFi.localIP()[i]);
    if (i < 3)
      udp.print(".");
  }

  udp.print(" DevID: ");
  for (i = 1; i < 9; i++) {
    if (sensorID < pow(10, i))
      udp.print("0");
  }
  udp.print(sensorID);
  udp.endPacket();

  return;
}

/********************************************************************
   Function:        void Udp_Respond_GUI_Binding(void)
 *******************************************************************/
void Udp_Respond_GUI_Binding(void) {
  uint8_t i = 0;

  if (device_bind == 0) {
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.print("HW Filter is ");
    for (i = 0; i < 4; i++) {
      if (WiFi.localIP()[i] < 10)
        udp.print("00");
      else if (WiFi.localIP()[i] < 100)
        udp.print("0");
      udp.print(WiFi.localIP()[i]);
      if (i < 3)
        udp.print(".");
    }
    udp.print(" MAC ");
    for (i = 0; i < 6; i++) {
      if (mac[i] < 0x10)
        udp.print("0");
      udp.print(mac[i], HEX);
      if (i < 5)
        udp.print(".");
    }
    udp.print("\n\r");
    udp.endPacket();

    device_bind = 1;
    for (i = 0; i < 4; i++)
      ip_partner[i] = udp.remoteIP()[i];
  }
  else {
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.print("Device already bound\n\r");
    udp.endPacket();
  }

  return;
}

/********************************************************************
   Function:        void Udp_Respond_GUI_User_Settings(void)
 *******************************************************************/
void Udp_Respond_GUI_User_Settings(void) {
  if (device_bind == 0)
    return;

  udp.beginPacket(ip_partner, udp.remotePort());
  #ifdef HTPA_8x8d
  udp.print("HTPA8x8d 2021/01/13 v.2.00 Heimann Sensor GmbH; written by D. Pauer\n\r");
  #endif
  // CK added for 32x32
  #ifdef HTPA_16x16d
  udp.print("HTPA16x16d 2021/01/13 v.2.00 Heimann Sensor GmbH; written by D. Pauer\n\r");
  #endif
  #ifdef HTPA_32x32d
  udp.print("HTPA32x32d 2021/01/13 v.2.00 Heimann Sensor GmbH; written by D. Pauer\n\r");
  #endif
  udp.print("BIAS: ");
  if (BIAS < 0x10)
    udp.print("0");
  udp.print(BIAS, HEX);
  udp.print("Clock: ");
  if (CLK < 0x10)
    udp.print("0");
  udp.print(CLK, HEX);
  udp.print("MBIT: ");
  if (MBIT < 0x10)
    udp.print("0");
  udp.print(MBIT, HEX);
  udp.print("BPA: ");
  if (BPA < 0x10)
    udp.print("0");
  udp.print(BPA, HEX);
  udp.print("PU: ");
  if (PU < 0x10)
    udp.print("0");
  udp.print(PU, HEX);
  udp.print("GlobalOffset: ");
  udp.print(0, HEX);
  udp.print("GlobalGain: ");
  udp.print(10000, HEX);
  udp.endPacket();

  return;
}

/********************************************************************
   Function:        void Udp_Respond_GUI_Releasing(void)
 *******************************************************************/
void Udp_Respond_GUI_Releasing(void) {
  uint8_t i = 0;

  send_data = 0;
  device_bind = 0;
  for (i = 0; i < 4; i++) {
    ip_partner[i] = 0;
  }
  udp.beginPacket(ip_partner, udp.remotePort());
  udp.print("HW-Filter released\r\n");
  udp.endPacket();

  return;
}

/********************************************************************
   Function:        void Udp_Respond_GUI_ChangeEmissivity(void)
 *******************************************************************/
void Udp_Respond_GUI_ChangeEmissivity(void) {
  send_data = 0;
  TimerLib.clearTimer();
  epsilon = (uint8_t)(packetBuffer[16] - '0') * 100 + (int32_t)(packetBuffer[17] - '0') * 10 + (int32_t)(packetBuffer[18] - '0');

  // write new epsilon to module
  if ((epsilon > 0) && (epsilon <= 100))
    Write_Module((uint8_t)CMD_WRITEEMI, (uint8_t*)&epsilon, 1);

  udp.beginPacket(ip_partner, udp.remotePort());
  udp.print("Emission changed to ");
  udp.print(epsilon);
  udp.print("%\r\n\r\n");
  udp.endPacket();
  delay(1000);
  TimerLib.setInterval_us(ISR, timert);

  return;
}

/********************************************************************
   Function:        checkSerial()
   Description:
 *******************************************************************/
void CheckSerial(void) {
  uint8_t setting = 0;
  uint16_t i,k;
  serial_input = Serial.read();

  switch (serial_input) {
    case 0xFF:
      //nothing
      break;
    case 'm':
      while (state);
      readingRoutineEnable = false;
      Print_Menu();
      readingRoutineEnable = true;
      break;

    case 'i':
      readingRoutineEnable = false;
      Read_Module((uint8_t)CMD_READID, (uint8_t*)&sensorID, 4);
      Serial.println(sensorID);
      readingRoutineEnable = true;
      break;

    case 'd':
      readingRoutineEnable = false;
      TimerLib.clearTimer();
      // wake up module
      Write_Module((uint8_t)CMD_DEEPSLEEP, (uint8_t*)&setting, 0);
      delay(10);
      Serial.println("Deep sleeping");
      break;      

    case 's':
      readingRoutineEnable = false;
      TimerLib.clearTimer();
      // wake up module
      Write_Module((uint8_t)CMD_SLEEP, (uint8_t*)&setting, 0);
      delay(10);
      Serial.println("Sleeping");
      break;

    case 'w':
      readingRoutineEnable = false;
      TimerLib.clearTimer();
      // wake up module
      Write_Module((uint8_t)CMD_WAKEUP, (uint8_t*)&setting, 0);
      delay(10);
      Serial.println("Awoken");
      readingRoutineEnable = true;
      TimerLib.setInterval_us(ISR, timert);
      break;

    case '1':
      readingRoutineEnable = false;
      TimerLib.clearTimer();
      Read_Pixelmask();
      readingRoutineEnable = true;
      TimerLib.setInterval_us(ISR, timert);
      break;

    case 'e':
      readingRoutineEnable = false;
      TimerLib.clearTimer();
      Read_Module((uint8_t)CMD_READEMI, (uint8_t*)&epsilon,1);
      Serial.println(epsilon);
      readingRoutineEnable = true;
      TimerLib.setInterval_us(ISR, timert);
      break;
      
    case '3':
      readingRoutineEnable = false;
      epsilon=80;
      TimerLib.clearTimer();
      Write_Module((uint8_t)CMD_WRITEEMI, (uint8_t*)&epsilon,1);
      delay(250);
      Serial.println(epsilon);
      readingRoutineEnable = true;
      TimerLib.setInterval_us(ISR, timert);
      break; 
       
    case '4':
      readingRoutineEnable = false;
      epsilon=100;
      TimerLib.clearTimer();
      Write_Module((uint8_t)CMD_WRITEEMI, (uint8_t*)&epsilon,1);
      delay(250);
      Serial.println(epsilon);
      readingRoutineEnable = true;
      TimerLib.setInterval_us(ISR, timert);
      break;    
            
    case '5':
      readingRoutineEnable = false;
      TimerLib.clearTimer();
      Read_Module((uint8_t)CMD_READEOC, (uint8_t*)&i,1);
      i&=0xFF;
      Serial.print("EOC is: ");
      Serial.println(i);
      readingRoutineEnable = true;
      TimerLib.setInterval_us(ISR, timert);
      break;        

    case '6':   //make sure don't call Read_temp_frame without starting the sensor first
      readingRoutineEnable = false;
      TimerLib.clearTimer();
      nrOfActPixel=PIXEL_PER_COLUMN * PIXEL_PER_ROW;
      Read_Temp_Frame();
      Serial.print("-------TA was: ");
      Serial.println(((float)ta/10.0)-ABSNULL);
      Serial.println("Frame content:-----------------------");
      for(i=0;i<nrOfActPixel;i++){
        Serial.print(((float)pixel_data[i]/10.0)-ABSNULL);
        Serial.print(" ");
        // CK added for 32x32 (changed from: "if((i%16)==15)")
        if((i % PIXEL_PER_ROW) == (PIXEL_PER_ROW - 1))
          Serial.print("\r\n");
          
      }
      readingRoutineEnable = true;
      TimerLib.setInterval_us(ISR, timert);
      break;

    case '7':  //write pixel mask to diagonal
      readingRoutineEnable = false;
      TimerLib.clearTimer();
      Write_Pixelmask();
      delay(120);
      Read_Pixelmask();
      readingRoutineEnable = true;
      TimerLib.setInterval_us(ISR, timert);
      break;      

    case '8':  //reset pixel mask to all pixel
      readingRoutineEnable = false;
      TimerLib.clearTimer();
    #ifdef HTPA_8x8d
      memset((void*)&pixel_mask[0],0xFF,PIXEL_PER_COLUMN);
      #endif
      #ifdef HTPA_16x16d
      memset((void*)&pixel_mask[0],0xFF,PIXEL_PER_COLUMN*2);
      #endif
      #ifdef HTPA_32x32d // CK added for 32x32
      memset((void*)&pixel_mask[0],0xFF,PIXEL_PER_COLUMN*4);
      #endif
      Write_Module((uint8_t)CMD_WRITEMASK, (uint8_t*)&pixel_mask[0], PIXEL_PER_COLUMN * sizeof(pixel_mask[1][0]));
      delay(120);
      Read_Pixelmask();
      readingRoutineEnable = true;
      TimerLib.setInterval_us(ISR, timert);
      break;      
  }      

  return;
}

/********************************************************************
   Function:        print_final_array()
   Description:
 *******************************************************************/
void Print_Final_Array(void) {
  uint8_t i = 0;

  Serial.println("\n\n---pixel data ---");
  for (i = 0; i < nrOfActPixel; i++) {
    Serial.print(pixel_data[i]);
    Serial.print("\t");
  }
  Serial.println("");

  return;
}
