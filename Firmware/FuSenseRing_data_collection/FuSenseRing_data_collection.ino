/*
  FuSenseRing Data Collection: Output PPG, IMU, Temperature, force sensor readings.
  By: XXXX
  Date: July 7th, 2025

  This code is released under the [MIT License](http://opensource.org/licenses/MIT).
*/

#include <Wire.h>
#include "MAX30105.h"
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <Adafruit_TMP117.h>
#include <Adafruit_Sensor.h>
// Liraries for Json
#include "ArduinoJson.h"

#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
Adafruit_TMP117  tmp117;

MAX30105 particleSensor;
BLEDis bledis;
BLEUart bleuart;

uint32_t rxCount = 0;
uint32_t rxStartTime = 0;
uint32_t rxLastTime = 0;
const int FSR_PIN = A3; // Pin connected to FSR/resistor divider

// Measure the voltage at 5V and resistance of your 3.3k resistor, and enter
// their value's below:
const float VCC = 3.3; // Measured voltage of Ardunio 5V line
const float R_DIV = 3230; // Measured resistance of 3.3k resistor


long send_time = millis();
int total_num_data = 16;

float ALL_MAX[16] = {
  -1, -1, -1,
  -1, -1, -1,
  -1, -1, -1,
  -1, -1, -1,
  -1, -1, -1,
  -1
};

// Liraries for Json
#include "ArduinoJson.h"

int COUNT;

void setup()
{
  Serial.begin(115200);
  Serial.println("one_ppg_i2c_one_imu_spi_one_ecg");

  Wire.setClock(3400000);
  Wire.begin();
  //Setup to sense up to 18 inches, max LED brightness
  byte ledBrightness = 0xFF; //Options: 0=Off to 255=50mA
  byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 3; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 1600; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 16384; //Options: 2048, 4096, 8192, 16384

  // Initialize sensor
  if (particleSensor.begin(Wire, I2C_SPEED_FAST, 0x57) == false)
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }


  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor. Use 6.4mA for LED drive
  particleSensor.setPulseAmplitudeRed(0x50);
  particleSensor.setPulseAmplitudeIR(0x50);
  particleSensor.setPulseAmplitudeGreen(0x96);
  particleSensor.setPulseAmplitudeProximity(0xff);
  Serial.println("MAX30105 Success!");

  // Try to initialize!
  if (!tmp117.begin()) {
    Serial.println("Failed to find TMP117 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("TMP117 Success!");
  tmp117.setAveragedSampleCount(TMP117_AVERAGE_8X);
  tmp117.setReadDelay(TMP117_DELAY_125_MS);

  
  SPI.begin();
  //  SPI.beginTransaction(SPISettings(7000000, MSBFIRST, SPI_MODE0));

  myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {

    myICM.begin(CS_PIN, SPI);

    myICM.startupDefault(false); // Force a full - not minimal - startup
    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      Serial.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
  Serial.println("ICM20948 Success!");


  pinMode(FSR_PIN, INPUT);


  Serial.println("Bluefruit52 Throughput Example");
  Serial.println("------------------------------\n");

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this manually via PIN 19
  Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  Bluefruit.Periph.setConnInterval(6, 12); // 7.5 - 15 ms
  Bluefruit.setName("PPG_Ring#1");
  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  bleuart.setRxCallback(bleuart_rx_callback);
  bleuart.setNotifyCallback(bleuart_notify_callback);

  // Set up and start advertising
  startAdv();
  Serial.println("Please use Adafruit's Bluefruit LE app to connect in UART mode");
  Serial.println("Once connected, enter character(s) that you wish to send");

}

void startAdv(void)
{
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // There is no room for Name in Advertising packet
  // Use Scan response for Name
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
     - Enable auto advertising if disconnected
     - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
     - Timeout for fast mode is 30 seconds
     - Start(timeout) with timeout = 0 will advertise forever (until connected)

     For recommended advertising interval
     https://developer.apple.com/library/content/qa/qa1931/_index.html
  */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void connect_callback(uint16_t conn_handle)
{
  BLEConnection* conn = Bluefruit.Connection(conn_handle);
  Serial.println("Connected");

  // request PHY changed to 2MB
  Serial.println("Request to change PHY");
  conn->requestPHY();

  // request to update data length
  Serial.println("Request to change Data Length");
  conn->requestDataLengthUpdate();

  // request mtu exchange
  Serial.println("Request to change MTU");
  conn->requestMtuExchange(247);

  // request connection interval of 7.5 ms
  //conn->requestConnectionParameter(6); // in unit of 1.25

  // delay a bit for all the request to complete
  delay(1000);
}

/**
   Callback invoked when a connection is dropped
   @param conn_handle connection where this event happens
   @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
*/
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  Serial.print("total count:");
  Serial.println(COUNT);
  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);

}

void bleuart_rx_callback(uint16_t conn_hdl)
{
  (void) conn_hdl;

  rxLastTime = micros();

  // first packet
  if ( rxCount == 0 )
  {
    rxStartTime = micros();
  }

  uint32_t count = bleuart.available();

  rxCount += count;
  bleuart.flush(); // empty rx fifo

  // Serial.printf("RX %d bytes\n", count);
}

void bleuart_notify_callback(uint16_t conn_hdl, bool enabled)
{
  if ( enabled )
  {
    Serial.println("Send a key and press enter to start test");
  }
}

void clear_all_max() {
  for (int i = 0; i < total_num_data; i++) {
    ALL_MAX[i] = -1;
  }
}

void get_ppg_data(void) {
  particleSensor.check();
  while (particleSensor.available()) //do we have new data?
  {
    ALL_MAX[0] = particleSensor.getFIFORed();
    ALL_MAX[1] = particleSensor.getFIFOIR();
    ALL_MAX[2] = particleSensor.getFIFOGreen();

    particleSensor.nextSample();
  }
}

void get_imu_data(void) {

  if (myICM.dataReady())
  {
    myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
    float ALL_IMU[] = {myICM.accX() * 0.01, myICM.accY() * 0.01, myICM.accZ() * 0.01, // acceleration is measured in m/s^2
                       myICM.gyrX(),  myICM.gyrY(), myICM.gyrZ(), // magnetometer is measured in uT
                       myICM.magX(), myICM.magY(), myICM.magZ(),// gyro is measured in radians/s
                      };


    for (int i = 0; i < 9; i++) {
      ALL_MAX[3 + i] = ALL_IMU[i];
    }

  }
  else
  {
    Serial.println("Waiting for data");
    delay(500);
  }
}

float get_fsr()
{
  float force = -1;
  int fsrADC = analogRead(FSR_PIN);
  // If the FSR has no pressure, the resistance will be
  // near infinite. So the voltage should be near 0.
  if (fsrADC != 0) // If the analog reading is non-zero
  {
    // Use ADC reading to calculate voltage:
    float fsrV = fsrADC * VCC / 1023.0;
    // Use voltage and static resistor value to
    // calculate FSR resistance:
    float fsrR = R_DIV * (VCC / fsrV - 1.0);
    //    Serial.println("Resistance: " + String(fsrR) + " ohms");
    // Guesstimate force based on slopes in figure 3 of
    // FSR datasheet:

    float fsrG = 1.0 / fsrR; // Calculate conductance
    // Break parabolic curve down into two linear slopes:
    if (fsrR <= 600)
      force = (fsrG - 0.00075) / 0.00000032639;
    else
      force =  fsrG / 0.000000642857;

  }
  //  Serial.println("Force: " + String(force) + " g");
  //  Serial.println();
  return force;
}

void get_tmp_data() {
  sensors_event_t temp; // create an empty event to be filled
  tmp117.getEvent(&temp);
  ALL_MAX[total_num_data - 4] = temp.temperature;
}

void loop()
{
  if (Bluefruit.connected() && bleuart.notifyEnabled())
  {
    send_time = millis();
    clear_all_max();
    get_imu_data();
    get_ppg_data();
    get_tmp_data();
    if (ALL_MAX[0] != -1)
    {
      ALL_MAX[total_num_data - 1] = send_time;
      ALL_MAX[total_num_data - 2] = get_fsr();
      ALL_MAX[total_num_data - 3] = analogRead(A2);
      send_message();
      COUNT += 1;
      Serial.println(COUNT);
//      delay(5);
    }

  }

  if (!Bluefruit.connected()) {
    COUNT = 0;
  }

  delay(3);
}




void send_message(void)
{
  //  int packet_size = total_num_data - 1;
  int total_size = total_num_data * 4;
  byte byteArray[total_size];// 10+1


  for (int i = 0; i < total_num_data; i++) {
    byteArray[i * 4] = ((uint8_t*)&ALL_MAX[i])[0];
    byteArray[i * 4 + 1] = ((uint8_t*)&ALL_MAX[i])[1];
    byteArray[i * 4 + 2] = ((uint8_t*)&ALL_MAX[i])[2];
    byteArray[i * 4 + 3] = ((uint8_t*)&ALL_MAX[i])[3];
  }

  size_t sent_data_size = bleuart.write(byteArray, sizeof(byteArray));

}
