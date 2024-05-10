#define RefreshTime 12



#include <WiFi.h>             // this header is used to enable esp8266 wifi
#include "Adafruit_MQTT.h"           // this header is used to inculde ada fruit IoT functions
#include "Adafruit_MQTT_Client.h"    // this header is used to include ada fruit  client IoT functions


#define WLAN_SSID       "BestProject"                           //used to store the name of Wifi network
#define WLAN_PASS       "12345678"                        //used to store the password of Wifi network


/********* Adafruit.io Setup ***********/
#define AIO_SERVER      "io.adafruit.com"                    // used to store server name
#define AIO_SERVERPORT  1883                                 // use 8883 for SSL
#define AIO_USERNAME    "Gagan_123"                          //used to store user name of account
#define AIO_KEY         "aio_BGpL04uzactyg51q14hrDKaVuUEr"   //used to store key of server

WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);          //set the data to mqqt function

Adafruit_MQTT_Publish Feed1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Moisture");  // used to send the voltage data to server
Adafruit_MQTT_Publish Feed2 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");  // used to send the current data to server
Adafruit_MQTT_Publish Feed3 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Light");      // used to send the Kwh data to server
Adafruit_MQTT_Publish Feed4 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Temperature");     // used to send the watt data to server
Adafruit_MQTT_Publish Feed5 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/state");     // used to send the watt data to server
Adafruit_MQTT_Publish Feed6 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/motion");     // used to send the watt data to server



void MQTT_connect();            // user defined function for MQQT function declation






// this function is used to assign serial data pin of arduino

#define D_23 23 // 
#define D_22 22 // 

#define D_25_A28 25 // ADC 28
#define D_26_A29 26 // ADC 29
#define D_27_A27 27 // ADC 27

#define TX_0 01 // 
#define RX_0 03 //

#define D_21 21 //
#define D_19 19 //
#define D_18 18 // 
#define D_05 05 // 

#define TX_2 17 // 
#define RX_2 16 // 

#define D_04_A20 04 // ADC 20
#define D_02_A22 02 // ADC 22
#define D_15_A23 15 // ADC 23
#define D_12_A25 12 // ADC 25
#define D_13_A24 13 // ADC 24
#define D_14_A26 14 // ADC 26


#define V_P_A10 36 // ADC 10
#define V_N_A13 39 // ADC 13


#define D_32_A14 32 // ADC 14
#define D_33_A15 33 // ADC 15
#define D_34_A16 34 // ADC 16
#define D_35_A17 35 // ADC 17



/*

  #include <Wire.h>
  #include <LiquidCrystal_I2C.h>
  LiquidCrystal_I2C lcd(0x27, 16, 2); //sometimes the LCD adress is not 0x3f. Change to 0x27 if it dosn't work.
*/


#include <LiquidCrystal.h>
//LiquidCrystal lcd(D8, 10, 3, D2,  D1, D0); esp8266
//LiquidCrystal lcd(7, 8, 9, 10, 11, 12);  arduino
LiquidCrystal lcd( D_05, D_18,  D_19, D_21,  D_22,  D_23); //  esp32
//                 (RS,   E,    D4,    D5,   D6,    D7)




// Part of DHT11 Sensor

int pinDHT11 = TX_2, humidityTst,  temperatureTst;
#include <SimpleDHT.h>
SimpleDHT11 dht11(pinDHT11);
float Temperature, humidity;




// Part of Moisture Sensor

int moisInput = D_34_A16  ;
float Moist_pin = 0, MoistureInPer = 0,  Moisture = 0,  a,  b,  c;



// Part of light sensor

int Light_pin = D_35_A17;
float r1 = 33000.0,  r2 = 56000.0, Light = 0;




// Part of PIR sensor

int pir_pin = D_04_A20;
int motion = 0;




int RelayPin = D_32_A14  , state;









// void set up is used to run anything once at the time of starting and Pin direction assignment

void setup()
{
  //  lcd.init();                 //Init the LCD
  //  lcd.backlight();            //Activate backlight

  lcd.begin(16, 2);

  pinMode(RelayPin, OUTPUT);
  pinMode(pir_pin, INPUT);






  //Name Printing on LCD//


  lcd.clear();                      // used to clea the data on LCD

  lcd.setCursor(0, 0);              // used to set the cursor position for data printing on LCD (column, Row)
  lcd.print("IoT Agriculture");    // used to print the data as it is

  lcd.setCursor(0, 1);              // used to set the cursor position for data printing on LCD (column, Row)
  lcd.print("  Monitoring");     // used to print the data as it is

  delay(2000);                      // delay is used to hold the data on LCD for 4 seconds
  lcd.clear();                      // used to clea the data on LCD




  Serial.begin(115200);    // this function is used to set the speed of serial monitor of IDE


  /********* Connecting to Wifi Network ***********/

  /*
    Serial.print("Connecting to ");       // print the as it is sentence on serial monitor
    Serial.println(WLAN_SSID);            // used to print the wifi name on Monitor
  */

  WiFi.begin(WLAN_SSID, WLAN_PASS);     // used to initialize the wifi network on Node MCU
  if (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print("."); // print the as it is sentence on serial monitor
  }
  /*
    Serial.println();
    Serial.println("WiFi connected"); // print the as it is sentence on serial monitor
    Serial.println("IP address: ");  // print the as it is sentence on serial monitor
    Serial.println(WiFi.localIP());
  */

}











// void loop is used to run the the fundtion continously untill the aduino is on //


void loop()
{


  // part of DHT11 sensor




  // read without samples.
  byte temperatureTst = 0;
  byte humidityTst = 0;
  int err = SimpleDHTErrSuccess;

  dht11.read(&temperatureTst, &humidityTst, NULL);

  Temperature = (int)temperatureTst;
  //  Serial.print("Sample OK: ");
  //  Serial.print(Temperature);

  humidity = (int)humidityTst;
  //  Serial.print(" *C, ");
  //  Serial.print(humidity);
  //  Serial.println(" H");





  // Part of light sensor

  Light = ( analogRead(Light_pin) / 4096.0 ) * ((r1 + r2) / r2) / 2 * 100;





  // Part of Moisture Sensor

  Moist_pin = analogRead(moisInput);
  MoistureInPer = (((Moist_pin / 4096.00)  * 100) - 100) ;
  Moisture = (abs(MoistureInPer)) * 1.2500;
  //Serial.println(Moisture);




  if (Moisture < 20)
  {
    digitalWrite(RelayPin, HIGH);
    state = 1;
  }

  if (Moisture > 70)
  {
    digitalWrite(RelayPin, LOW);
    state = 0;
  }

  if (Moisture > 100)
  {
    Moisture = 100;
  }



  lcd.clear();


  //lcd.clear();

  if (state == 1)
  {
    //lcd.clear();
    lcd.setCursor(7, 1);               // used to set the cursor position for data printing on LCD (column, Row)
    lcd.print("ON");
  }



  if (state == 0)
  {
    // lcd.clear();
    lcd.setCursor(7, 1);               // used to set the cursor position for data printing on LCD (column, Row)
    lcd.print("OF");
  }


  // data printing on lcd//

  lcd.setCursor(7, 0);               // used to set the cursor position for data printing on LCD (column, Row)
  lcd.print("MT");                  // used to print the data as it is
  // used to print the current data and, 0 is used to display no of decimal point of the data




  lcd.setCursor(0, 0);               // used to set the cursor position for data printing on LCD (column, Row)
  lcd.print("M=");                  // used to print the data as it is
  lcd.setCursor(2, 0);               // used to set the cursor position for data printing on LCD (column, Row)
  lcd.print(Moisture, 0);             // used to print the current data and, 0 is used to display no of decimal point of the data
  lcd.setCursor(5, 0);               // used to set the cursor position for data printing on LCD (column, Row)
  lcd.print("%");



  lcd.setCursor(0, 1);               // used to set the cursor position for data printing on LCD (column, Row)
  lcd.print("L=");                   // used to print the data as it is
  lcd.setCursor(2, 1);               // used to set the cursor position for data printing on LCD (column, Row)
  lcd.print(Light, 0);           // used to print the voltage data and, 2 is used to display no of decimal point of the data
  lcd.setCursor(5, 1);               // used to set the cursor position for data printing on LCD (column, Row)
  lcd.print("%");



  lcd.setCursor(10, 0);              // used to set the cursor position for data printing on LCD (column, Row)
  lcd.print("H=");                   // used to print the data as it is
  lcd.setCursor(12, 0);              // used to set the cursor position for data printing on LCD (column, Row)
  lcd.print(humidity, 0);               // used to print the watte  data and, 3 is used to display no of decimal point of the data
  lcd.setCursor(15, 0);              // used to set the cursor position for data printing on LCD (column, Row)
  lcd.print("%");



  lcd.setCursor(10, 1);               // used to set the cursor position for data printing on LCD (column, Row)
  lcd.print("T=");                 // used to print the data as it is
  lcd.setCursor(12, 1);             // used to set the cursor position for data printing on LCD (column, Row)
  lcd.print(Temperature, 0); // used to print the milli watt hour data and, 0 is used to display no of decimal point of the data
  lcd.setCursor(15, 1);               // used to set the cursor position for data printing on LCD (column, Row)
  lcd.print("c");








  MQTT_connect(); // this funcion connets to server, it is defined in the last

  // this part continously check either the data is sent to sever or not and see the status on monitor of IDE //


  if (! Feed1.publish(Moisture))
  {
    Serial.println(F("Failed"));     // print the as it is sentence on serial monitor
  }
  else
  {
    Serial.println(F("OK!"));        // print the as it is sentence on serial monitor
  }



  if (! Feed3.publish(Light))
  {
    Serial.println(F("Failed"));    // print the as it is sentence on serial monitor
  }
  else
  {
    Serial.println(F("OK!"));       // print the as it is sentence on serial monitor
  }




  if (! Feed2.publish(humidity))
  {
    Serial.println(F("Failed"));    // print the as it is sentence on serial monitor
  }
  else
  {
    Serial.println(F("OK!"));       // print the as it is sentence on serial monitor
  }



  if (! Feed4.publish(Temperature))

  {
    Serial.println(F("Failed"));    // print the as it is sentence on serial monitor
  }
  else
  {
    Serial.println(F("OK!"));       // print the as it is sentence on serial monitor
  }


  if (! Feed5.publish(state))

  {
    Serial.println(F("Failed"));    // print the as it is sentence on serial monitor
  }
  else
  {
    Serial.println(F("OK!"));       // print the as it is sentence on serial monitor
  }
  // ping the server to keep the mqtt connection alive





  delay(RefreshTime * 1000);                     // delay is used to hold the data on LCD for 9 seconds
  // used to clear the data on LCD so that next data should only present on LCD





}









// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect()

{
  int8_t ret;


  // Stop if already connected.
  if (mqtt.connected())
  {
    return;
  }

  // Serial.println("Connecting to Adafruit... ");

  uint8_t retries = 3;

  if ((ret = mqtt.connect()) != 0)
  {
    // connect will return 0 for connected

    //    Serial.println(mqtt.connectErrorString(ret));
    //    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
  }
  // Serial.println("Adafruit Connected!");
}