#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include<SoftwareSerial.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>


SoftwareSerial espSerial(10, 11); // RX, TX

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//web page

const char index_html[] PROGMEM = R"rawliteral( <!DOCTYPE html><html><head><title>Remote IoT Data Monitoring and Control</title><style> body {font-family: Arial, sans-serif;margin: 0;padding: 30px;background-color: white;}h1 {text-align: center;color: #333; }.button-container {text-align: center;margin-top: 20px;}.button {padding: 10px 20px; font-size: 15px; font-weight: bold; margin: 0 10px; cursor: pointer; background-color: #00695c;   color: white;  border: none;  border-radius: 5px; }.button.on { background-color: green; }.button.off { background-color: red;  }.alert { background-color: #ff555b;  color: #d80c;   padding: 10px;  margin-top: 20px; text-align: center;}</style></head><body><h1>Remote IoT Data Monitoring and Control (using Wi-Fi)</h1><div class="button-container"><button id="led1Button" class="button" onclick="sendCommand('led1_on')">LED1 ON</button><button id="led2Button" class="button" onclick="sendCommand('led1_off')">LED1 OFF</button><br><br><button class="button" onclick="sendCommand('led2_on')">LED2 ON</button><button class="button" onclick="sendCommand('led2_off')">LED2 OFF</button></div><div id="alert" class="alert" style="display: none;"></div><script>function sendCommand(command) { var xhttp = new XMLHttpRequest();xhttp.onreadystatechange = function() {if (this.readyState == 4 && this.status == 200) {if (this.responseText == "success") {displayAlert("Command sent successfully.");updateButtonState(command);} else {displayAlert("Failed to send command.");}}};xhttp.open('GET', '/' + command, true); xhttp.send();}function displayAlert(message) {var alertDiv = document.getElementById("alert");alertDiv.innerHTML = message;alertDiv.style.display = "block";setTimeout(function() {alertDiv.style.display = "none";}, 3000);}function updateButtonState(command) {var button = document.getElementById(command == 'led1_on' ? 'led1Button' : 'led1Button');if (command.endsWith('_on')) {button.classList.remove('off');button.classList.add('on');}else {button.classList.remove('on');button.classList.add('off');}}</script></body></html><!DOCTYPE html><html lang="en"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0"><title>Sensor Value</title><style> h1 { color: #ec788a; }button { padding: 10px 20px; font-size: 15px; font-weight: bold;margin: 0 10px;cursor: pointer;background-color: #00695c; color: white;border: none;border-radius: 5px; } </style></head><body><h1>Sensors Value</h1><button onclick="displaySensor1()">Smoke Sensor</button><button onclick="displaySensor2()">Temperature Sensor</button><button onclick="displaySensor3()">Humidity Sensor</button> <div class="sensor-box" id="sensorBox1"></div><div class="sensor-box" id="sensorBox2"></div><div class="sensor-box" id="sensorBox3"></div> <script>function displaySensor1() {var sensor1Value = smoke_reading; document.getElementById('sensorBox1').textContent = "Smoke Sensor: " + sensor1Value; }function displaySensor2() {var sensor2Value = temp;document.getElementById('sensorBox2').textContent = "Temperature Sensor: " + sensor2Value; }function displaySensor3() {var sensor3Value = humidity;document.getElementById('sensorBox3').textContent = "Humidity Sensor: " + sensor3Value; }</script></body></html> )rawliteral" ;

String led1OnCommand = "/LEDON1" ;
String led2OnCommand = "/LEDON2" ;
String led1OffCommand = "/LEDOFF1" ;
String led2OffCommand = "/LEDOFF2" ;


void webPage();
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



// Define I2C address and LCD size
const int lcdAddress = 0x27; // Replace with your LCD I2C address if different

LiquidCrystal_I2C lcd(lcdAddress, 16, 2);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//pins
const int Mq2_pin = A0; // Analog input 0 (PC0)

const int buzzer_pin = 6; // Digital output 6 (PB0)

const int lm_pin = A1; // Analog input 1 (PC1) 

const int DHTPIN = 8; // Digital output 8 (PD0) 

#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);


const int PB_pin = 13;

const int ldr_pin = A2; // Analog input 2 (PC2) 
const int PIR_pin = 3; // Digital input 3 (PD3)  INT0
const int led1 = 4; // Digital output 4 (PB2)
const int led2 = 5; // Digital output 5 (PB3) 

uint16_t desired_delay_count = 32; // 2 seconds Clock frequency XTAL 16Mhz * Delay time 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//functions


void sendATCommand(String command, int waitTime = 1000);
void printIP();

void lcd_show();

void handleClientCommunication();

void smoke_alarm();

void temp();

void humidity();

void motion_detection();

uint16_t myAnalogRead(uint8_t pin); 

void timer1_init();

void thank_you();

volatile uint16_t timer1_count = 0; // Counter 
volatile bool delay_complete = false; // dealy complete

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





void setup()
{
 

  // Initialize the LCD communication
  lcd.init();
  //2l3b b lcd backlight
   lcd.backlight();
   //lcd.noBacklight();

//serial monitor
  Serial.begin(115200);
  espSerial.begin(9600);


 
  // Connect to Wi-Fi network
  sendATCommand("AT+RST", 2000);
  sendATCommand("AT+CWMODE=1");
  sendATCommand("AT+CWJAP=usus,12345678", 5000);
  sendATCommand("AT+CIFSR");
  printIP();
  sendATCommand("AT+CIPMUX=1");
  sendATCommand("AT+CIPSERVER=1,80");


//smoke sensor
  pinMode(Mq2_pin,INPUT);



//buzzer
  pinMode(buzzer_pin,OUTPUT);


//lm35
  pinMode(lm_pin,INPUT);

//dht11
dht.begin();


//motion 

 
  pinMode(PIR_pin, INPUT_PULLUP); // pull-up resistor 
  EICRA |= (1 << ISC01); // set INT0 interrupt on rising edge
  sei(); // enable global interrupt

//ldr 

 pinMode(ldr_pin,INPUT);

//leds
  pinMode(led1,OUTPUT);
  pinMode(led2,OUTPUT);

//push button for lcd light
pinMode(PB_pin,INPUT_PULLUP);


//timer 1 interrupt 
 timer1_init();

 

}

void loop()
{
delay(200);
webPage();
handleClientCommunication();



  
//smoke
smoke_alarm();

//temp lm 35
temp();

//temp lm 35 interrupt
//Timer1_Init_Interrupt();

//dht humidity
humidity();

//motion 
motion_detection();




//thank you lcd 
thank_you();

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//ADC func 

uint16_t myAnalogRead(uint8_t pin) {
  ADMUX = (1 << REFS0) | (pin & 0x07); //2l v ref 5 volt 
  ADCSRA |= (1 << ADEN); // Enable ADC
  ADCSRA |= (1 << ADSC); // Start conversion

  while (ADCSRA & (1 << ADSC)); // momkn use ADIF 

  float data = ADC;  // ADCH:ADCL
  data = ((data * 5000 / 1024) / 10 )/4;
  
  return data;
}



//number 1 2sdy timer 1 hahaha  

 //XTAL of arduino uno 16 Mhz

void timer1_init() {

   cli();       // Disable interrupts . 34an kant btr34
  
  TCCR1A = 0; // normal mode 
  TCCR1B = 0; 
  
  TCCR1B |= (1 << CS12) | (1 << CS10); //prescaling clk/1024 23la 7aga


    OCR1AH = 0x7A;
  OCR1AL = 0x12;                        // Set Timer 1 compare match value to 2 seconds ta2reeban
  
  TCCR1B |= (1 << WGM12);               // Set Timer 1 to CTC mode
 
  
  TIMSK1 |= (1 << OCIE1A); // tool ma 2l flag mtrf34
  sei(); 
}




// Interrupt service routine for Timer 1

ISR(TIMER1_COMPA_vect) {
  timer1_count++;
  if (timer1_count >= desired_delay_count) {
    timer1_count = 0;
    delay_complete = true;
    return;
  }
}






//lcd func
void lcd_show(int sensor_reading,String show){
  
   lcd.setCursor(0, 0);
  lcd.print("                "); // Clear 16 characters

 
  lcd.setCursor(0, 0);
  
  lcd.print(show);
  
  lcd.print(sensor_reading, 1); // Print with one decimal place

 

  delay(5000); 

  
  }


//smoke func

void smoke_alarm(){

  const int smoke_trigger = 500;//gas 
  
  int smoke_reading = analogRead(Mq2_pin);

  delay(5000);

  if(smoke_reading >= smoke_trigger){

    digitalWrite(buzzer_pin,HIGH);
    delay(2000);
    digitalWrite(buzzer_pin,LOW);
    delay(2000);
    
    }
  else{

    lcd_show(smoke_reading,"smoke :");
    }
  
}


//temp func

void temp(){
  

  float temp = myAnalogRead(lm_pin);

  Serial.println("\n darget 2l harara now");
      Serial.print(temp);
      delay(1000);

  if(temp > 45){

       lcd.setCursor(0, 0);
  lcd.print("                "); // Clear 16 characters

 
  lcd.setCursor(0, 0);
  
  lcd.print("It's too HOT!!!!");

  digitalWrite(buzzer_pin,HIGH);
  delay(1000);
  digitalWrite(buzzer_pin,LOW);
  delay(1000);
    

  delay(5000);

   
    }
}
  
  



//dht func 

void humidity(){
   float humidity = dht.readHumidity();

   
    Serial.println("\n humidty:");
    
   Serial.println(humidity);

  lcd_show(humidity,"humidity:");

  delay(2000);
  
//   while (!delay_complete);
//  delay_complete = false;


  }





//check motion in garden 
void motion_detection(){
  int pir_reading;

pir_reading =  digitalRead(PIR_pin);

float light_value = myAnalogRead(ldr_pin);

    Serial.println("\n light value is :");
    Serial.println( light_value);

//check for day light
    if(light_value <= 15){

if (pir_reading == 1){
  Serial.print("\n motion detected");

  //trigger , buzzer goes high 
  digitalWrite(buzzer_pin,HIGH);
  delay(1000);
   digitalWrite(buzzer_pin,LOW);
   delay(1000);
  
     lcd.setCursor(0, 0);
  lcd.print("                "); // Clear 16 characters

 
  lcd.setCursor(0, 0);
  
  lcd.print("Someone IS There");

  delay(2000);
  
  
}
    }
}


void sendATCommand(String command, int waitTime) {
  espSerial.println(command);
  delay(waitTime);
  while (espSerial.available()) {
    Serial.write(espSerial.read());
  }
}

void printIP() {
  espSerial.println("AT+CIFSR");
  delay(1000);
  while (espSerial.available()) {
    String response = espSerial.readString();
    if (response.indexOf("STAIP") != -1) {
      Serial.println("IP Address: " + response.substring(response.indexOf(""") + 1, response.lastIndexOf(""")));
    }
  }
}




void handleClientCommunication() {
  if (espSerial.available()) {
    
    String message = espSerial.readStringUntil('\n');  // Read message

   // Check for "+IPD" notification
    
   if (message.indexOf("+IPD,") == 0) { // fe client connect listen w recieve mnoo data 
      
     int indexOfDataStart = message.indexOf(':') + 1; //sebk mn 2l header w 5o4 3l content 
     
      String data = message.substring(indexOfDataStart); //7otly 2l data frame fe string w abd2 mn index zero 
      
     // Find LED control command in the request
      int commandIndex;
    
   commandIndex = data.indexOf(led1OnCommand);
    
    if (commandIndex >= 0) { //-1 lw m4 la2y 2l command 
      
      Serial.println("LED1 truned On!");
      
      digitalWrite(led1, HIGH);  

       return;
     }

      commandIndex = data.indexOf(led1OffCommand);
     
     if (commandIndex >= 0) { //-1 lw m4 la2y 2l command 
      
      Serial.println("LED1 truned Off!");
      
      digitalWrite(led1, LOW);  

       return ;
     }

     commandIndex = data.indexOf(led1OffCommand);
     
     if (commandIndex >= 0) { //-1 lw m4 la2y 2l command 
      
      Serial.println("LED1 truned Off!");
      
      digitalWrite(led1, LOW);  

       return ;
     }

      commandIndex = data.indexOf(led2OnCommand);
     
     if (commandIndex >= 0) { //-1 lw m4 la2y 2l command 
      
      Serial.println("LED2 truned On!");
      
      digitalWrite(led2, HIGH);  

       return ;
     }

     commandIndex = data.indexOf(led2OffCommand);
     
     if (commandIndex >= 0) { //-1 lw m4 la2y 2l command 
      
      Serial.println("LED2 truned Off!");
      
      digitalWrite(led2, LOW);  

       return ;
     }

     Serial.println("Unknown command received!");
   }
 }
}





void webPage(){
  
  if (espSerial.available()) {
    String message = espSerial.readStringUntil('\n');  // Read message

    // Check for a connection request
    if (message.startsWith("GET / HTTP/1.1")) {
      // Send the HTML page content
      espSerial.print("HTTP/1.1 200 OK\r\n");
      espSerial.print("Content-Type: text/html\r\n");
      espSerial.print("Connection: close\r\n");
      espSerial.print("Content-Length: ");
      espSerial.println(sizeof(index_html) - 1);  // Calculate and print content length
      espSerial.println();  // Add an empty line for header separation
      espSerial.print(index_html);  // Send the HTML content from PROGMEM
    } else {
      // Handle other types of requests (optional)
      Serial.println("Unknown request received!");
    }
  }
  
  }

void thank_you(){
  
int buttonState = digitalRead(PB_pin);
  if (buttonState == LOW) {

    lcd.clear();
  lcd.println("Thank You Doctor");

  delay(5000); 

  }
}
