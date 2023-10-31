#include <RTClib.h>
#include <SerialFlash.h>
#include <SPI.h>
#include <Wire.h>
#include <lmic.h>
#include <hal/hal.h>
#include <DHT.h>

#define SENSOR_FILE_SIZE 16384
#define STATUS_FILE_SIZE 4096
#define DHTPIN 2
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);
//SoftwareSerial Serial1(3,4);
const int FlashChipSelect = 12;
int intervaloEstado = 180;
int intervaloSensor = 60;
unsigned long lastEstado = 0;
unsigned long lastSensor = 0;

//Datos para sensor
int conta = 0;  //Variable para guardar el conteo de los pulsos
int pin = A6;
int value = 1;
float rainRate = 0.0;

RTC_DS3231 rtc;

//LORAWAN DATA
static uint8_t NWKSKEY[16] = { 0xCF, 0xD2, 0x96, 0x7E, 0xC8, 0xA1, 0x0F, 0x85, 0xF0, 0xE7, 0x4E, 0xE5, 0x0A, 0xA7, 0x99, 0x8C };
static uint8_t APPSKEY[16] = { 0xE1, 0x07, 0x12, 0x6F, 0xCA, 0x21, 0xBA, 0x86, 0x0B, 0xC3, 0x60, 0x09, 0xB1, 0x62, 0xF4, 0x5B };
static uint32_t DEVADDR = 0x26011B67 ;

static osjob_t sendjob;
const unsigned TX_INTERVAL = 30;
String dataToSend = "";  

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 4,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 1,
    .dio = {13, 14, 15}, 
};

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

//Permite controlar las acciones a realizar en el envio de datos de LORAWAN
void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
   
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
        
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            
            // Se programa el proximo envio de datos            
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

//Funcion que inicializa las variables y procesos a realizar 
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);

  #ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
  #endif
    
  pinMode(pin, INPUT);

  // wait for Arduino Serial Monitor
  while (!Serial);
  
  delay(1000);

  //Se inicia la memoria
  if (!SerialFlash.begin(FlashChipSelect)) {
    Serial.println("Unable to access SPI Flash chip");
  }

  //Se inicializa el reloj
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));   

  #ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  #endif

  dht.begin();

  delay(1000);

  //Se inicia LMIC
  os_init();

  setConfigData();
  setFirstRegistry();
}

//Funcion que se ejecuta reiterativamente
void loop() {
  unsigned long time = millis();

  verificarMCU();

  //Sensor pluviométrico
  value = digitalRead(pin);  //lectura digital de pin
 
  //mandar mensaje a puerto serie en función del valor leido
  if (value != LOW) {
    conta++;               //Incrementa el contador
    while(value != LOW){
        value = digitalRead(pin);
    }
  }

  //Registrar datos de estado
  if (lastEstado == 0 || time - lastEstado >= getMillisFromSeconds(intervaloEstado)){
    lastEstado = time;
    sendStatusData();
  }

  //Registrar datos de sensor
  if (lastSensor == 0 || time - lastSensor >= getMillisFromSeconds(intervaloSensor)){
    rainRate = conta * (0.2);
    conta=0;
    lastSensor = time;
    sendSensorData();
  }
  
  os_runloop_once();
}

//Funcion que permite inicializar el LORA con los parametros especificados
void initLORA(uint32_t daddr, uint8_t* nwkskey, uint8_t* appskey){
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  LMIC_setSession (0x1, daddr, nwkskey, appskey); 

  #if defined(CFG_eu868)
  LMIC_setupChannel(0, 433100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);      // g-band
  #elif defined(CFG_us915)
  LMIC_selectSubBand(1);
  #endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7,14);

  // Start job
  do_send(&sendjob);
}

//Funcion que verifica si hay datos en la cola para enviar a traves del LORA
void do_send(osjob_t* j){
  Serial.println("Data completa: " + dataToSend);
  
  if (dataToSend.length() == 0){
    os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
    return;
  }
  
  byte buffer[13];

  String data = getFirst(dataToSend, ";");
  dataToSend = removeFirst(dataToSend, ";");
  Serial.println("Data: " + data);
  Serial.println("Data cola: " + dataToSend);

  getDataArray(buffer, data);
     
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
      // Prepare upstream data transmission at the next possible time.
      LMIC.bands[BAND_MILLI].avail = os_getTime();
      LMIC.bands[BAND_CENTI].avail = os_getTime();
      LMIC.bands[BAND_DECI ].avail = os_getTime();
      LMIC_setTxData2(2, buffer, 13, 0);
      //LMIC_setTxData2(2, mydata1, 34, 0);
      Serial.println(F("Packet queued"));

  }
  // Next TX is scheduled after TX_COMPLETE event.
}

//Funcion que permite recoger los datos de Estado, almacenarlos en la memoria flash y ponerlos en cola
//para posteriormente enviarlos a traves del LORA
void sendStatusData(){
  byte buffer[12];
  DateTime now = rtc.now();
  unsigned long time = now.unixtime();
  String formattedDate = getFormattedDate(now);
  String fileName = "status_" + formattedDate + ".dat";

  int bateria = 100; //tomar valor real
	float t = dht.readTemperature(); //tomar valor real
	float h = dht.readHumidity(); //tomar valor real

  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    t = 0.0;
    h = 0.0;
  }

  int tInt = (int)(t * 10.0);
  int hInt = (int)(h * 10.0);

  buffer[0] = byte(time>>24);  
  buffer[1] = byte(time>>16);
  buffer[2] = byte(time>>8);
  buffer[3] = byte(time);
  buffer[4] = byte(intervaloEstado>>8);
  buffer[5] = byte(intervaloEstado);
  buffer[6] = byte(tInt>>8);
  buffer[7] = byte(tInt);
  buffer[8] = byte(hInt>>8);
  buffer[9] = byte(hInt);
  buffer[10] = byte(bateria>>8);
  buffer[11] = byte(bateria);

  saveData(fileName, STATUS_FILE_SIZE, buffer, 12, false);
  appendDataToSend(1, time, intervaloEstado, tInt, hInt, bateria);
}

//Funcion que permite recoger los datos del Sensor, almacenarlos en la memoria flash y ponerlos en cola
//para posteriormente enviarlos a traves del LORA
void sendSensorData(){
  byte buffer[8];
  DateTime now = rtc.now();
  unsigned long time = now.unixtime();
  String formattedDate = getFormattedDate(now);
  String fileName = "sensor_" + formattedDate + ".dat";

  //float rainRate = 25.3; //tomar valor real

  int rainRateInt = (int)(rainRate * 10.0);

  buffer[0] = byte(time>>24);  
  buffer[1] = byte(time>>16);
  buffer[2] = byte(time>>8);
  buffer[3] = byte(time);
  buffer[4] = byte(intervaloSensor>>8);
  buffer[5] = byte(intervaloSensor);
  buffer[6] = byte(rainRateInt>>8);
  buffer[7] = byte(rainRateInt);

  saveData(fileName, SENSOR_FILE_SIZE, buffer, 8, false);
  appendDataToSend(0, time, intervaloSensor, rainRateInt, 0, 0);
}

//Almacena en la memoria flash la primera fecha de la que se esta tomando datos
void setFirstRegistry(){
  char fileName[10] = "first.dat";
  int bufferSize = 32;
  
  if (!SerialFlash.exists(fileName)) {
    byte buffer[bufferSize];
    DateTime now = rtc.now();
    String registry = getFormattedDate(now);
    registry.getBytes(buffer, registry.length()+1);

    saveData(String(fileName), bufferSize, buffer, registry.length(), false);
  }
}

//Se obtiene de la memoria flash la primera fecha de la que se registro informacion
String getFirstRegistry(){
  char fileName[10] = "first.dat";

  if (SerialFlash.exists(fileName)) {
    SerialFlashFile file = SerialFlash.open(fileName);

    if (!file){
      Serial.println("The file first.dat cannot be opened");
      return "";
    }

    uint32_t lastPosition = getLastWrittenPosition(file);

    byte buffer[lastPosition];
    file.seek(0);    
    file.read(buffer, lastPosition);
    file.close();

    return String((char*)buffer);
  }

  return "";
}

//Se obtiene de la memoria flash los parametros de configuracion
String getConfigData(){
  char fileName[11] = "config.dat";

  if (SerialFlash.exists(fileName)) {
    SerialFlashFile file = SerialFlash.open(fileName);

    if (!file){
      Serial.println("The file first.dat cannot be opened");
      return "";
    }

    uint32_t lastPosition = getLastWrittenPosition(file);

    byte buffer[lastPosition];
    file.seek(0);    
    file.read(buffer, lastPosition);
    file.close();

    return String((char*)buffer);     
  }

  return "";
}

//Se configura la placa con los parametros almacenados en la memoria flash
void setConfigData(){
  setConfigDataStr(getConfigData());
}

//Se configura la placa con los parametros recibidos en la cadena de texto
void setConfigDataStr(String configData){

  if (configData.equals("")){ 
    setConfigDataParams(intervaloSensor, intervaloEstado, DEVADDR, NWKSKEY, 16, APPSKEY, 16);
  }else{
    int count = countStrings(configData, ";");
    String configArray[count];
    splitString(configArray, configData, ";");
    
    String is = count > 0 ? configArray[0] : "";
    String ie = count > 1 ? configArray[1] : "";
    String addrs = count > 2 ? configArray[2] : "";
    String nwks = count > 3 ? configArray[3] : ""; 
    String appks = count > 4 ? configArray[4] : ""; 

    intervaloSensor = is.toInt();
    intervaloEstado = ie.toInt();

    //Get Device Address
    uint32_t addr = 0;
    if (!addrs.equals("")){
      char addrC[addrs.length()];
      addrs.toCharArray(addrC, addrs.length()+1);
      addr = strtoul(addrC, NULL, 10); 
    }

    //Get Network Key
    int countNK = countStrings(nwks, ",");
    uint8_t nwkskey[countNK];
    getUint8ArrayFromString(nwkskey, nwks, ",");

    //Get Application Key
    int countAK = countStrings(appks, ",");
    uint8_t appskey[countAK];
    getUint8ArrayFromString(appskey, appks, ",");

    /*initLORA(addr != 0 ? addr : DEVADDR, 
              countNK != 0 ? nwkskey : NWKSKEY, 
              countAK != 0 ? appskey : APPSKEY);*/

    setConfigDataParams(intervaloSensor, intervaloEstado, 
                          addr != 0 ? addr : DEVADDR, 
                          countNK != 0 ? nwkskey : NWKSKEY, 
                          countNK != 0 ? countNK : 16, 
                          countAK != 0 ? appskey : APPSKEY, 
                          countAK != 0 ? countAK : 16);
  }
}

//Se configura la placa con los parametros recibidos y se almacenan en la memoria flash
void setConfigDataParams(int intSensor, int intStatus, uint32_t addr, uint8_t* nwkskey, int nwkscount, uint8_t* appskey, int appscount){
  char fileName[11] = "config.dat";
  int bufferSize = 256;
  
  if (SerialFlash.exists(fileName)) {
    SerialFlash.remove(fileName);
    /*SerialFlashFile file = SerialFlash.open(fileName);

    if (!file){
      Serial.println("The file " + String(fileName) + " cannot be opened and erased");
      return;
    }
    
    file.erase();
    file.close();*/
  }
  
  byte buffer[bufferSize];
  String data = String(intSensor) + ";" + String(intStatus) + ";" + String(addr) + ";" + getStringFromUint8Array(nwkskey, nwkscount, ",") + ";" + getStringFromUint8Array(appskey, appscount, ",");
  data.getBytes(buffer, data.length()+1);

  saveData(String(fileName), bufferSize, buffer, data.length(), true);

  intervaloSensor = intSensor;
  intervaloEstado = intStatus; 
  initLORA(addr, nwkskey, appskey);
}

//Permite almacenar informacion en la memoria flash, si el archivo no existe lo crea
void saveData(String fileName, int fileSize, byte* buffer, int bufferSize, boolean erasable){
  SerialFlashFile file;
  char buf[64];
  uint32_t lastPosition;
  boolean newFile = false;

  fileName.toCharArray(buf, fileName.length()+1);

  if (!SerialFlash.exists(buf)) {
    if(erasable){
      if (!SerialFlash.createErasable(buf, fileSize)) {
        Serial.println("The file " + fileName + " was not created");
        return;
      }
    } else{
      if (!SerialFlash.create(buf, fileSize)) {
        Serial.println("The file " + fileName + " was not created");
        return;
      } 
    }

    newFile = true;
    lastPosition = 0;
  } 

  file = SerialFlash.open(buf);

  if (!file){
    Serial.println("The file " + fileName + " cannot be opened");
    return;
  }

  if (!newFile){
    lastPosition = getLastWrittenPosition(file);      
  }

  file.seek(lastPosition);  
  file.write(buffer, bufferSize);

  Serial.println("Archivo " + fileName + " escrito " + String((char*)buffer) + " posicion actual " + String(file.position()));

  file.close();
}

//Permite obtener la ultima posicion escrita en un archivo determinado de la memoria flash
uint32_t getLastWrittenPosition(SerialFlashFile file){
  int size = file.size();
  int position = 0;
  int bufferLength = 8;
  byte buffer[bufferLength];
  file.seek(0);

  int cant = 0;

  while (true){
    position = file.position();
      file.read(buffer, bufferLength);

      for (int i = 0; i<bufferLength; i++){
        int val = (int)buffer[i];
       
        if (val==255){
          cant++;
        }else{
          cant=0;  
        }

        if (cant==8){
          return position + i - 7;
        }
      }
  }

  return 0;
}

//Permite leer la informacion de un archivo en la memoria flash y enviarla a traves del puerto serial
void sendDataMCU(String fileName){
  int bufferSize = 64;
  char buf[64];
  fileName.toCharArray(buf, fileName.length()+1);

  if (SerialFlash.exists(buf)) {
    SerialFlashFile file = SerialFlash.open(buf);

    if (file){
      byte buffer[bufferSize];
      uint32_t lastPosition = getLastWrittenPosition(file);

      Serial.println("El tamaño del archivo es " + String(lastPosition));
      
      byte sizeBuf[4];
      sizeBuf[0] = byte(lastPosition>>24);  
      sizeBuf[1] = byte(lastPosition>>16);
      sizeBuf[2] = byte(lastPosition>>8);
      sizeBuf[3] = byte(lastPosition);

      Serial.println("Los bytes del archivo es: " + String((int)sizeBuf[0]) + " " + String((int)sizeBuf[1]) + " " + String((int)sizeBuf[2]) + " " + String((int)sizeBuf[3]) + " ");
      
      Serial1.write(sizeBuf, 4);
      Serial1.flush();

      //while(Serial1.available()==0);
      //Serial.println(Serial1.readString());
      
      file.seek(0);

      if (lastPosition <= bufferSize){
        file.read(buffer, lastPosition); 
        Serial1.write(buffer, lastPosition);
        Serial1.flush();
      }else{
        int leidos = 0;

        while (true){
          int diff = 0;
          if (leidos + bufferSize <= lastPosition){
            diff = bufferSize;
          }else{
            diff = lastPosition - leidos;
          }

          int bytes = file.read(buffer, diff); 
          leidos += bytes;
          Serial1.write(buffer, diff);
          Serial1.flush();

          //Serial.println(base64::encode(String((const char*)buffer)));

          if (lastPosition == leidos || leidos == file.size()){
            break;
          }
        }
      }
           
      file.close();
    }else{
      byte res[] = {0, 0, 0, 0};
      Serial.println("El archivo no puede ser leido");
      Serial1.write(res, 4);
    }
  }else{
      byte res[] = {0, 0, 0, 0};
    Serial.println("El archivo no existe");
    Serial1.write(res, 4);
  }
}


//Permite obtener los nombres de todos los archivos en la memoria flash
String getFilesFromMemory(){
  String fileNames = "";

  SerialFlash.opendir();
  while (1) {
    char filename[64];
    uint32_t filesize;

    if (SerialFlash.readdir(filename, sizeof(filename), filesize)) {
      fileNames = fileNames + String(filename) + ";";
    } else {
      break; // no more files
    }
  }

  return fileNames;
}

//Permite verificar si es que se esta recibiendo informacion a traves del Serial, 
//en este caso puede estar conectado el Gateway WiFi
void verificarMCU(){
  if(Serial1.available()>0)
  {
    String data = Serial1.readString();
    Serial.println("Receiving data: " + data);

    if (data.equals("files")){
      String fileNames = getFilesFromMemory();
      Serial1.print(fileNames);
      Serial.println("Data sent: " + fileNames);
    }else if (data.equals("first")){
      String first = getFirstRegistry();
      Serial1.print(first);
      Serial.println("Data sent: " + first);
    }else if (data.equals("config")){
      String config = getConfigData();
      Serial1.print(config);
      Serial.println("Data sent: " + config);
    }else if (data.startsWith("file_")){
      String fileName = data.substring(5);
      sendDataMCU(fileName);
    }else if (data.startsWith("config_")){
      String configData = data.substring(7);

      if (!configData.equals("")){
        setConfigDataStr(configData);
      }   

      Serial1.print("OK");
    }
  }
}

//Permite obtener un arreglo de bytes a partir de una cadena de texto
void getDataArray(byte* data, String cad){
  if (cad.length() != 0){
    int tipo = getFirst(cad, ",").toInt();
    cad = removeFirst(cad, ",");
    String timeStr = getFirst(cad, ","); 
    char timeC[timeStr.length()];
    timeStr.toCharArray(timeC, timeStr.length()+1);
    unsigned long time = strtoul(timeC, NULL, 10);
    cad = removeFirst(cad, ",");
    int intervalo = getFirst(cad, ",").toInt();
    cad = removeFirst(cad, ",");
    int d1 = getFirst(cad, ",").toInt();
    cad = removeFirst(cad, ",");
    int d2 = getFirst(cad, ",").toInt();
    cad = removeFirst(cad, ",");
    int d3 = getFirst(cad, ",").toInt();

    getDataArray(data, tipo, time, intervalo, d1, d2, d3);
  }
}

//Permite formar un arreglo de bytes a partir de los parametros solicitados
void getDataArray(byte* data, int tipo, unsigned long time, int intervalo, int d1, int d2, int d3){
  data[0] = tipo;
  data[1] = byte(time>>24);  
  data[2] = byte(time>>16);
  data[3] = byte(time>>8);
  data[4] = byte(time);
  data[5] = byte(intervalo>>8);
  data[6] = byte(intervalo);
  data[7] = byte(d1>>8);
  data[8] = byte(d1);
  data[9] = byte(d2>>8);
  data[10] = byte(d2);
  data[11] = byte(d3>>8);
  data[12] = byte(d3);
}

//Permite agregar informacion a la cola de datos a ser enviados por LORA 
void appendDataToSend(int tipo, unsigned long time, int intervalo, int d1, int d2, int d3){
  dataToSend = dataToSend + String(tipo) + "," + String(time) + "," + String(intervalo) 
    + "," + String(d1) + "," + String(d2) + "," + String(d3) + ";";
}

//Permite obtener el primer dato de una cadena de texto que se encuentra separada por el delimitador
String getFirst(String cad, String delimiter){
  if (cad.length() == 0)
    return "";
  else if (cad.indexOf(delimiter) == -1)
    return cad;
  else{
    return cad.substring(0, cad.indexOf(delimiter));
  }
}

//Permite eliminar el primer dato de una cadena de texto que se encuentra separada por el delimitador
String removeFirst(String cad, String delimiter){
  if (cad.length() == 0 || cad.indexOf(delimiter) == -1 || cad.length() == cad.indexOf(delimiter) + 1)
    return "";
  else{
    return cad.substring(cad.indexOf(delimiter) + 1);
  }
}

//Permite obtener una cadena de texto a partir de un arreglo de datos de tipo uint8_t
String getStringFromUint8Array(uint8_t* arr, int arraycount, String delimitter){
  if (arraycount == 0)
    return "";
  else{
    String cad = "";
    
    for (int i = 0; i < arraycount; i++){
      if (cad.length() != 0)
        cad += ",";

      cad += String(arr[i]);
    }

    return cad;
  }
}

//Permite obtener un arreglo de datos de tipo uint8_t a partir de una cadena de texto
void getUint8ArrayFromString(uint8_t* arr, String cad, String delimitter){
  int count = countStrings(cad, delimitter);
  String stringArray[count];
  splitString(stringArray, cad, delimitter);

  for (int i = 0; i<count; i++){
    String strValue = stringArray[i];
    arr[i] = (uint8_t)(strValue.toInt());
  }
}

//Permite dividir una cadena de texto en base al delimitador especificado
void splitString(String* cadSp, String cad, String delimiter){
  int count = countStrings(cad, delimiter);
  String copyCat = cad + "";

  for (int i = 0; i < count; i++){
    cadSp[i] = getFirst(copyCat, delimiter);
    copyCat = removeFirst(copyCat, delimiter);
  }
}

//Permite contar las subcadenas que se encuentran separadas por el delimitador en una cadena de texto
int countStrings(String cad, String delimiter){
  if (cad.length() == 0){
    return 0;
  }else if (delimiter.length() == 0 || cad.indexOf(delimiter) == -1){
    return 1;
  }else{
    String cadCopy = "" + cad;
    int count = 0;

    while (cadCopy.length() != 0){
      cadCopy = removeFirst(cadCopy, delimiter);
      count++;
    }

    return count;
  }
}

//Permite obtener una fecha en el formato YYYY-MM-DD
String getFormattedDate(DateTime dt){
  return getAdjustedValue(dt.year()) + '-' + getAdjustedValue(dt.month()) + '-' + getAdjustedValue(dt.day());
}

//Permite obtener la hora en el formato hh:mm:ss
String getFormattedTime(DateTime dt){
  return getAdjustedValue(dt.hour()) + ':' + getAdjustedValue(dt.minute()) + ':' + getAdjustedValue(dt.second());
}

//Permite obtener la fecha y la hora en el formato YYYY-MM-DD hh:mm:ss
String getFormattedDateTime(DateTime dt){
  return getFormattedDate(dt) + " " + getFormattedTime(dt);
}

unsigned long getMillisFromSeconds(int seconds){
  return (unsigned long)seconds * 1000ul;
}

String getAdjustedValue(int val){
  String value = "";

  if (val<10){
    value = "0";
  }

  value += (String)val;

  return value;
}
