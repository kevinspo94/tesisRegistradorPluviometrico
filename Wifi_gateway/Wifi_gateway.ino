#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>
#include <SoftwareSerial.h>
#include <base64.h>

// Replace with your network credentials
const char* ssid     = "Arduino Connection";
const char* password = "ucuenca2020";

//Se inicia un servidor web en el disposito para recibir y responder peticiones
ESP8266WebServer server(80);
//Se inicia la conexion serial del dispositivo con el arduino
SoftwareSerial SerialArduino(D2,D3);

/**
 * Funcion que sirve para inicializar el dispositivo
 */
void setup() {
  delay(1000);
  Serial.begin(9600);
  SerialArduino.begin(9600);

  WiFi.softAP(ssid, password);

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  /***Se asigna los eventos a las rutas que seran escuchadas por el servidor***/

  //Manejar raíz o verificar conexión
  server.on("/", handleRoot);

  // Ruteo para '/files' para poder obtener el listado de archivos disponibles en memoria
  server.on("/files", []() {
    SerialArduino.write("files");

    if (waitForResponse(20000)){
      String fileNames = SerialArduino.readString();
      server.send(200, "text/plain", fileNames);
    }else{
      server.send(504, "text/plain", "Se alcanzó el tiempo de espera para la petición");
    }
  });

  // Ruteo para '/file' para poder obtener un archivo determinado
  server.on("/file", []() {
    int bufferSize = 32;
    String fileName = server.arg(String("filename"));
    String request = "file_" + fileName;

    //server.sendHeader("Content-Length", "20");
    //server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    //server.send(200, "application/octet-stream", "");//
    /*server.sendContent("ABCde");
    server.sendContent("HNHG=");
    server.sendContent("HJIMN");
    server.sendContent("DFE");
    server.sendContent("DF");

    return;*/

    SerialArduino.print(request);

    if (waitForResponse(20000, 4)){
      Serial.println("Available: " + String(SerialArduino.available()));
      
      byte sizeBuf[4];
      size_t n = SerialArduino.readBytes(sizeBuf, 4);
      Serial.println("Data: " + String(n) + " " + String((int)sizeBuf[0])+ String((int)sizeBuf[1])+ String((int)sizeBuf[2])+ String((int)sizeBuf[3]));
      uint32_t totalBytes = 0;
      totalBytes = (uint32_t) sizeBuf[0]<<24;
      totalBytes |= (uint32_t) sizeBuf[1]<<16;
      totalBytes |= (uint32_t) sizeBuf[2]<<8;
      totalBytes |= (uint32_t) sizeBuf[3];
      //String lengthStr = SerialArduino.readStringUntil('*');
      //String str = SerialArduino.readString();

      Serial.println("Length: " + String(totalBytes));
      //Serial.println("Length: " + str);
      
      if (totalBytes==0){
        server.send(204, "text/plain", "La petición no devolvio ningun resultado");  
      }else{
        //int totalBytes = lengthStr.toInt();
        uint32_t leidos = 0;
        boolean first = true;
	      String data = "";

        while (true){
          
          /**int diff = 0;
          
          if (leidos + bufferSize <= totalBytes){
            diff = bufferSize;
          }else{
            diff = totalBytes - leidos;
          }*/

          if (waitForResponse(20000)){

            String str = SerialArduino.readString();
	          data += str;
            leidos += str.length();
            Serial.println("4444");
            //encoded.toCharArray(data, encoded.length()+1);
            
              //server.sendContent_P(data);
              
            if (totalBytes == leidos){
              Serial.println("Terminado: " + String(leidos));
              break;
            }
          }else{
            server.send(504, "text/plain", "Se alcanzó el tiempo de espera para la petición");
            return;
          }          
        }

        String encodedData = base64::encode(data);

        server.setContentLength(CONTENT_LENGTH_UNKNOWN);
        server.send(200, "application/octet-stream", encodedData);
      }

      
    }else{
      server.send(504, "text/plain", "Se alcanzó el tiempo de espera para la petición");
    }
    
    //server.send(200, "text/plain", "Esto tambien funciona");
  });

  // Ruteo para '/first' para poder obtener la primera fecha de registro de datos
  server.on("/first", []() {
    SerialArduino.write("first");

    if (waitForResponse(20000)){
      String first = SerialArduino.readString();
      server.send(200, "text/plain", first);
    }else{
      server.send(504, "text/plain", "Se alcanzó el tiempo de espera para la petición");
    }    
  });

  // Ruteo para '/config' para poder obtener las configuraciones del sistema
  server.on("/config", []() {
    SerialArduino.write("config");

    if (waitForResponse(20000)){
      String config = SerialArduino.readString();
      server.send(200, "text/plain", config);
    }else{
      server.send(504, "text/plain", "Se alcanzó el tiempo de espera para la petición");
    }    
  });

  // Ruteo para '/setconfig' para poder cambiar las configuraciones del sistema
  server.on("/setconfig", []() {
    String data = server.arg(String("data"));
    String request = "config_" + data;

    SerialArduino.print(request);

    if (waitForResponse(20000)){
      String response = SerialArduino.readString();
      
      if (response.equals("OK")){
        server.send(200, "text/plain", "Configuración realizada exitosamente");  
      }else{
        server.send(500, "text/plain", "Error interno");
      }

      
    }else{
      server.send(504, "text/plain", "Se alcanzó el tiempo de espera para la petición");
    }    
  });
  
  server.begin();
  Serial.println("HTTP server started");
}

/**
 * Funcion que se utiliza para esperar una respuesta por un tiempo determinado del sensor
 */
boolean waitForResponse(unsigned long timeOutMillis){
  unsigned long start = millis();  
  
  while (true){
    unsigned long now = millis();  

    if (SerialArduino.available()==0 && (now - start>=timeOutMillis)){
      return false;
    } else if (SerialArduino.available()!=0){
      return true;
    }

    delay(1);
    //yield();
  }

  return false;
}

/**
 * Funcion que se utiliza para esperar una respuesta de bytes por un tiempo determinado del sensor
 */
boolean waitForResponse(unsigned long timeOutMillis, int min){
  unsigned long start = millis();  
  
  while (true){
    unsigned long now = millis();  

    if (SerialArduino.available()<min && (now - start>=timeOutMillis)){
      return false;
    } else if (SerialArduino.available()>=min){
      return true;
    }

    delay(100);
  }

  return false;
}

void loop() {
  server.handleClient();
}

//Funcion que se ejecutara en la URI '/'
void handleRoot() 
{
   server.send(200, "text/plain", "Ok");
}

// Funcion que se ejecutara en URI desconocida
void handleNotFound() 
{
   server.send(404, "text/plain", "Not found");
}
