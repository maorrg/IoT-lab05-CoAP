# Informe Laboratorio 05 - CoAP

[https://github.com/mrg2000/IoT-lab05-CoAP](https://github.com/mrg2000/IoT-lab05-CoAP)

## 1. Identificar las funciones de generación de paquetes

<aside>
💡 Identificar las funciones de generación de paquetes usando la librería [SimpleCoap](https://github.com/hirotakaster/CoAP-simple-library) comentando las líneas de código relevantes y hacer un diagrama de bloques a nivel de llamadas de función del protocolo CoAP.

</aside>

### 1.1. Funciones identificadas en el cliente CoAP

- Constructor `coap`.
- `coap.response`
- `coap.start`
- `coap.loop`

```cpp
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <coap-simple.h>
#include <DHT.h>

/* Definición de variables para el sensor de temperatura. */
#define DHTPIN 4 //Connect Out pin to D2 in NODE MCU
#define DHTTYPE DHT11  
DHT dht(DHTPIN, DHTTYPE);

/* Identificador y clave WiFi. */
const char *ssid = "Casa Trianon"; // Enter your WiFi name
const char *password = "Trianon$2222"; // Enter WiFi password

/* Declaración del callback de CoAP. */
void callback_response(CoapPacket &packet, IPAddress ip, int port);

/* Variables para guardar registro de la temperatura y humedad. */
double temperature = 0;
double humedity = 0;

/* Declaración de variables necesarias para usar CoAP */
WiFiUDP udp;
Coap coap(udp);

/*
La función readSensor() se encarga de leer e imprimir los datos del sensor de temperatura; 
en este caso, la temperatura y la humedad.
*/
void readSensor()
{
    temperature = dht.readTemperature();
    humedity = dht.readHumidity();
    Serial.print("Temperature : ");
    Serial.print(temperature);
    Serial.print("    Humidity : ");
    Serial.println(humedity);
    if (isnan(humedity) || isnan(temperature)) {
      Serial.println("Failed to read from DHT sensor!");
    return;
  }
}

/*
Implementación del callback de CoAP.
En este caso se encarga de imprimir en consola el payload del paquete.
*/
void callback_response(CoapPacket &packet, IPAddress ip, int port) {
  Serial.println("[Coap Response got]");
  
  char p[packet.payloadlen + 1];
  memcpy(p, packet.payload, packet.payloadlen);
  p[packet.payloadlen] = NULL;
  
  Serial.println(p);
}

/*
Función setup de Arduino.
Establece la coneción WiFi, asigna el callback de CoAP usando coap.response
e inicia CoAP con coap.start.
*/
void setup() {
  Serial.begin(115200);
  
  // connecting to a WiFi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  coap.response(callback_response);
  coap.start();
}

/*
Función loop de Arduino.
Lee el sensor cada 2000 ms.
Genera el string message con la humedad y temperatura medidas.
Este string es enviado a través de CoAP usando la función coap.put y coap.loop.
*/
void loop() {
    delay(2000);
    // Read sensor and send message
    readSensor();
    String message = "{\"humidity\":" + String(humedity) + ",\"temperature\":" + String(temperature) + "}";
    int msgid = coap.put(IPAddress(10, 0, 2, 72), 5683, "alarm", message.c_str());
    coap.loop();
}
```

### 1.2. Diagrama de bloques a nivel de llamadas de función del cliente CoAP

En la Figura 1 se ve nuestro diagrama de funciones del cliente.

→ El bloque `setup` se encarga de establecer la conexión WiFi y de asignar el callback a CoAP antes de iniciarlo.

→ El bloque `loop` se encarga de leer la temperatura del sensor y actualizar los valores leídos a través de CoAP.

![Figura 1: Diagrama de bloques a nivel de llamada de función del cliente CoAP.](Informe%20Laboratorio%2005%20-%20CoAP%20c5e75e085cbe40ba871ec012db7afb7b/Untitled_Diagram.drawio_(1).png)

Figura 1: Diagrama de bloques a nivel de llamada de función del cliente CoAP.

### 1.3. Funciones identificadas en el servidor CoAP

- `add_resource`
- `create_server_context`
- `aiocap.resource.Site`

```python
import threading
import asyncio

import aiocoap.resource as resource
import aiocoap

"""
Esta clase se encarga de los PUT y GET requests.
"""
class AlarmResource(resource.ObservableResource):
    def __init__(self):
        super().__init__()

        self.status = "OFF"
        self.has_observers = False
        self.notify_observers = False

    """
    La función notify_observers_check se encarga de notificar
    con un print a los que se encuentran "observando" el recurso.
    """
    def notify_observers_check(self):
        while True:
            if self.has_observers and self.notify_observers:
                print('Notifying observers')
                self.updated_state()
                self.notify_observers = False

    """
    La función update_observation_count se encarga de establecer
    el booleano self.has_observers revisando si count es mayor que 0.
    """
    def update_observation_count(self, count):
        if count:
            self.has_observers = True
        else:
            self.has_observers = False

    """
    La función render_get se encarga de imprimir y retornar el estado guardado en
    self.status (GET request).
    """
    async def render_get(self, request):
        print('Return alarm state: %s' % self.status)
        payload = b'%s' % self.status.encode('ascii')

        return aiocoap.Message(payload=payload)

    """
    La función render_put se encarga de actualizar el estado guardado en self.status.
    Asimismo, imprime el valor actualizado y retorna el valor actualizado.
    """
    async def render_put(self, request):
        self.status = request.payload.decode('ascii')
        print('Update alarm state: %s' % self.status)
        self.notify_observers = True

        return aiocoap.Message(code=aiocoap.CHANGED, payload=b'%s' % self.status.encode('ascii'))
    
def main():
    """
    Definición del recurso. Se asigna una IP y un puerto.
    """
    root = resource.Site()
    alarm_resource = AlarmResource()
    root.add_resource(['alarm'], alarm_resource)
    asyncio.Task(aiocoap.Context.create_server_context(root, bind=('10.0.2.72', 5683)))

    """
    Daemon para notificar a los usuarios si el estado cambia con la función notify_observers_check
    """
    observers_notifier = threading.Thread(target=alarm_resource.notify_observers_check)
    observers_notifier.daemon = True
    observers_notifier.start()

    """
    Correr servidor indefinidamente.
    """
    asyncio.get_event_loop().run_forever()

if __name__ == "__main__":
    main()
```

### 1.4. Diagrama de bloques a nivel de llamadas de función del servidor CoAP

En la Figura 2 se ve nuestro diagrama de funciones del servidor.

→ La clase `class AlarmResource` cuenta con las funciones para el método `GET` y el método `PUT`.

→ La función `main` se encarga de establecer el servidor CoAP añadiendo el recurso de la clase y estableciendo el contexto del servidor (IP y puerto).

![Figura 2: Digrama de bloques a nivel de llamada de función para el servidor CoAP.](Informe%20Laboratorio%2005%20-%20CoAP%20c5e75e085cbe40ba871ec012db7afb7b/Untitled_Diagram.drawio.png)

Figura 2: Digrama de bloques a nivel de llamada de función para el servidor CoAP.

---

## 2. Utilizando un Arduino, comunicar los datos con CoAP

<aside>
💡 Diagrama de conexión hacia el broker. Incluir el código comentado y una captura con el código recibido y enviado.

</aside>

### 2.1. Diagrama de conexión del circuito

Antes de mostrar la conexión entre el cliente y el servidor CoAP, es importante mostrar el circuito de conexión del microcontrolador con el sensor de temperatura y humedad. Como se puede observar en la figura 3, el elemento (1) es el sensor de humedad y temperatura `DHT11` , mientras que el (2) es el microcontrolador `ESP8266` . Respecto a las conexiones realizadas, se puede observar en la figura 4 que la señal (data) del sensor de humedad fue conectado al pin digital `D2` que desde el código fue leído como el pin 4 (información recuperada de la guía de `NODE MCU`).

![Figura 3: Se puede observar un sensor de temperatura y humedad DHT11 (1) conectado a un Arduino ESP8266 (2). ](Informe%20Laboratorio%2005%20-%20CoAP%20c5e75e085cbe40ba871ec012db7afb7b/Captura_de_Pantalla_2022-11-09_a_la(s)_19.43.18.png)

Figura 3: Se puede observar un sensor de temperatura y humedad DHT11 (1) conectado a un Arduino ESP8266 (2). 

### 2.2. Captura del código enviado y recibido

Como se puede observar en las figuras 5 y 6 se puede ver que se envía en código desde el cliente coAP y se recibe en el broker que utiliza la librería `aiocoap`.

![Figura 4: Se observa un esquema que representa al Arduino ESP8266, que se utilizó para la lectura del sensor. Los pines utilizados son los señalados: GND, VCC y DATA.](Informe%20Laboratorio%2005%20-%20CoAP%20c5e75e085cbe40ba871ec012db7afb7b/NodeMCU_V2_v2.png)

Figura 4: Se observa un esquema que representa al Arduino ESP8266, que se utilizó para la lectura del sensor. Los pines utilizados son los señalados: GND, VCC y DATA.

![Figura 5: Monitor serial imprimiendo los mensajes que están siendo enviados al servidor CoAP.](Informe%20Laboratorio%2005%20-%20CoAP%20c5e75e085cbe40ba871ec012db7afb7b/Captura_de_Pantalla_2022-11-27_a_la(s)_22.20.13.png)

Figura 5: Monitor serial imprimiendo los mensajes que están siendo enviados al servidor CoAP.

![Figura 6: Consola del servidor CoAP recibiendo los mensajes enviados por el cliente en la figura 5.](Informe%20Laboratorio%2005%20-%20CoAP%20c5e75e085cbe40ba871ec012db7afb7b/Captura_de_Pantalla_2022-11-27_a_la(s)_22.20.24.png)

Figura 6: Consola del servidor CoAP recibiendo los mensajes enviados por el cliente en la figura 5.

### 2.3. Diagrama de arquitectura de conexión hacia el broker

Finalmente, se puede observar el diagrama de arquitectura de conexión hacia el broker (figura 7). Se puede observar que el sensor `DHT11` se conecta el `ESP8266` que ejecuta el cliente CoAP. Este se comunica con el Broker CoAP mediante el protocolo CoAP. Finalmente, se envían los datos al `backend` para enviar los datos a `Mixpanel` mediante el protocolo `HTTP`.

![Figura 7: Diagrama de conexión entre el cliente y el broker.](Informe%20Laboratorio%2005%20-%20CoAP%20c5e75e085cbe40ba871ec012db7afb7b/Diagrama_sin_titulo.drawio-2.png)

Figura 7: Diagrama de conexión entre el cliente y el broker.

---

## 3. Plataforma REST mostrando el servicio de IoT

<aside>
💡 Plataforma web para mostrar el servicio.

</aside>

Utilizando el servicio [Mixpanel](https://mixpanel.com) se desarrollo la plataforma web para visualizar los datos, como se puede observar en la figura 8.

![Figura 8: Plataforma web con Mixpanel visualizando los datos escuchados por el servicio web. ](Informe%20Laboratorio%2005%20-%20CoAP%20c5e75e085cbe40ba871ec012db7afb7b/Captura_de_Pantalla_2022-11-27_a_la(s)_23.25.42.png)

Figura 8: Plataforma web con Mixpanel visualizando los datos escuchados por el servicio web. 

## Anexo

### Anexo 1.0: Código de cliente CoAP

```cpp
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <coap-simple.h>
#include <DHT.h>

#define DHTPIN 4 //Connect Out pin to D2 in NODE MCU
#define DHTTYPE DHT11  
DHT dht(DHTPIN, DHTTYPE);

const char *ssid = "Casa Trianon"; // Enter your WiFi name
const char *password = "Trianon$2222"; // Enter WiFi password

// CoAP client response callback
void callback_response(CoapPacket &packet, IPAddress ip, int port);

double temperature = 0;
double humedity = 0;

WiFiUDP udp;
Coap coap(udp);

void readSensor()
{
    temperature = dht.readTemperature();
    humedity = dht.readHumidity();
    Serial.print("Temperature : ");
    Serial.print(temperature);
    Serial.print("    Humidity : ");
    Serial.println(humedity);
    if (isnan(humedity) || isnan(temperature)) {
      Serial.println("Failed to read from DHT sensor!");
    return;
  }
}

void callback_response(CoapPacket &packet, IPAddress ip, int port) {
  Serial.println("[Coap Response got]");
  
  char p[packet.payloadlen + 1];
  memcpy(p, packet.payload, packet.payloadlen);
  p[packet.payloadlen] = NULL;
  
  Serial.println(p);
}

void setup() {
  Serial.begin(115200);
  
  // connecting to a WiFi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  coap.response(callback_response);
  coap.start();
}

void loop() {
    delay(2000);
    // Read sensor and send message
    readSensor();
    String message = "{\"humidity\":" + String(humedity) + ",\"temperature\":" + String(temperature) + "}";
    int msgid = coap.put(IPAddress(10, 0, 2, 72), 5683, "alarm", message.c_str());
    coap.loop();
}
```

### Anexo 1.1: Código del servidor CoAP

```python
import threading
import asyncio

import aiocoap.resource as resource
import aiocoap

class AlarmResource(resource.ObservableResource):
    """This resource supports the GET and PUT methods and is observable.
    GET: Return current state of alarm
    PUT: Update state of alarm and notify registered observers
    """

    def __init__(self):
        super().__init__()

        self.status = "OFF"
        self.has_observers = False
        self.notify_observers = False

    # Ensure observers are notify if required
    def notify_observers_check(self):
        while True:
            if self.has_observers and self.notify_observers:
                print('Notifying observers')
                self.updated_state()
                self.notify_observers = False

    # Observers change event callback
    def update_observation_count(self, count):
        if count:
            self.has_observers = True
        else:
            self.has_observers = False

    # Handles GET request or observer notify
    async def render_get(self, request):
        print('Return alarm state: %s' % self.status)
        payload = b'%s' % self.status.encode('ascii')

        return aiocoap.Message(payload=payload)

    # Handles PUT request
    async def render_put(self, request):
        self.status = request.payload.decode('ascii')
        print('Update alarm state: %s' % self.status)
        self.notify_observers = True

        return aiocoap.Message(code=aiocoap.CHANGED, payload=b'%s' % self.status.encode('ascii'))
    
def main():
    # Resource tree creation
    root = resource.Site()
    alarm_resource = AlarmResource()
    root.add_resource(['alarm'], alarm_resource)
    asyncio.Task(aiocoap.Context.create_server_context(root, bind=('10.0.2.72', 5683)))

    # Spawn a daemon to notify observers when alarm status changes
    observers_notifier = threading.Thread(target=alarm_resource.notify_observers_check)
    observers_notifier.daemon = True
    observers_notifier.start()

    asyncio.get_event_loop().run_forever()

if __name__ == "__main__":
    main()
```

### Anexo 1.2: Código del backend del servicio web

```python
from aiocoap import *
from flask import Flask, session
import asyncio
import json
from mixpanel import Mixpanel
mp = Mixpanel("MIXPANEL_API_TOKEN")

def observe_callback(response):
    if response.code.is_successful():
        data_json = json.loads(response.payload)
        humidity = data_json['humidity']
        temperature = data_json['temperature']
        print(humidity, temperature)
        mp.track('humidity', humidity)
        mp.track('temperature', temperature)

async def observer_server():
    global session
    context = await Context.create_client_context()
    request = Message(code=GET)
    request.set_request_uri('coap://10.0.2.72/alarm')
    request.opt.observe = 0
    observation_is_over = asyncio.Future()
    try:
        context_request = context.request(request)
        context_request.observation.register_callback(observe_callback)
        _ = await context_request.response
        exit_reason = await observation_is_over
        print('Observation is over: %r' % exit_reason)
    finally:
        if not context_request.response.done():
            context_request.response.cancel()
        if not context_request.observation.cancelled:
            context_request.observation.cancel()

def observer():
    asyncio.run(observer_server())
            
if __name__ == '__main__':
    observer()
```