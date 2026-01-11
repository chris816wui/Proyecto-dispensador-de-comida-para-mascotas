#define MQTT_MAX_PACKET_SIZE 512
#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h>
#include "HX711.h"
#include <time.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "esp_wifi.h"
#include <Preferences.h>


// Prototipo requerido (Opción 1: declarar antes de usar)
void uidToString(const byte* uid, char* out, size_t outSize);

// ================ PINES =================
#define SS_PIN 5
#define RST_PIN 4

#define LED_VERDE 27
#define LED_ROJO  14

#define SERVO_PUERTA1 25
#define SERVO_PUERTA2 26

#define HX711_DT  32
#define HX711_SCK 33

#define MQTT_BROKER   "10.10.10.166"   // IP de la PC con Mosquitto
#define MQTT_PORT     18830
#define MQTT_CLIENTID "feeder01"
#define TOPIC_EVENTOS "dispensador/feeder01/eventos"
#define TOPIC_CONFIG  "dispensador/feeder01/config"
#define TOPIC_CONFIG_ACK    "dispensador/feeder01/config/ack"
#define TOPIC_CONFIG_STATUS "dispensador/feeder01/config/status"

#define ZONA_MUERTA_G 0.05

#define TOPIC_MASCOTAS "dispensador/feeder01/mascotas"



//WIFI------------------------------------------------------------------------------------
const char* WIFI_SSID ="Microcontroladores";
const char* WIFI_PASS = "raspy123";

WiFiClient espClient;
PubSubClient mqtt(espClient);

unsigned long ultimoEnvioMQTT = 0;
const unsigned long INTERVALO_ENVIO_MQTT_MS = 15000; // 15s (ajusta a 3600000 para 1 hora)

// ================ MODELO ==================
#define UID_SIZE 4
#define MAX_MASCOTAS 5
#define MAX_VENTANAS 3

#define UID_STR_LEN 12  // "AA:BB:CC:DD" + '\0'
#define TS_STR_LEN 20   // "YYYY-MM-DDTHH:MM:SS" + '\0'
#define EVENT_STR_LEN 20

struct VentanaHoraria {
  uint16_t inicio; // minutos del día
  uint16_t fin;
  bool yaAlimentoHoy;
};

struct Mascota {
  byte uid[UID_SIZE];
  char nombre[16];
  float pesoObjetivoKg;
  VentanaHoraria ventanas[MAX_VENTANAS];
  uint8_t numVentanas;
};

Mascota mascotas[MAX_MASCOTAS];
uint8_t numMascotas = 0;

// forward declarations (se usan en el callback)
int buscarMascota(byte *uid);
int buscarMascotaPorUIDStr(const char* uidStr);
bool uidStringToBytes(const char* uidStr, byte* uidOut);

// Añadir prototype para conectarMQTT
bool conectarMQTT();

// MQTT callback forward
void mqttCallback(char* topic, byte* payload, unsigned int length);

// ------------------ Cola de eventos ------------------
#define MAX_EVENTOS 5


typedef enum {
  EVT_DOSIFICANDO,
  EVT_YA_COMIO_HOY,
  EVT_FUERA_HORARIO,
  EVT_UID_NO_REGISTRADO
} EventoTipo;

struct Evento {
  char timestamp[TS_STR_LEN];
  char uid[UID_STR_LEN];
  char evento[EVENT_STR_LEN];
};

static Evento colaEventos[MAX_EVENTOS];
static uint16_t colaHead = 0; // índice del primer elemento válido
static uint16_t colaCount = 0; // cuántos elementos hay
// -----------------------------------------------------

Preferences prefs;
const char* PREF_NAMESPACE = "feeder_cfg";
uint32_t configVersion = 0;


// ================ CONSTANTES =============
const unsigned long TIEMPO_ABIERTO_MS = 200;
const unsigned long TIEMPO_ESTABLE_MS = 700;
const unsigned long LED_ROJO_NO_AUT_MS = 10000;

const unsigned long TIMEOUT_DOSIFICACION_MS = 20000; // 20 s
const float MARGEN_CORTE_ANTICIPADO_KG = 0.002;
const unsigned long LED_VERDE_BLINK_MS = 40;

float CALIBRATION_FACTOR = 1990000.0;


int ultimoDia = -1;
int matchedWindowIndex = -1; // índice de ventana que permitió la validación (por sesión)

// ================ OBJETOS =================
MFRC522 mfrc522(SS_PIN, RST_PIN);
Servo servoPuerta1;
Servo servoPuerta2;
HX711 balanza;

// ================ FSM =====================
enum ResultadoValidacion {
  VALIDACION_OK,
  YA_COMIO_HOY,
  FUERA_DE_HORARIO
};

enum EstadoSistema {
  ESPERANDO_TARJETA,
  VALIDANDO,
  DOSIFICANDO,
  LIBERANDO,
  BLOQUEADO
};

EstadoSistema estadoActual = ESPERANDO_TARJETA;
int indiceMascotaActual = -1;

// ================ UID TEMP ================
byte uidLeido[UID_SIZE];
bool hayUIDLeido = false;

// ================ FUNCIONES ==============
// Guarda mascotas y numMascotas y configVersion en NVS (Preferences)
bool saveConfigToNVS() {
  prefs.begin(PREF_NAMESPACE, false); // RW
  // Guardar número de mascotas
  prefs.putUShort("nmasc", numMascotas);
  // Guardar array de mascotas (solo los numMascotas primeros)
  if (numMascotas > 0) {
    size_t bytes = sizeof(Mascota) * (size_t)numMascotas;
    prefs.putBytes("masc", (const void*)mascotas, bytes);
  } else {
    // eliminar key si no hay mascotas
    prefs.remove("masc");
  }
  prefs.putUInt("cfgver", configVersion);
  prefs.end();
  Serial.printf("Guardado en NVS: numMascotas=%u cfgver=%u\n", (unsigned)numMascotas, (unsigned)configVersion);
  return true;
}

// Carga mascotas y configVersion desde NVS. Devuelve true si había datos.
bool loadConfigFromNVS() {
  prefs.begin(PREF_NAMESPACE, true); // read-only
  if (!prefs.isKey("nmasc")) {
    // nada guardado
    configVersion = prefs.getUInt("cfgver", 0);
    prefs.end();
    Serial.println("NVS: no hay mascotas guardadas");
    return false;
  }
  uint16_t n = prefs.getUShort("nmasc", 0);
  if (n > MAX_MASCOTAS) n = 0; // protección
  numMascotas = n;
  size_t bytes = prefs.getBytesLength("masc");
  if (bytes >= sizeof(Mascota) * (size_t)numMascotas && numMascotas > 0) {
    prefs.getBytes("masc", (void*)mascotas, sizeof(Mascota) * (size_t)numMascotas);
  } else {
    // si no hay bytes válidos, dejamos numMascotas = 0 para evitar inconsistencias
    if (numMascotas > 0) {
      Serial.println("NVS: tamaño de datos inválido, limpiando");
      numMascotas = 0;
    }
  }
  configVersion = prefs.getUInt("cfgver", 0);
  prefs.end();
  Serial.printf("Cargado NVS: numMascotas=%u cfgver=%u\n", (unsigned)numMascotas, (unsigned)configVersion);
  // debug: listar nombres
  for (uint8_t i = 0; i < numMascotas; i++) {
    Serial.printf(" M%i: %s uid %02X:%02X:%02X:%02X\n", i,
                  mascotas[i].nombre,
                  mascotas[i].uid[0], mascotas[i].uid[1], mascotas[i].uid[2], mascotas[i].uid[3]);
  }
  return true;
}

// incrementa version y guarda en NVS (llamar después de modificar)
void bumpConfigVersionAndSave() {
  configVersion++;
  saveConfigToNVS();
}

// Enviar ACK simple por MQTT (topic TOPIC_CONFIG_ACK)
void sendConfigAck(const char* action, const char* uidStr, const char* status) {
  // construir json sencillo
  char buf[128];
  int n = snprintf(buf, sizeof(buf),
                   "{\"action\":\"%s\",\"uid\":\"%s\",\"status\":\"%s\",\"config_version\":%u}",
                   action, uidStr ? uidStr : "", status, (unsigned)configVersion);
  if (n > 0 && n < (int)sizeof(buf)) {
    if (!mqtt.connected()) {
      conectarMQTT(); // intentar reconectar
    }
    if (mqtt.connected()) {
      mqtt.publish(TOPIC_CONFIG_ACK, buf, false);
      Serial.print("ACK enviado: ");
      Serial.println(buf);
    } else {
      Serial.println("No conectado: ACK no enviado");
    }
  } else {
    Serial.println("ACK: buffer overflow");
  }
}

// Publicar estado/config_version al reconectar para que Node-RED decida sincronizar
void publishConfigStatus() {
  char buf[64];
  int n = snprintf(buf, sizeof(buf), "{\"config_version\":%u}", (unsigned)configVersion);
  if (n>0 && n < (int)sizeof(buf)) {
    if (mqtt.connected()) {
      mqtt.publish(TOPIC_CONFIG_STATUS, buf, false);
      Serial.print("Status publicado: "); Serial.println(buf);
    }
  }
}

void publishMascotas() {
  StaticJsonDocument<512> doc;

  doc["config_version"] = configVersion;
  JsonArray arr = doc.createNestedArray("mascotas");

  for (uint8_t i = 0; i < numMascotas; i++) {
    JsonObject m = arr.createNestedObject();

    char uidStr[UID_STR_LEN];
    uidToString(mascotas[i].uid, uidStr, sizeof(uidStr));

    m["uid"] = uidStr;
    m["nombre"] = mascotas[i].nombre;
    m["pesoObjetivoKg"] = mascotas[i].pesoObjetivoKg;

    JsonArray vArr = m.createNestedArray("ventanas");
    for (uint8_t j = 0; j < mascotas[i].numVentanas; j++) {
      JsonObject v = vArr.createNestedObject();
      v["inicio"] = mascotas[i].ventanas[j].inicio;
      v["fin"] = mascotas[i].ventanas[j].fin;
    }
  }

  char buffer[512];
  size_t n = serializeJson(doc, buffer);

  mqtt.publish(TOPIC_MASCOTAS, buffer, n);
  Serial.println("Mascotas publicadas");
}


void conectarWiFi() {
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);

  Serial.print("Conectando a WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED && intentos < 30) {
    delay(500);
    Serial.print(".");
    intentos++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi conectado");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nERROR: No se pudo conectar al WiFi");
  }
}


void configurarHora() {
  configTime(-5 * 3600, 0, "pool.ntp.org"); // Ecuador GMT-5

  struct tm timeinfo;
  Serial.print("Sincronizando hora");
  while (!getLocalTime(&timeinfo)) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("\nHora sincronizada");
}

bool conectarMQTT() {
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);

  Serial.print("Conectando a MQTT...");
  if (mqtt.connect(MQTT_CLIENTID)) {
    Serial.println(" conectado");
    // Suscribirse al topic de configuración al reconectar
    mqtt.subscribe(TOPIC_CONFIG);
    // al reconectar, publicar estado para que el servidor sepa qué versión tiene este dispositivo
    publishConfigStatus();
    publishMascotas();

    Serial.print("Suscrito a: ");
    Serial.println(TOPIC_CONFIG);
    return true;
  } else {
    Serial.print(" fallo, rc=");
    Serial.println(mqtt.state());
    return false;
  }
}

// convierte "AA:BB:CC:DD" -> uid bytes (más tolerante)
bool uidStringToBytes(const char* uidStr, byte* uidOut) {
  unsigned int b0, b1, b2, b3;
  int r = sscanf(uidStr, "%x:%x:%x:%x", &b0, &b1, &b2, &b3);
  if (r != 4) return false;
  uidOut[0] = (byte)b0;
  uidOut[1] = (byte)b1;
  uidOut[2] = (byte)b2;
  uidOut[3] = (byte)b3;
  return true;
}

int buscarMascota(byte *uid) {
  for (int i = 0; i < numMascotas; i++) {
    bool igual = true;
    for (int j = 0; j < UID_SIZE; j++) {
      if (mascotas[i].uid[j] != uid[j]) { igual = false; break; }
    }
    if (igual) return i;
  }
  return -1;
}

int buscarMascotaPorUIDStr(const char* uidStr) {
  byte uidTmp[UID_SIZE];
  if (!uidStringToBytes(uidStr, uidTmp)) return -1;
  return buscarMascota(uidTmp);
}

const char* nombrePorUID(const char* uidStr) {
  for (int i = 0; i < numMascotas; i++) {
    char uidMascota[UID_STR_LEN];
    snprintf(uidMascota, UID_STR_LEN, "%02X:%02X:%02X:%02X",
             mascotas[i].uid[0], mascotas[i].uid[1],
             mascotas[i].uid[2], mascotas[i].uid[3]);
    if (strcmp(uidStr, uidMascota) == 0) {
      return mascotas[i].nombre;
    }
  }
  return "DESCONOCIDO";
}

uint16_t horaActualMin() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return 0;
  return timeinfo.tm_hour * 60 + timeinfo.tm_min;
}

ResultadoValidacion validarVentana(
  const Mascota &m,
  uint16_t horaActual,
  int &indiceVentanaValida
) {
  indiceVentanaValida = -1;
  bool existeVentanaActual = false;

  for (uint8_t i = 0; i < m.numVentanas; i++) {
    const VentanaHoraria &v = m.ventanas[i];

    bool dentroVentana;
    if (v.inicio <= v.fin) {
      dentroVentana = (horaActual >= v.inicio && horaActual <= v.fin);
    } else {
      dentroVentana = (horaActual >= v.inicio || horaActual <= v.fin);
    }

    if (!dentroVentana) continue;

    existeVentanaActual = true;

    if (!v.yaAlimentoHoy) {
      indiceVentanaValida = i;
      return VALIDACION_OK;
    }
  }

  if (existeVentanaActual) {
    return YA_COMIO_HOY;
  } else {
    return FUERA_DE_HORARIO;
  }
}

// ---------- SERVOS (MOVIMIENTO SUAVE) ----------
void abrirPuerta1Lento() {
  for (int angulo = 90; angulo >= 45; angulo--) {
    servoPuerta1.write(angulo);
    delay(10);   // 
  }
}

void cerrarPuerta1Lento() {
  for (int angulo = 45; angulo <= 90; angulo++) {
    servoPuerta1.write(angulo);
    delay(10);
  }
}

void abrirPuerta2()  { servoPuerta2.write(135); }
void cerrarPuerta2() { servoPuerta2.write(45); }

float leerPesoKg() {
  float peso= balanza.get_units(10);
  if (abs(peso) < ZONA_MUERTA_G) peso = 0.0;
  peso = round(peso * 10.0) / 10.0;

  return peso; // promedio 10 muestras
}

void imprimirUID(byte *uid) {
  Serial.print("UID leído: ");
  for (byte i = 0; i < UID_SIZE; i++) {
    Serial.print("0x");
    if (uid[i] < 0x10) Serial.print("0");
    Serial.print(uid[i], HEX);
    if (i < UID_SIZE - 1) Serial.print(", ");
  }
  Serial.println();
}

void activarServos() {
  servoPuerta1.attach(SERVO_PUERTA1, 500, 2400);
  servoPuerta2.attach(SERVO_PUERTA2, 500, 2400);
  cerrarPuerta1Lento();
  cerrarPuerta2();
  delay(50);
}

void desactivarServos() {
  servoPuerta1.detach();
  servoPuerta2.detach();
}

// Implementación de uidToString (Opción 1)
void uidToString(const byte* uid, char* out, size_t outSize) {
  if (outSize == 0) return;
  // Formato "AA:BB:CC:DD" -> máximo 11 chars + '\0' => UID_STR_LEN == 12
  // Protegemos buffer y garantizamos terminador.
  if (outSize < UID_STR_LEN) {
    // intentar escribir lo que quepa sin overflow
    int written = snprintf(out, outSize, "%02X:%02X:%02X:%02X",
                           uid[0], uid[1], uid[2], uid[3]);
    // Asegurar terminador
    out[outSize - 1] = '\0';
    (void)written;
    return;
  }
  // Buffer suficientemente grande
  snprintf(out, outSize, "%02X:%02X:%02X:%02X",
           uid[0], uid[1], uid[2], uid[3]);
  out[outSize - 1] = '\0';
}

// Obtiene timestamp ISO sin zona: "YYYY-MM-DDTHH:MM:SS"
bool makeIsoTimestamp(char *buf, size_t len) {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return false;
  snprintf(buf, len, "%04d-%02d-%02dT%02d:%02d:%02d",
           timeinfo.tm_year + 1900,
           timeinfo.tm_mon + 1,
           timeinfo.tm_mday,
           timeinfo.tm_hour,
           timeinfo.tm_min,
           timeinfo.tm_sec);
  return true;
}

// Encola evento. Devuelve true si fue encolado, false si cola llena.
bool encolarEvento(const byte *uidBytes, EventoTipo tipo) {
  if (colaCount >= MAX_EVENTOS) {
    Serial.println("WARN: cola de eventos llena, evento descartado");
    return false;
  }

  uint16_t index = (colaHead + colaCount) % MAX_EVENTOS;
  Evento &e = colaEventos[index];

  // timestamp
  if (!makeIsoTimestamp(e.timestamp, sizeof(e.timestamp))) {
    unsigned long ms = millis()/1000;
    unsigned long hh = (ms / 3600) % 24;
    unsigned long mm = (ms / 60) % 60;
    unsigned long ss = ms % 60;
    snprintf(e.timestamp, sizeof(e.timestamp), "1970-01-01T%02lu:%02lu:%02lu", hh, mm, ss);
  }

  // uid -> string con ":" (usa la función implementada)
  uidToString(uidBytes, e.uid, sizeof(e.uid));

  // evento string
  switch (tipo) {
    case EVT_DOSIFICANDO:      strncpy(e.evento, "DOSIFICANDO", sizeof(e.evento)); break;
    case EVT_YA_COMIO_HOY:     strncpy(e.evento, "YA_COMIO_HOY", sizeof(e.evento)); break;
    case EVT_FUERA_HORARIO:    strncpy(e.evento, "FUERA_HORARIO", sizeof(e.evento)); break;
    case EVT_UID_NO_REGISTRADO: strncpy(e.evento, "UID_NO_REGISTRADO", sizeof(e.evento)); break;
    default:                   strncpy(e.evento, "UNKNOWN", sizeof(e.evento));
  }
  e.evento[EVENT_STR_LEN-1] = '\0';

  colaCount++;
  return true;
}

void vaciarCola() {
  colaHead = 0;
  colaCount = 0;
}

String colaToJsonBatch(const char *deviceId) {
  if (colaCount == 0) return String();

  String s;
  s.reserve(colaCount * 120);
  s += "{\"device\":\""; s += deviceId; s += "\",\"eventos\":[";

  for (uint16_t i = 0; i < colaCount; i++) {
    uint16_t idx = (colaHead + i) % MAX_EVENTOS;
    Evento &e = colaEventos[idx];

    const char* nombreMascota = nombrePorUID(e.uid);

    s += "{\"fecha\":\""; s += String(e.timestamp).substring(0,10); s += "\"";
    s += ",\"hora\":\""; s += String(e.timestamp).substring(11,19); s += "\"";
    s += ",\"mascota\":\""; s += nombreMascota; s += "\"";
    s += ",\"evento\":\""; s += e.evento; s += "\"}";

    if (i < colaCount - 1) s += ",";
  }

  s += "]}";
  return s;
}

// ---------------- MQTT: envío individual ----------------
bool publishEventoIndividual(const Evento &e) {
  char payload[256];
  const char *masc = nombrePorUID(e.uid);
  int n = snprintf(payload, sizeof(payload),
                   "{\"fecha\":\"%.10s\",\"hora\":\"%s\",\"mascota\":\"%s\",\"evento\":\"%s\"}",
                   e.timestamp,
                   &e.timestamp[11],
                   masc,
                   e.evento);
  if (n < 0 || n >= (int)sizeof(payload)) {
    Serial.println("Payload demasiado largo para evento individual");
    return false;
  }

  if (!mqtt.connected()) {
    Serial.println("MQTT desconectado al intentar publicar evento individual, intentando reconectar...");
    conectarMQTT();
    if (!mqtt.connected()) {
      Serial.println("No se pudo reconectar MQTT");
      return false;
    }
  }

  mqtt.loop(); // procesar pings/etc

  bool ok = mqtt.publish(TOPIC_EVENTOS, payload, false);
  if (!ok) {
    Serial.print("Publish evento individual falló, mqtt.state()=");
    Serial.println(mqtt.state());
    Serial.print("FreeHeap: "); Serial.println(ESP.getFreeHeap());
    Serial.print("Payload len: "); Serial.println(strlen(payload));
  }
  return ok;
}

void enviarColaPorEventos() {
  while (colaCount > 0) {
    uint16_t idx = colaHead % MAX_EVENTOS;
    Evento &e = colaEventos[idx];

    mqtt.loop();
    if (publishEventoIndividual(e)) {
      colaHead = (colaHead + 1) % MAX_EVENTOS;
      colaCount--;
    } else {
      Serial.println("Fallo al publicar evento, preservando cola");
      break;
    }
  }
}

// ---------------- MQTT callback ---------------------------------------------------------------------------------------
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Parse JSON (payload no está null-terminated)
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.print("Config JSON invalido: ");
    Serial.println(err.c_str());
    return;
  }

  const char* action = doc["action"];
  if (!action) {
    Serial.println("Config JSON sin campo 'action'");
    // no podemos ACKear porque no sabemos la acción; solo logueamos
    return;
  }

  if (strcmp(action, "get_mascotas") == 0) {
  publishMascotas();
  return;
  }


  // -------------------- DELETE --------------------
  if (strcmp(action, "delete") == 0) {
    const char* uidStr = doc["uid"];
    if (!uidStr) {
      // ACK con error: falta UID
      sendConfigAck("delete", "", "ERROR: uid_missing");
      return;
    }

    int idx = buscarMascotaPorUIDStr(uidStr);
    if (idx < 0) {
      sendConfigAck("delete", uidStr, "ERROR: not_found");
      return;
    }

    // Compactar array (eliminar elemento)
    for (int i = idx; i < (int)numMascotas - 1; i++) {
      mascotas[i] = mascotas[i + 1];
    }
    if (numMascotas > 0) numMascotas--;

    // Persistir y confirmar
    bumpConfigVersionAndSave();                // incrementa configVersion y guarda en NVS
    sendConfigAck("delete", uidStr, "OK");    // enviar ACK de éxito

    Serial.printf("Mascota %s eliminada. numMascotas=%u\n", uidStr, (unsigned)numMascotas);
    return;
  }

  // -------------------- UPSERT (create or update) --------------------
  if (strcmp(action, "upsert") == 0) {
    // obtenemos el objeto mascota
    JsonVariant mv = doc["mascota"];
    if (mv.isNull()) {
      sendConfigAck("upsert", "", "ERROR: no_mascota");
      return;
    }

    const char* uidStr = mv["uid"];
    if (!uidStr) {
      sendConfigAck("upsert", "", "ERROR: uid_missing");
      return;
    }

    // convertir uid string a bytes
    byte uidBytes[UID_SIZE];
    if (!uidStringToBytes(uidStr, uidBytes)) {
      sendConfigAck("upsert", uidStr, "ERROR: uid_invalid");
      return;
    }

    // buscar si existe
    int idx = buscarMascota(uidBytes);
    bool nueva = false;
    if (idx < 0) {
      if (numMascotas >= MAX_MASCOTAS) {
        sendConfigAck("upsert", uidStr, "ERROR: max_mascotas");
        return;
      }
      idx = numMascotas++;
      nueva = true;
    }

    // Referencia a la mascota en RAM
    Mascota &mascota = mascotas[idx];
    memcpy(mascota.uid, uidBytes, UID_SIZE);

    // nombre (opcional)
    if (mv.containsKey("nombre")) {
      const char* name = (const char*)mv["nombre"];
      if (name) {
        strncpy(mascota.nombre, name, sizeof(mascota.nombre));
        mascota.nombre[sizeof(mascota.nombre)-1] = '\0';
      }
    }

    // peso objetivo (opcional)
    if (mv.containsKey("pesoObjetivoKg")) {
      float p = mv["pesoObjetivoKg"].as<float>();
      // opcional: validar rango sensato del peso
      if (p >= 0.0f && p < 10.0f) { // ejemplo: <10kg razonable
        mascota.pesoObjetivoKg = p;
      } else {
        // si el peso es inválido, rechazamos con ACK y no aplicamos cambios
        sendConfigAck("upsert", uidStr, "ERROR: peso_invalid");
        // si creamos la entrada y no queremos dejarla vacía, decrementamos numMascotas
        if (nueva && numMascotas > 0) numMascotas--;
        return;
      }
    }

    // ventanas (opcional) - validamos 0..1439
    if (mv.containsKey("ventanas")) {
      JsonArray arr = mv["ventanas"].as<JsonArray>();
      uint8_t count = 0;
      for (JsonObject v : arr) {
        if (count >= MAX_VENTANAS) break;
        // usar default 0 si no existe, pero validar rangos
        uint32_t inicio = v["inicio"] | 0;
        uint32_t fin    = v["fin"]    | 0;
        if (inicio > 1439 || fin > 1439) {
          // ignorar ventana inválida (podrías también truncar o enviar error)
          Serial.printf("Ventana ignorada por rango invalido: inicio=%u fin=%u\n", (unsigned)inicio, (unsigned)fin);
          continue;
        }
        mascota.ventanas[count].inicio = (uint16_t)inicio;
        mascota.ventanas[count].fin    = (uint16_t)fin;
        mascota.ventanas[count].yaAlimentoHoy = false;
        count++;
      }
      mascota.numVentanas = count;
    }

    // persistir y confirmar
    bumpConfigVersionAndSave();
    sendConfigAck("upsert", uidStr, "OK");

    Serial.printf("%s mascota %s (idx=%d). numMascotas=%u\n", (nueva ? "Agregada":"Actualizada"), uidStr, idx, (unsigned)numMascotas);
    return;
  }

  // -------------------- acción desconocida --------------------
  Serial.print("Accion desconocida: ");
  Serial.println(action);
  // opcional: ACK de acción desconocida (si quieres)
  sendConfigAck(action, "", "ERROR: unknown_action");
}


// ================ SETUP ===================
void setup() {
  Serial.begin(115200);

  conectarWiFi();
  configurarHora();

  pinMode(LED_VERDE, OUTPUT);
  pinMode(LED_ROJO, OUTPUT);
  digitalWrite(LED_VERDE, LOW);
  digitalWrite(LED_ROJO, LOW);

  // SPI y RC522: dejamos antena OFF por defecto; se enciende solo en ESPERANDO_TARJETA
  SPI.begin();
  mfrc522.PCD_Init();
  mfrc522.PCD_AntennaOff();

  // Reservar timers para servos y fijar periodo (50 Hz).
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servoPuerta1.setPeriodHertz(50);
  servoPuerta2.setPeriodHertz(50);

  // HX711: inicializar, calibrar y apagar (power_down real)
  balanza.begin(HX711_DT, HX711_SCK);
  balanza.set_scale(CALIBRATION_FACTOR);
  balanza.tare();
  balanza.power_down();

  // Ejemplo en RAM
  Mascota m1;
  m1.uid[0]=0x15; m1.uid[1]=0x57; m1.uid[2]=0xA9; m1.uid[3]=0xB1;
  strncpy(m1.nombre, "Firulais", sizeof(m1.nombre));
  m1.pesoObjetivoKg = 0.020;
  m1.ventanas[0] = {7*60, 12*60+40, false};
  m1.ventanas[1] = {10*60, 17*60 + 30, false};
  m1.ventanas[2] = {18*60, 19*60, false};
  m1.numVentanas = 3;
  mascotas[numMascotas++] = m1;

  Mascota m2;
  m2.uid[0]=0x1C; m2.uid[1]=0xE4; m2.uid[2]=0x00; m2.uid[3]=0x39;
  strncpy(m2.nombre, "Pelusa", sizeof(m2.nombre));
  m2.pesoObjetivoKg = 0.020;
  m2.ventanas[0] = {7*60, 12*60+40, false};
  m2.ventanas[1] = {12*60 + 30, 13*60 + 30, false};
  m2.ventanas[2] = {18*60, 23*60, false};
  m2.numVentanas = 3;
  mascotas[numMascotas++] = m2;

  Serial.println("Setup terminado. Esperando tarjeta...");

  // registrar callback antes de conectar para que onConnect lo mantenga si reconectamos
  mqtt.setCallback(mqttCallback);
  mqtt.setKeepAlive(120);   // 120 segundos
  // cargar configuración guardada (si existe)
  loadConfigFromNVS();
  conectarMQTT();
}

// ================ LOOP ====================
void loop() {
  // Detectar nuevo día para reset diario
  struct tm timeinfo_now;
  if (getLocalTime(&timeinfo_now)) {
    if (timeinfo_now.tm_yday != ultimoDia) {
      ultimoDia = timeinfo_now.tm_yday;
      for (uint8_t m = 0; m < numMascotas; m++) {
        for (uint8_t v = 0; v < mascotas[m].numVentanas; v++) {
          mascotas[m].ventanas[v].yaAlimentoHoy = false;
        }
      }
      Serial.println("Nuevo dia detectado -> ventanas reseteadas");
    }
  }

  switch (estadoActual) {
    case ESPERANDO_TARJETA: {
      mfrc522.PCD_AntennaOn();
      if (!mfrc522.PICC_IsNewCardPresent()) break;
      if (!mfrc522.PICC_ReadCardSerial()) break;

      for (byte i = 0; i < UID_SIZE; i++) uidLeido[i] = mfrc522.uid.uidByte[i];
      hayUIDLeido = true;
      mfrc522.PICC_HaltA();
      mfrc522.PCD_StopCrypto1();
      mfrc522.PCD_AntennaOff();

      indiceMascotaActual = buscarMascota(uidLeido);
      matchedWindowIndex = -1;
      estadoActual = VALIDANDO;
      break;
    }

    case VALIDANDO: {
      uint16_t hora = horaActualMin();
      Serial.print("Hora actual (min): ");
      Serial.println(hora);

      if (!hayUIDLeido) { estadoActual = ESPERANDO_TARJETA; break; }

      if (indiceMascotaActual < 0 || indiceMascotaActual >= numMascotas) {
        Serial.println("UID NO REGISTRADO");
        imprimirUID(uidLeido);
        encolarEvento(uidLeido, EVT_UID_NO_REGISTRADO);
        hayUIDLeido = false;
        estadoActual = BLOQUEADO;
        break;
      }

      int idx = -1;
      ResultadoValidacion res = validarVentana(mascotas[indiceMascotaActual], hora, idx);

      if (res == VALIDACION_OK) {
        matchedWindowIndex = idx;
        Serial.print("Validado. Ventana index: ");
        Serial.println(matchedWindowIndex);
        encolarEvento(uidLeido, EVT_DOSIFICANDO);
        estadoActual = DOSIFICANDO;
        break;
      }

      if (res == YA_COMIO_HOY) {
        Serial.println(" La mascota YA COMIÓ en esta ventana hoy");
        encolarEvento(uidLeido, EVT_YA_COMIO_HOY);
      } else if (res == FUERA_DE_HORARIO) {
        Serial.println(" Fuera del horario de alimentación");
        encolarEvento(uidLeido, EVT_FUERA_HORARIO);
      }

      hayUIDLeido = false;
      estadoActual = BLOQUEADO;
      break;
    }

    case DOSIFICANDO: {
      digitalWrite(LED_VERDE, HIGH);
      activarServos();
      balanza.power_up();
      delay(120);

      float objetivo = mascotas[indiceMascotaActual].pesoObjetivoKg;
      unsigned long tInicio = millis();

      while (true) {
        if (!mqtt.connected()) {
          Serial.println("MQTT desconectado durante dosificación, intentando reconectar...");
          conectarMQTT();
        }
        mqtt.loop();

        abrirPuerta1Lento();
        delay(TIEMPO_ABIERTO_MS);
        cerrarPuerta1Lento();

        mqtt.loop();
        delay(TIEMPO_ESTABLE_MS);

        float peso = leerPesoKg();
        Serial.print("Peso: ");
        Serial.println(peso, 3);

        if (peso >= (objetivo - MARGEN_CORTE_ANTICIPADO_KG)) {
          Serial.println("Peso objetivo alcanzado.");
          break;
        }
        if (millis() - tInicio > TIMEOUT_DOSIFICACION_MS) {
          Serial.println("Timeout de dosificación.");
          break;
        }
      }

      if (matchedWindowIndex >= 0 && matchedWindowIndex < mascotas[indiceMascotaActual].numVentanas) {
        mascotas[indiceMascotaActual].ventanas[matchedWindowIndex].yaAlimentoHoy = true;
        Serial.print("Marcada ventana "); Serial.print(matchedWindowIndex); Serial.println(" como ya alimentada.");
      } else {
        uint16_t hora_fin = horaActualMin();
        for (uint8_t i = 0; i < mascotas[indiceMascotaActual].numVentanas; i++) {
          VentanaHoraria &v = mascotas[indiceMascotaActual].ventanas[i];
          if (v.inicio <= v.fin) {
            if (hora_fin >= v.inicio && hora_fin <= v.fin) { v.yaAlimentoHoy = true; break; }
          } else {
            if (hora_fin >= v.inicio || hora_fin <= v.fin) { v.yaAlimentoHoy = true; break; }
          }
        }
        Serial.println("Marcado por fallback (ventana encontrada por hora).");
      }
      matchedWindowIndex = -1;

      balanza.power_down();
      estadoActual = LIBERANDO;
      break;
    }

    case LIBERANDO: {
      digitalWrite(LED_VERDE, LOW);
      abrirPuerta2();
      delay(5000);
      cerrarPuerta2();
      delay(600);
      desactivarServos();

      hayUIDLeido = false;
      indiceMascotaActual = -1;
      estadoActual = ESPERANDO_TARJETA;
      break;
    }

    case BLOQUEADO: {
      digitalWrite(LED_ROJO, HIGH);
      delay(LED_ROJO_NO_AUT_MS);
      digitalWrite(LED_ROJO, LOW);

      if (indiceMascotaActual < 0 || indiceMascotaActual >= numMascotas) {
        Serial.println("UID NO REGISTRADO");
        imprimirUID(uidLeido);
      }

      hayUIDLeido = false;
      indiceMascotaActual = -1;
      estadoActual = ESPERANDO_TARJETA;
      break;
    }
  } // switch

  // Mantener MQTT y procesar loop
  if (!mqtt.connected()) {
    Serial.println("MQTT desconectado, intentando reconectar...");
    conectarMQTT();
  }
  mqtt.loop();

  if (millis() - ultimoEnvioMQTT >= INTERVALO_ENVIO_MQTT_MS) {
    ultimoEnvioMQTT = millis();

    if (mqtt.connected()) {
      enviarColaPorEventos();
      Serial.println("Datos enviados");
    } else {
      Serial.println("MQTT sigue desconectado, no se pudo enviar");
    }
  }
}
