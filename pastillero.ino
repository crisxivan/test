#include <Arduino.h>
#include <ESP32Time.h>
#include <time.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ---------- BEGIN DEBUG ---------- //
#define SERIAL_DEBUG_ENABLED 1

#if SERIAL_DEBUG_ENABLED
  #define DebugPrint(str)\
    {\
      Serial.println(str);\
    }
#else
  #define DebugPrint(str)
#endif

#define DebugPrintState(st, ev)\
  {\
    String est = st;\
    String evt = ev;\
    String str;\
    str = "-----------------------------------------------------";\
    DebugPrint(str);\
    str = "EST-> [" + est + "]: " + "EVT-> [" + evt + "].";\
    DebugPrint(str);\
    str = "-----------------------------------------------------";\
    DebugPrint(str);\
  }
// ---------- END DEBUG ---------- //

// ---------- BEGIN STATE MACHINE ---------- //
#define MAX_STATES        5
#define MAX_EVENTS        8
#define MAX_TYPE_EVENTS   7

enum states {ST_INIT, ST_IDLE, ST_ROT, ST_DSP, ST_ERR} current_state;
String s_states [] = {"ST_INIT", "ST_IDLE", "ST_ROTATING", "ST_DISPATCHING", "ST_ERROR"};

enum events {EV_CONT, EV_CFG, EV_IN, EV_TMATCH, EV_END, EV_TOUT, EV_BPRESS, EV_VOL} new_event;
String s_events [] = {"EV_CONT", "EV_CONFIG", "EV_INPUT", "EV_TIME_MATCH", "EV_END_CARRY", "EV_TIMEOUT", "EV_BUTTON_PRESSED", "EV_VOLUMEN"};

void init_();
void match();
void rotate();
void dispatch();
void success();
void error();
void unnotified();
void volumen();
void reset();
void none();

typedef void (*transition)();
transition state_table[MAX_STATES][MAX_EVENTS] = {
  {none,     init_,     none,       none,       none,     none,         none,       none},          // ST_INIT
  {none,     none,      match,      rotate,     none,     none,         none,       volumen},       // ST_IDLE
  {none,     none,      none,       none,       dispatch, error,        none,       none},          // ST_ROTATING
  {none,     none,      none,       none,       none,     unnotified,   success,    none},          // ST_DISPATCHING
  {none,     none,      none,       none,       none,     none,         reset,      none}           // ST_ERROR
//EV_CONT    EV_CFG,    EV_INPUT    EV_TMATCH   EV_END    EV_TOUT       EV_BPRESS   EV_VOL
};

bool wifi_sensor(unsigned long ct);
bool input_listener(unsigned long ct);
bool scheduled(unsigned long ct);
bool end_carry_sensor(unsigned long ct);
bool timer_sensor(unsigned long ct);
bool button_sensor(unsigned long ct);
bool pot_sensor(unsigned long ct);

typedef bool (*eventType)(unsigned long ct);
eventType event_type[MAX_TYPE_EVENTS] = {wifi_sensor, input_listener, scheduled, end_carry_sensor, timer_sensor, button_sensor, pot_sensor};
// ---------- END STATE MACHINE ---------- //

// ---------- BEGIN CONSTANTS ---------- //
#define PIN_BUZZER            32  
#define PIN_POT               34 
#define PIN_LED_SPEED         26  
#define PIN_LED_MOTOR         25 
#define PIN_BTN               14 
#define PIN_BTN_END           4

#define BUZZER_IN_MIN         0
#define BUZZER_IN_MAX         4095
#define BUZZER_OUT_MIN        100
#define BUZZER_OUT_MAX        2000

#define SPEED_MOTOR           80
//#define PWM_FREQUENCY       5000
//#define PWM_RESOLUTION      8          

#define UMBRAL_DIFF_NEW_EVENT 200
#define UMBRAL_DIFF_TIMES     5
#define UMBRAL_COUNTDOWN      10
#define UMBRAL_DIFF_VOL       50

#define TS_CHANGE_STATE       200
#define TS_COUNTDOWN          1000

#define GM_OFFSET_ARG         -10800
#define DAY_LIGHT_OFFSET      0 

const char* ssid = "Wokwi-GUEST";
const char* password = "";

const char* mqtt_server = "industrial.api.ubidots.com";
const int mqtt_port = 1883;
const char* mqtt_user = "BBUS-LoruLGYMGNGH4qFXgB0qkgn8FYsfx2";
const char* mqtt_passwd = "";
const char* topic_fecha = "/v1.6/devices/esp32/fecha";
const char* topic_volume = "/v1.6/devices/esp32/volume";

const long gmtOffset_sec = GM_OFFSET_ARG;
const int daylightOffset_sec = DAY_LIGHT_OFFSET;
const char* ntpServer = "pool.ntp.org";

#define OFFSET_YEAR 1900
#define OFFSET_MONTH 1
#define CONFIG_TIME -1
// ---------- END CONSTANTS ---------- //

// ---------- BEGIN VARIABLES ---------- //
bool timeout_new_event;
long lct;
short last_index_type_sensor  = 0;
int countdown                 = 0;
int pot_value                 = 0;
int pot_value_new             = 0;

struct tm user_time;
time_t user_timestamp;
ESP32Time rtc;

WiFiClient espClient;
PubSubClient client(espClient);

bool mqtt_input_ready = false;

QueueHandle_t queue_motor;

long ts_wifi                  = 0;
long ts_input                 = -1;
long ts_check_time            = -1;
long ts_end_carry             = -1;
long ts_timer                 = -1;
long ts_button                = -1;
long ts_pot                   = -1;
// ---------- END VARIABLES ---------- //

// ---------- BEGIN FUNCTIONS ---------- //
bool is_valid_timestamp();
void reconnectMQTT();
void toggle_motor(void *param);
void callback(char* topic, byte* payload, unsigned int length);
void motor_on();
void motor_off();
void buzzer_on();
void buzzer_off();
// ---------- END FUNCTIONS ---------- //

// ---------- BEGIN ACTIONS ---------- //
void init_()
{
  ts_wifi = -1;
  ts_input = 0;
  ts_pot = 0;

  DebugPrint("Ingrese fecha/hora de la toma. (YYYY-MM-DD HH:MM:SS)");

  current_state = ST_IDLE;
}

void match()
{
  ts_input = -1;
  ts_check_time = 0;
}

void rotate()
{
  motor_on();

  ts_check_time = -1;
  ts_pot = -1;
  ts_end_carry = 0;
  ts_timer = 0;
  countdown = 0;

  current_state = ST_ROT;
  new_event = EV_CONT;
}

void dispatch()
{
  motor_off();
  buzzer_on();

  ts_end_carry = -1;
  ts_timer = 0;
  countdown = 0;
  ts_button = 0;

  current_state = ST_DSP;
  new_event = EV_CONT;
}

void success()
{
  buzzer_off();

  ts_timer = -1;
  ts_button = -1;
  ts_input = 0;
  ts_pot = 0;

  DebugPrint("Ingrese fecha/hora de la toma. (YYYY-MM-DD HH:MM:SS)");

  current_state = ST_IDLE;
}

void unnotified()
{
  buzzer_off();

  ts_timer = -1;
  ts_button = -1;
  ts_input = 0;
  ts_pot = 0;

  DebugPrint("Ingrese fecha/hora de la toma. (YYYY-MM-DD HH:MM:SS)");

  current_state = ST_IDLE;
}

void error()
{
  motor_off();

  ts_timer = -1;
  ts_end_carry = -1;
  ts_button = 0;
  
  current_state = ST_ERR;
}

void volumen()
{
  reconnectMQTT();
  String volume = String(pot_value_new);
  client.publish(topic_volume, volume.c_str());

  pot_value = pot_value_new;

  DebugPrint(pot_value);
}

void reset()
{
  ts_input = 0;
  ts_pot = 0;
  ts_timer = -1;
  ts_button = -1;

  DebugPrint("Ingrese fecha/hora de la toma. (YYYY-MM-DD HH:MM:SS)");

  current_state = ST_IDLE;
}

void none()
{
}

void do_init()
{
  Serial.begin(115200);
  
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_POT, INPUT);
  pinMode(PIN_LED_MOTOR, OUTPUT);
  pinMode(PIN_LED_SPEED, OUTPUT);
  pinMode(PIN_BTN, INPUT);
  pinMode(PIN_BTN_END, INPUT_PULLUP);

  WiFi.begin(ssid, password);

  queue_motor = xQueueCreate(10, sizeof(int));
  xTaskCreate(
    toggle_motor,
    "toggle_motor",
    1024,              
    NULL,             
    1,                
    NULL
  );

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  analogWrite(PIN_LED_SPEED, SPEED_MOTOR);

  pot_value = analogRead(PIN_POT);

  timeout_new_event = false;
  lct = millis();

  DebugPrint("SETUP configurado correctamente.");

  current_state = ST_INIT;
  new_event = EV_CONT;

  DebugPrintState(s_states[current_state], s_events[new_event]);
}
// ---------- END ACTIONS ---------- //

// ---------- BEGIN EVENTS ---------- //
bool wifi_sensor(unsigned long ct)
{
  if(ts_wifi >= 0)
  {
    int diff = (ct - ts_wifi);
    
    if(diff >= TS_CHANGE_STATE)
    {
      ts_wifi = ct;

      if (WiFi.status() == WL_CONNECTED) 
      {
        struct tm time_system;
        configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
        
        if (getLocalTime(&time_system)) 
        {
          rtc.setTime(mktime(&time_system));
          DebugPrint("Horario del sistema configurado.");

          new_event = EV_CFG;
          return true;
        } 
      }
    }
  }

  return false;
}

bool input_listener(unsigned long ct)
{
  if(ts_input >= 0)
  {
    int diff = (ct - ts_input);
    
    if(diff >= TS_CHANGE_STATE)
    {
      ts_input = ct;

      reconnectMQTT();

      client.loop();

      if(mqtt_input_ready)
      {
        if(is_valid_timestamp())
        {
          DebugPrint("Fecha y hora almacenadas correctamente.");
          mqtt_input_ready = false;

          new_event = EV_IN;
          return true;
        }
      }
    }
  }

  return false;
}

bool scheduled(unsigned long ct)
{
  if(ts_check_time >= 0)
  {
    int diff = (ct - ts_check_time);
    
    if(diff >= TS_CHANGE_STATE)
    {
      ts_check_time = ct;

      time_t now = rtc.getEpoch();

      int dt = (now - user_timestamp);

      DebugPrint(dt);
      
      if(abs(dt) < UMBRAL_DIFF_TIMES)
      {
        new_event = EV_TMATCH;
        return true;
      }
    }
  }

  return false;
}

bool end_carry_sensor(unsigned long ct)
{
  if(ts_end_carry >= 0)
  {
    int diff = (ct - ts_end_carry);
    
    if(diff >= TS_CHANGE_STATE)
    {
      ts_end_carry = ct;

      int state = digitalRead(PIN_BTN_END);

      if(state == HIGH)
      {
        new_event = EV_END;
        return true;
      }
    }
  }

  return false;
}

bool timer_sensor(unsigned long ct)
{
  if(ts_timer >= 0)
  {
    int diff = (ct - ts_timer);
    
    if(diff >= TS_COUNTDOWN)
    {
      ts_timer = ct;
      countdown += 1;

      if(countdown >= UMBRAL_COUNTDOWN)
      {
        new_event = EV_TOUT;
        return true;
      }
    }
  }

  return false;
}

bool button_sensor(unsigned long ct)
{
  if(ts_button >= 0)
  {
    int diff = (ct - ts_button);
    
    if(diff >= TS_CHANGE_STATE)
    {
      ts_button = ct;

      int state = digitalRead(PIN_BTN);

      if(state == HIGH)
      {
        new_event = EV_BPRESS;
        return true;
      }
    }
  }

  return false;
}

bool pot_sensor(unsigned long ct)
{
  if(ts_pot >= 0)
  {
    int diff = (ct - ts_pot);
    
    if(diff >= TS_CHANGE_STATE)
    {
      ts_pot = ct;

      int pot_value_now = analogRead(PIN_POT); 

      if (abs(pot_value_now - pot_value) > UMBRAL_DIFF_VOL)
      {
        pot_value_new = pot_value_now;
        new_event = EV_VOL;
        return true;
      }
    }
  }

  return false;
}
// ---------- END EVENTS ---------- //

// ---------- BEGIN FUNCTIONS ---------- //
bool is_valid_timestamp()
{
  time_t now = rtc.getEpoch();

  return (user_timestamp > now);
}

void reconnectMQTT()
{
  if (!client.connected())
  {
    if (client.connect("ubidots", mqtt_user, mqtt_passwd)) 
    {
      if(client.subscribe(topic_fecha))
      {
        DebugPrint("Conectado al broker MQTT.");
      }
      else
      {
        DebugPrint("Error al suscribirse al topic.");
      }
    } 
    else 
    {
      DebugPrint("Fallo la conexión al broker MQTT.");
    }
  }
}

void toggle_motor(void *param) 
{
  int value_recv;

  while (true) 
  {
    if (xQueueReceive(queue_motor, &value_recv, portMAX_DELAY) == pdPASS) 
    {
      digitalWrite(PIN_LED_MOTOR, value_recv);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) 
{
  String message((char*)payload, length);

  //DebugPrint(String(message));  
  const size_t capacity = 1024;
  StaticJsonDocument<capacity> doc;

  DeserializationError error = deserializeJson(doc, message);
  if (error) 
  {
    DebugPrint("Error al parsear JSON: ");
    DebugPrint(error.c_str());
    return;
  }

  if (!doc.containsKey("value")) 
  {
    DebugPrint("No se encontró el campo 'value'.");
    return;
  }

  const time_t timestamp_now = doc["value"];

  if (user_timestamp != timestamp_now) 
  {
    user_timestamp = timestamp_now;
    mqtt_input_ready = true;
  }
}


void motor_on()
{
  int value_motor = HIGH;
	xQueueSend(queue_motor, &value_motor, portMAX_DELAY);
}

void motor_off()
{
  int value_motor = LOW;
	xQueueSend(queue_motor, &value_motor, portMAX_DELAY);
}

void buzzer_on()
{
  int frecuency = map(pot_value, BUZZER_IN_MIN, BUZZER_IN_MAX, BUZZER_OUT_MIN, BUZZER_OUT_MAX); 
  tone(PIN_BUZZER, frecuency);
}

void buzzer_off()
{
  noTone(PIN_BUZZER);
}
// ---------- END FUNCTIONS ---------- //

// ---------- BEGIN STATE MACHINE ---------- //
void get_new_event()
{
  short index = 0;
  long ct = millis();
  int diff = (ct - lct);
  timeout_new_event = (diff > UMBRAL_DIFF_NEW_EVENT)?(true):(false);

  if( timeout_new_event )
  {
    timeout_new_event = false;
    lct = ct;

    index = (last_index_type_sensor % MAX_TYPE_EVENTS);

    last_index_type_sensor++;

    if(event_type[index](ct))
    {
      return;
    }
  }

  new_event = EV_CONT;
}

void do_event()
{
  get_new_event();

  if((new_event >= 0) && (new_event < MAX_EVENTS) && 
    (current_state >= 0) && (current_state < MAX_STATES))
  {
    if( new_event != EV_CONT )
    {
      DebugPrintState(s_states[current_state], s_events[new_event]);
    }

    state_table[current_state][new_event]();
  }
  else
  {
    DebugPrint("Error, Estado o Evento fuera de rango.");
  }
}
// ---------- END STATE MACHINE ---------- //

// ---------- BEGIN ESP32 ---------- //
void setup() 
{
  do_init();
}

void loop() 
{
  do_event();
}
// ---------- END ESP32 ---------- //
