/*
 * Lampa - управление LED-матрицей 16x16 (ESP32-C3) через HTTP и MQTT.
 * Версия прошивки 1.8.5 (INMP441 I2S, новый драйвер i2s_std)
 *
 * ВАЖНО ДЛЯ ESP32-C3 SUPER MINI:
 * - GPIO8  — встроенный синий светодиод + Strapping-пин, НЕ использовать для LED-ленты
 * - GPIO9  — кнопка BOOT + Strapping-пин
 * - GPIO2  — Strapping-пин, должен быть HIGH при загрузке
 * - GPIO18/19 — USB D-/D+
 *
 * В этой прошивке используются безопасные пины:
 *   LED_PIN     = GPIO10  (данные WS2812B)
 *   BUTTON_PIN  = GPIO5   (кнопка TTP223)
 *   I2S_BCLK    = GPIO4   (INMP441 SCK)
 *   I2S_WS      = GPIO3   (INMP441 WS)
 *   I2S_DATA    = GPIO6   (INMP441 SD)
 *
 * В Arduino IDE обязательно:
 *   Tools → USB CDC On Boot = "Enabled"
 *   Tools → Flash Size      = "4MB (32Mb)"
 *   Tools → Partition Scheme= "Minimal SPIFFS (1.9MB APP with OTA)"
 */

// ============================================================
// ОТЛАДКА ЧЕРЕЗ SERIAL
// ============================================================
#define DEBUG_SERIAL
// #define DEBUG_VERBOSE

#define DEBUG_BAUD 115200

#ifdef DEBUG_SERIAL
  #define DBG(x)          Serial.print(x)
  #define DBGLN(x)        Serial.println(x)
  #define DBGF(...)       Serial.printf(__VA_ARGS__)
  #define DBG_BEGIN()     do { Serial.begin(DEBUG_BAUD); delay(500); \
                               Serial.println(); \
                               Serial.println(F("=== Lampa debug started ===")); } while(0)
  #define DBG_TAG(tag,msg) do { Serial.print('['); Serial.print(millis()); \
                                Serial.print(F("ms][")); Serial.print(tag); \
                                Serial.print(F("] ")); Serial.println(msg); } while(0)
#else
  #define DBG(x)
  #define DBGLN(x)
  #define DBGF(...)
  #define DBG_BEGIN()
  #define DBG_TAG(tag,msg)
#endif

#ifdef DEBUG_VERBOSE
  #define DBG_V(x)      Serial.print(x)
  #define DBG_VLN(x)    Serial.println(x)
  #define DBG_VF(...)   Serial.printf(__VA_ARGS__)
#else
  #define DBG_V(x)
  #define DBG_VLN(x)
  #define DBG_VF(...)
#endif

#define MQTT_MAX_PACKET_SIZE 4096
#define MQTT_BUFFER_SIZE 4096

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <FastLED.h>
#include <WiFiManager.h>
#include <time.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <esp_ota_ops.h>
#include <math.h>
#include <arduinoFFT.h>
#include <driver/i2s_std.h>

// ============ НАСТРОЙКИ ПО УМОЛЧАНИЮ ============
String mqttServer = "192.168.1.100";
int mqttPort = 1883;
String mqttUser, mqttPassword, mqttPrefix = "lampa";
bool mqttEnabled = false, needPublishState = false, needPublishAlarms = false;
bool mqttForceReconnect = false;
#define MQTT_APPLY_DELAY_MS  1500
bool mqttRestartPending = false;
unsigned long mqttRestartAt = 0;
unsigned long mqttLastAttempt = 0;
bool mqttHadAttempt = false;
uint8_t mqttFailCount = 0;
bool haDiscovery = true, needPublishDiscovery = false;

#define STATE_PUBLISH_INTERVAL 30000UL
unsigned long lastStatePublish = 0;
char mqttServerBuf[64] = "192.168.1.100";

String topicState  = "lampa/state";
String topicSet    = "lampa/set";
String topicAlarms = "lampa/alarms";
String topicAvail  = "lampa/availability";
String mqttSubscribedTopic;
const char* HA_DISCOVERY_PREFIX = "homeassistant";
const char* HA_STATUS_TOPIC     = "homeassistant/status";

// ============ LED (WS2812B) ============
#define LED_PIN       10
#define LED_TYPE      WS2812B
#define COLOR_ORDER   GRB
#define BRIGHTNESS_DEFAULT 5
#define LED_COUNT     256
#define ZIGZAG_WIDTH  16
#define MIN_BRIGHTNESS 1

// ============ КНОПКА (TTP223) ============
#define BUTTON_PIN        5
#define DEBOUNCE_DELAY    30
#define LONG_PRESS_TIME   800
#define CLICK_TIMEOUT     500
#define BRIGHTNESS_STEP   1
#define SPEED_STEP        1
#define SCALE_STEP        1
#define ADJUST_INTERVAL   40

// ============ INMP441 I2S SOUND SENSOR ============
#define I2S_BCLK_PIN      4
#define I2S_WS_PIN        3
#define I2S_DATA_PIN      6
#define I2S_PORT_NUM      I2S_NUM_0
#define I2S_SAMPLE_RATE   16000
#define I2S_BUFFER_SIZE   256
#define SOUND_SAMPLE_MS   8
#define I2S_READ_TIMEOUT_MS 20

#define FFT_SIZE           64
#define SPECTRUM_BANDS     16
#define FFT_UPDATE_MS      50

const char* DEVICE_MODEL = "LampaESP32C3";
const char* DEVICE_MANUFACTURER = "vtgHome";
String deviceHostname = "lampa";

#define FIRMWARE_VERSION "1.8.5"
#define OTA_PASSWORD     "admin"
#define OTA_HTTP_USER    "admin"
#define MAX_FPS          80

volatile bool otaInProgress = false;
unsigned long lastShow = 0;
bool rebootPending = false;
unsigned long rebootAt = 0;
bool wifiRestartPending = false;
unsigned long wifiRestartAt = 0;
String mqttClientId;

CRGB leds[LED_COUNT];
uint8_t matrixDrops[LED_COUNT];
uint8_t heat[ZIGZAG_WIDTH][ZIGZAG_WIDTH];
uint8_t fireplaceHeat[LED_COUNT], campfireHeat[LED_COUNT];

WebServer server(80);
WiFiClient espClient;
PubSubClient mqttClient(espClient);
WiFiManager wifiManager;
Preferences preferences;

bool lampOn = true;
uint8_t brightness = BRIGHTNESS_DEFAULT, maxBrightness = 255;
CRGB solidColor = CRGB::White;
String currentEffect = "confetti";
unsigned long lastEffectUpdate = 0;
uint8_t effectSpeed = 50, effectScale = 50;

bool     soundEnabled     = false;
uint8_t  soundSensitivity = 50;
uint8_t  soundMode        = 0;
uint8_t  soundLevel       = 0;
uint8_t  soundBeatFlash   = 0;
uint8_t  soundBeatAvg     = 0;
uint8_t  soundBeatCooldown = 0;
unsigned long lastSoundSample = 0;

double   fftReal[FFT_SIZE];
double   fftImag[FFT_SIZE];
uint8_t  spectrum[SPECTRUM_BANDS];
uint8_t  spectrumPeak[SPECTRUM_BANDS];
unsigned long lastFftTime = 0;
arduinoFFT soundFFT = arduinoFFT(fftReal, fftImag, FFT_SIZE, I2S_SAMPLE_RATE);

i2s_chan_handle_t i2s_rx_handle = NULL;
int32_t  i2sBuffer[I2S_BUFFER_SIZE];
int16_t  sampleBuffer[FFT_SIZE];
int      sampleIndex = 0;

uint8_t hue = 0, rainbowStep = 0, runningPos = 0;
unsigned long sunriseTimer = 0;
uint8_t sunrisePhase = 0;
uint8_t fireplacePhase = 0;
uint16_t fireplaceNoise = 0;
uint8_t campfirePhase = 0;
uint16_t campfireNoise = 0;

uint16_t plasmaPhase = 0, gradientPhase = 0, breathingPhase = 0;
uint8_t  strobePhase = 0;
uint16_t pacificaTime = 0, auroraTime = 0, radialPhase = 0, heartbeatPhase = 0;

#define SNOW_COUNT 20
uint8_t snowX[SNOW_COUNT], snowY[SNOW_COUNT], snowSpeed[SNOW_COUNT], snowPhase[SNOW_COUNT];
uint8_t twinklePhase[LED_COUNT], twinkleSpeed[LED_COUNT];

#define FIREWORKS_MAX 32
struct FireworkParticle { int8_t x, y, dx, dy; uint8_t life; };
FireworkParticle fireworks[FIREWORKS_MAX];
uint16_t fireworksTimer = 0;
uint8_t lightningFlash = 0;
int16_t ballX = 8, ballY = 8;
int8_t ballDX = 1, ballDY = 1;
uint8_t ballHue = 0;
int16_t sinelonPos = 0;
int8_t sinelonDir = 1;
uint8_t lifeGrid[ZIGZAG_WIDTH][ZIGZAG_WIDTH], lifeNext[ZIGZAG_WIDTH][ZIGZAG_WIDTH];
uint16_t lifeGenTimer = 0, lifeStep = 0;

int timezoneOffsetMinutes = 0;
String ntpServer = "pool.ntp.org";
int alarmOnMinutes[7] = {-1,-1,-1,-1,-1,-1,-1};
int alarmOffMinutes[7] = {-1,-1,-1,-1,-1,-1,-1};
bool alarmEnabled[7] = {false,false,false,false,false,false,false};
bool alarmTriggeredToday = false;
int lastDay = -1;

enum EffectIndex {
  EFFECT_SOLID, EFFECT_RAINBOW, EFFECT_CONFETTI, EFFECT_FIRE, EFFECT_FIREPLACE,
  EFFECT_CAMPFIRE, EFFECT_RUNNING_LIGHTS, EFFECT_MATRIX, EFFECT_SUNRISE,
  EFFECT_PLASMA, EFFECT_GRADIENT, EFFECT_BREATHING, EFFECT_STROBE, EFFECT_PACIFICA,
  EFFECT_SPARKLE, EFFECT_SNOW, EFFECT_TWINKLE, EFFECT_AURORA, EFFECT_FIREWORKS,
  EFFECT_LIGHTNING, EFFECT_BOUNCING_BALL, EFFECT_RADIAL_PULSE, EFFECT_SINELON,
  EFFECT_HEARTBEAT, EFFECT_LIFE, EFFECT_SPECTRUM, EFFECT_COUNT
};

struct EffectDef { const char* id; const char* ru; };
static const EffectDef EFFECTS[EFFECT_COUNT] = {
  {"solid",          "Статичный"},
  {"rainbow",        "Радуга"},
  {"confetti",       "Конфетти"},
  {"fire",           "Огонь"},
  {"fireplace",      "Камин"},
  {"campfire",       "Костёр"},
  {"running_lights", "Бегущие огни"},
  {"matrix",         "Матрица"},
  {"sunrise",        "Рассвет"},
  {"plasma",         "Плазма"},
  {"gradient",       "Градиент"},
  {"breathing",      "Дыхание"},
  {"strobe",         "Стробоскоп"},
  {"pacifica",       "Пацифика"},
  {"sparkle",        "Искры"},
  {"snow",           "Снег"},
  {"twinkle",        "Мерцание"},
  {"aurora",         "Аврора"},
  {"fireworks",      "Фейерверк"},
  {"lightning",      "Молния"},
  {"bouncing_ball",  "Прыгающий мяч"},
  {"radial_pulse",   "Радиальный пульс"},
  {"sinelon",        "Синтелон"},
  {"heartbeat",      "Сердцебиение"},
  {"life",           "Жизнь"},
  {"spectrum",       "Спектр"}
};

uint8_t presetBrightness[EFFECT_COUNT], presetSpeed[EFFECT_COUNT], presetScale[EFFECT_COUNT];

// ============ КОЛЬЦЕВОЙ БУФЕР ЛОГОВ ============
#define LOG_BUFFER_SIZE 30
String logBuffer[LOG_BUFFER_SIZE];
int logIndex = 0, logCount = 0;

enum ButtonMode { MODE_NORMAL, MODE_BRIGHTNESS, MODE_SPEED, MODE_SCALE };
ButtonMode buttonMode = MODE_NORMAL;
int8_t speedDirection = 1, scaleDirection = 1, brightnessDirection = 1;
unsigned long lastButtonRelease = 0;
int clickCount = 0;
bool isInSequence = false, waitingForClick = false, longPressProcessed = false;
bool buttonPressed = false;
unsigned long pressStartTime = 0;

static const char PAGE_HEAD[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="UTF-8"><title>%TITLE%</title>
<style>
  body{font-family:Arial;margin:20px;max-width:900px;color:#222}
  h1{margin-top:0}
  h3{margin-top:28px;border-bottom:1px solid #ddd;padding-bottom:4px}
  a{color:#4a90d9}
  .nav{margin-bottom:18px}.nav a{margin-right:12px;font-size:16px;text-decoration:none}
  .control{margin-bottom:15px}
  label{display:inline-block;width:180px;vertical-align:top}
  button{padding:8px 16px;margin:5px;cursor:pointer;border:1px solid #bbb;border-radius:4px;background:#f6f6f6}
  button:hover{background:#eaeaea}
  input[type=range]{width:220px;vertical-align:middle}
  input[type=text],input[type=number],input[type=password]{width:200px;padding:4px;box-sizing:border-box}
  select{min-width:200px;padding:4px}
  .value{display:inline-block;min-width:30px;margin-left:5px}
  .row{margin-bottom:10px}
  .section{padding:12px;background:#f9f9f9;border:1px solid #ddd;border-radius:5px;margin-bottom:10px}
  .info{background:#e8f4fd;padding:10px;border-radius:5px;margin-bottom:15px;font-size:14px;line-height:1.6}
  .warn{color:#c0392b;font-weight:bold;margin:10px 0}
  .danger{background:#c0392b;color:#fff;border-color:#a93226}
  .danger:hover{background:#a93226}
  .time-block{margin:20px 0;padding:10px;background:#f0f0f0;border-radius:5px}
  .log-container{background:#f8f8f8;padding:10px;max-height:220px;overflow:auto;font-size:12px;font-family:monospace;border:1px solid #ddd}
  table{border-collapse:collapse;margin-bottom:20px}
  th,td{border:1px solid #ccc;padding:8px;text-align:center}
  th{background:#f0f0f0}
  .day-row input[type=time]{width:120px}
  .day-row input[type=checkbox]{transform:scale(1.5)}
  .upload-box{border:2px dashed #4a90d9;border-radius:8px;padding:20px;text-align:center;background:#fafbfc}
  .upload-box input{margin:10px 0}
  #bar{width:100%;height:18px;background:#eee;border-radius:9px;overflow:hidden;display:none;margin-top:10px}
  #fill{height:100%;width:0;background:#4a90d9;transition:width .3s}
  #status{margin-top:10px;font-weight:bold}
  .mono{font-family:monospace}
  .level-bar{display:inline-block;width:200px;height:12px;background:#eee;border-radius:6px;margin-left:8px;vertical-align:middle;overflow:hidden}
  .level-bar>div{height:100%;width:0;background:#4a90d9;transition:width .1s}
</style>
<script>
function cmd(c){return fetch('/api?command='+encodeURIComponent(c))}
function cmdText(c){return cmd(c).then(r=>r.text())}
function escHtml(s){return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;')}
function updateTime(){fetch('/api/time').then(r=>r.json()).then(d=>{
  var e=document.getElementById('current-time');if(e)e.textContent=d.time}).catch(()=>{})}
</script>
</head><body>
<h1>%TITLE%</h1>
<p class="nav"><a href="/">Главная</a><a href="/alarm">Будильники</a><a href="/settings">Настройки</a></p>
)HTML";

static void sendPage(const char* title, const String& body) {
  String html;
  html.reserve(strlen_P(PAGE_HEAD) + body.length() + 32);
  html = FPSTR(PAGE_HEAD);
  html.replace("%TITLE%", title);
  html += body;
  html += "</body></html>";
  server.send(200, "text/html; charset=utf-8", html);
}

void setupWiFi(); void setupMQTT(); void mqttCallback(char*, byte*, unsigned int);
void reconnectMQTT(); void handleRoot(); void handleAPI(); void handleAlarmPage();
void handleSettingsPage(); void handleStatus(); void handleTime(); void handleLogs();
void setupOTA(); void otaBeginGuard(); void otaEndGuard(bool);
void handleUpdatePage(); void handleUpdateUpload(); void applyEffect(); void publishState();
void processMQTTCommand(String); void setSolidColor(int,int,int); void setBrightness(int);
int getLedIndex(int); int getLedIndexRotatedMinus90(int);
void checkAlarm(); void syncTime(); void publishAlarmState();
void saveSettings(); void loadSettings(); String getLocalTimeString();
void applyMqttSettings(); void scheduleMqttRestart();
void addLog(const String&); String mqttStateText(int);
bool mqttPublishJson(const String&, const String&, const char*);
void publishDiscovery(); void removeDiscovery(); void publishAvailability(const char*);
bool applyHaJsonCommand(const String&); String minutesToHHMM(int); int parseHHMM(const String&);
int getEffectIndex(const String&); void loadEffectSettings(int);
void saveCurrentEffectSettings(); void updateMqttTopic(); void applyHostname();
String sanitizeHostname(const String&); void handleButton(); void showValueScale(int,int,int,CRGB);
void switchEffectBy(int); void resetEffectState(const String&);
void updateSoundLevel(); void updateSpectrum();
void initI2S();

static bool trimArg(const String& in, String& out) {
  out = in; out.trim(); return out.length() > 0;
}

// ============ SETUP ============
void setup() {
  DBG_BEGIN();

  pinMode(BUTTON_PIN, INPUT);

  uint64_t mac = ESP.getEfuseMac();
  String macStr = String((uint32_t)(mac >> 24), HEX) + String((uint32_t)(mac & 0xFFFFFF), HEX);
  macStr.toUpperCase();
  mqttClientId = String(DEVICE_MODEL) + "-" + macStr;
  DBG_TAG("BOOT", "fw=" FIRMWARE_VERSION " clientId=" + mqttClientId);

  initI2S();

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, LED_COUNT);
  FastLED.setBrightness(brightness);
  FastLED.clear(); FastLED.show();
  DBG_TAG("LED", "FastLED initialized, pin=" + String(LED_PIN) + " count=" + String(LED_COUNT));

  for (int i = 0; i < LED_COUNT; i++) { matrixDrops[i] = 0; fireplaceHeat[i] = 0; campfireHeat[i] = 0; }
  memset(heat, 0, sizeof(heat));
  memset(lifeGrid, 0, sizeof(lifeGrid));
  memset(lifeNext, 0, sizeof(lifeNext));
  memset(spectrum, 0, sizeof(spectrum));
  memset(spectrumPeak, 0, sizeof(spectrumPeak));
  for (int i = 0; i < FIREWORKS_MAX; i++) fireworks[i].life = 0;

  loadSettings();
  DBG_TAG("NVS", "settings loaded: hostname=" + deviceHostname
                + " mqtt=" + String(mqttEnabled) + " ha=" + String(haDiscovery)
                + " snd=" + String(soundEnabled));

  setupWiFi();
  if (WiFi.status() == WL_CONNECTED) syncTime(); else addLog("NTP sync skipped (no WiFi)");
  setupOTA();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api", HTTP_GET, handleAPI);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/time", HTTP_GET, handleTime);
  server.on("/api/logs", HTTP_GET, handleLogs);
  server.on("/alarm", HTTP_GET, handleAlarmPage);
  server.on("/settings", HTTP_GET, handleSettingsPage);
  server.on("/update", HTTP_GET, handleUpdatePage);
  server.on("/update", HTTP_POST, handleUpdatePage, handleUpdateUpload);
  server.onNotFound([](){ server.send(404, "text/plain", "Not found"); });
  server.begin();
  DBG_TAG("HTTP", "server started on port 80");

  addLog("System started, fw " FIRMWARE_VERSION);
  addLog("WiFi IP: " + WiFi.localIP().toString());
  addLog("Hostname: " + deviceHostname);
  addLog("MQTT Client ID: " + mqttClientId);
  setupMQTT();
  if (mqttEnabled) reconnectMQTT();

  DBG_TAG("BOOT", "setup() complete, free heap=" + String(ESP.getFreeHeap()));
}

// ============ I2S INIT (новый драйвер i2s_std) ============
void initI2S() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT_NUM, I2S_ROLE_MASTER);
  chan_cfg.auto_clear = true;
  esp_err_t err1 = i2s_new_channel(&chan_cfg, NULL, &i2s_rx_handle);

  i2s_std_config_t std_cfg = {
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S_BCLK_PIN,
      .ws   = (gpio_num_t)I2S_WS_PIN,
      .dout = I2S_GPIO_UNUSED,
      .din  = (gpio_num_t)I2S_DATA_PIN,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv   = false,
      },
    },
  };
  std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

  esp_err_t err2 = i2s_channel_init_std_mode(i2s_rx_handle, &std_cfg);
  esp_err_t err3 = i2s_channel_enable(i2s_rx_handle);

  addLog("I2S initialized: BCLK=" + String(I2S_BCLK_PIN) + " WS=" + String(I2S_WS_PIN) + " DATA=" + String(I2S_DATA_PIN));
  DBG_TAG("I2S", "new_channel=" + String((int)err1)
                + " init_std=" + String((int)err2)
                + " enable=" + String((int)err3)
                + " BCLK=" + String(I2S_BCLK_PIN)
                + " WS=" + String(I2S_WS_PIN)
                + " DATA=" + String(I2S_DATA_PIN)
                + " rate=" + String(I2S_SAMPLE_RATE));
}

// ============ LOOP ============
void loop() {
  ArduinoOTA.handle();
  if (rebootPending && (long)(millis() - rebootAt) >= 0) {
    addLog("Rebooting..."); delay(100); ESP.restart();
  }
  server.handleClient();
  if (otaInProgress) { yield(); return; }

  if (wifiRestartPending && (long)(millis() - wifiRestartAt) >= 0) {
    wifiRestartPending = false;
    addLog("Restarting WiFi to apply hostname: " + deviceHostname);
    MDNS.end();
    WiFi.disconnect(false, false); delay(300);
    WiFi.setHostname(deviceHostname.c_str());
    WiFi.begin();
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) { delay(100); ArduinoOTA.handle(); }
    if (WiFi.status() == WL_CONNECTED) {
      addLog("WiFi reconnected, IP: " + WiFi.localIP().toString());
      addLog("Hostname applied: " + String(WiFi.getHostname()));
      if (MDNS.begin(deviceHostname.c_str())) { MDNS.addService("http","tcp",80); addLog("mDNS restarted"); }
      needPublishState = true;
    } else addLog("WiFi reconnect failed");
  }

  if (mqttRestartPending && (long)(millis() - mqttRestartAt) >= 0) {
    mqttRestartPending = false;
    applyMqttSettings();
  }

  if (mqttEnabled) {
    if (WiFi.status() != WL_CONNECTED) {
      static unsigned long lastWifiLog = 0;
      if (millis() - lastWifiLog > 30000) { lastWifiLog = millis(); addLog("MQTT paused: no WiFi"); }
    } else {
      if (!mqttClient.connected()) reconnectMQTT();
      mqttClient.loop();
      if (mqttClient.connected()) {
        if (needPublishDiscovery) { needPublishDiscovery = false; publishDiscovery(); }
        if (needPublishState)     { needPublishState = false;     publishState(); }
        if (needPublishAlarms)    { needPublishAlarms = false;    publishAlarmState(); }
        if (millis() - lastStatePublish >= STATE_PUBLISH_INTERVAL) publishState();
      }
    }
  }

  updateSoundLevel();
  updateSpectrum();
  handleButton();
  unsigned long effectDelay = map(effectSpeed, 1, 100, 100, 5);
  if (millis() - lastEffectUpdate > effectDelay) { lastEffectUpdate = millis(); applyEffect(); }

  static unsigned long lastAlarmCheck = 0;
  if (millis() - lastAlarmCheck > 1000) { lastAlarmCheck = millis(); checkAlarm(); }

  if (millis() - lastShow >= (1000UL / MAX_FPS)) { lastShow = millis(); FastLED.show(); }
}

// ============ OTA ============
void otaBeginGuard() {
  otaInProgress = true;
  if (mqttClient.connected()) mqttClient.disconnect();
  FastLED.clear(true);
  addLog("OTA started");
  DBG_TAG("OTA", "start");
}
void otaEndGuard(bool ok) {
  otaInProgress = false;
  addLog(ok ? "OTA finished OK" : "OTA failed");
  DBG_TAG("OTA", ok ? "finished OK" : "failed");
}

void setupOTA() {
  String host = sanitizeHostname(deviceHostname);
  ArduinoOTA.setHostname(host.c_str());
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([](){ otaBeginGuard(); });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t){
    static int lastPct = -1;
    int pct = t ? (int)(p * 100 / t) : 0;
    if (pct != lastPct && pct % 10 == 0) {
      lastPct = pct;
      DBGF("OTA %d%%\n", pct);
    }
  });
  ArduinoOTA.onEnd([](){ otaEndGuard(true); });
  ArduinoOTA.onError([](ota_error_t e){
    DBG_TAG("OTA", "error code=" + String((int)e));
    otaEndGuard(false); delay(1000); ESP.restart();
  });
  ArduinoOTA.begin();
  if (MDNS.begin(host.c_str())) { MDNS.addService("http","tcp",80); addLog("mDNS: " + host + ".local"); }
  else addLog("mDNS start failed");
  if (esp_ota_get_next_update_partition(NULL) == NULL) addLog("WARNING: no OTA partition");
  addLog("OTA ready, fw " FIRMWARE_VERSION);
  DBG_TAG("OTA", "ready host=" + host + " password=" + String(OTA_PASSWORD));
}

void handleUpdatePage() {
  if (!server.authenticate(OTA_HTTP_USER, OTA_PASSWORD)) return server.requestAuthentication();
  if (server.method() == HTTP_POST) {
    bool ok = !Update.hasError();
    server.sendHeader("Connection", "close");
    server.send(200, "text/html; charset=utf-8",
                ok ? "<h2>Обновление успешно. Перезагрузка...</h2>" : "<h2>Ошибка</h2>");
    delay(500); if (ok) ESP.restart();
    return;
  }
  String body = String("<p>Текущая версия: ") + FIRMWARE_VERSION + "</p>"
    "<form id='f' method='POST' action='/update' enctype='multipart/form-data'>"
    "<input type='file' name='firmware' accept='.bin' required>"
    "<button type='submit'>Загрузить</button></form>"
    "<div id='bar'><div id='fill'></div></div><p id='st'></p>"
    "<script>"
    "document.getElementById('f').onsubmit=function(e){"
    "e.preventDefault();var fd=new FormData(this),x=new XMLHttpRequest();"
    "x.open('POST','/update',true);"
    "x.upload.onprogress=function(ev){if(ev.lengthComputable){"
    "var p=Math.round(ev.loaded/ev.total*100);"
    "document.getElementById('fill').style.width=p+'%';"
    "document.getElementById('st').textContent=p+'%';}};"
    "x.onload=function(){document.getElementById('st').textContent="
    "x.responseText.replace(/<[^>]+>/g,' ').trim();};"
    "x.onerror=function(){document.getElementById('st').textContent='Ошибка';};"
    "x.send(fd);};</script>";
  sendPage("Обновление прошивки", body);
}

void handleUpdateUpload() {
  if (!server.authenticate(OTA_HTTP_USER, OTA_PASSWORD)) return server.requestAuthentication();
  HTTPUpload& up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    otaBeginGuard();
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) { addLog("OTA begin failed"); otaInProgress = false; }
    else DBG_TAG("OTA", "Update.begin OK, file=" + up.filename);
  } else if (up.status == UPLOAD_FILE_WRITE) {
    Update.write(up.buf, up.currentSize);
  } else if (up.status == UPLOAD_FILE_END) {
    Update.end(true) ? otaEndGuard(true) : otaEndGuard(false);
    DBG_TAG("OTA", "uploaded " + String(up.totalSize) + " bytes");
  } else if (up.status == UPLOAD_FILE_ABORTED) { Update.abort(); otaEndGuard(false); }
  yield();
}

// ============ WiFi / hostname ============
void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setHostname(deviceHostname.c_str());
  wifiManager.setHostname(deviceHostname.c_str());
  DBG_TAG("WiFi", "connecting (hostname=" + deviceHostname + ")...");
  if (!wifiManager.autoConnect("LampaAP")) {
    DBG_TAG("WiFi", "autoConnect failed, restarting");
    delay(3000); ESP.restart();
  }
  WiFi.setHostname(deviceHostname.c_str());
  addLog("WiFi connected, IP: " + WiFi.localIP().toString());
  addLog("DHCP hostname: " + String(WiFi.getHostname()));
  DBG_TAG("WiFi", "connected IP=" + WiFi.localIP().toString() + " RSSI=" + String(WiFi.RSSI()));
}

void applyHostname() {
  String n = sanitizeHostname(deviceHostname);
  bool changed = (n != deviceHostname);
  deviceHostname = n;
  if (String(WiFi.getHostname()) != deviceHostname) {
    if (!WiFi.setHostname(deviceHostname.c_str())) { addLog("setHostname failed"); return; }
  }
  if (changed) addLog("Hostname set to: " + deviceHostname);
}

String sanitizeHostname(const String& in) {
  String r;
  for (unsigned int i = 0; i < in.length() && r.length() < 32; i++) {
    char c = in.charAt(i);
    if (isalnum(c) || c == '-') r += c;
  }
  if (!r.length()) r = "lampa";
  r.toLowerCase();
  return r;
}

// ============ NVS ============
void loadSettings() {
  preferences.begin("lampa", false);
  timezoneOffsetMinutes = preferences.getInt("tz_offset", 0);
  ntpServer = preferences.getString("ntp_server", "pool.ntp.org");
  maxBrightness = preferences.getUChar("max_bright", 255);
  if (maxBrightness < MIN_BRIGHTNESS) maxBrightness = MIN_BRIGHTNESS;
  deviceHostname = sanitizeHostname(preferences.getString("hostname", "lampa"));
  for (int i = 0; i < EFFECT_COUNT; i++) {
    presetBrightness[i] = preferences.getUChar(("pbright_" + String(i)).c_str(), BRIGHTNESS_DEFAULT);
    presetSpeed[i]      = preferences.getUChar(("pspeed_"  + String(i)).c_str(), 50);
    presetScale[i]      = preferences.getUChar(("pscale_"  + String(i)).c_str(), 50);
  }
  mqttEnabled = preferences.getBool("mqtt_en", false);
  mqttServer  = preferences.getString("mqtt_server", "192.168.1.100");
  mqttPort    = preferences.getInt("mqtt_port", 1883);
  mqttUser    = preferences.getString("mqtt_user", "");
  mqttPassword= preferences.getString("mqtt_pass", "");
  mqttPrefix  = preferences.getString("mqtt_prefix", "lampa");
  haDiscovery = preferences.getBool("ha_disc", true);

  soundEnabled     = preferences.getBool("snd_en",  false);
  soundSensitivity = preferences.getUChar("snd_sens", 50);
  if (soundSensitivity < 1) soundSensitivity = 1;
  if (soundSensitivity > 100) soundSensitivity = 100;
  soundMode        = preferences.getUChar("snd_mode", 0);
  if (soundMode > 2) soundMode = 0;

  topicSet = ""; updateMqttTopic();
  for (int i = 0; i < 7; i++) {
    alarmOnMinutes[i]  = preferences.getInt(("alarm_on_" + String(i)).c_str(), -1);
    alarmOffMinutes[i] = preferences.getInt(("alarm_off_" + String(i)).c_str(), -1);
    alarmEnabled[i]    = preferences.getBool(("alarm_en_" + String(i)).c_str(), false);
  }
  preferences.end();
  int idx = getEffectIndex(currentEffect);
  if (idx >= 0) { loadEffectSettings(idx); resetEffectState(currentEffect); }
}

void saveSettings() {
  preferences.begin("lampa", false);
  preferences.putInt("tz_offset", timezoneOffsetMinutes);
  preferences.putString("ntp_server", ntpServer);
  preferences.putUChar("max_bright", maxBrightness);
  preferences.putBool("mqtt_en", mqttEnabled);
  preferences.putString("mqtt_server", mqttServer);
  preferences.putInt("mqtt_port", mqttPort);
  preferences.putString("mqtt_user", mqttUser);
  preferences.putString("mqtt_pass", mqttPassword);
  preferences.putString("mqtt_prefix", mqttPrefix);
  preferences.putBool("ha_disc", haDiscovery);
  preferences.putString("hostname", deviceHostname);

  preferences.putBool ("snd_en",   soundEnabled);
  preferences.putUChar("snd_sens", soundSensitivity);
  preferences.putUChar("snd_mode", soundMode);

  for (int i = 0; i < 7; i++) {
    preferences.putInt(("alarm_on_" + String(i)).c_str(), alarmOnMinutes[i]);
    preferences.putInt(("alarm_off_" + String(i)).c_str(), alarmOffMinutes[i]);
    preferences.putBool(("alarm_en_" + String(i)).c_str(), alarmEnabled[i]);
  }
  for (int i = 0; i < EFFECT_COUNT; i++) {
    preferences.putUChar(("pbright_" + String(i)).c_str(), presetBrightness[i]);
    preferences.putUChar(("pspeed_"  + String(i)).c_str(), presetSpeed[i]);
    preferences.putUChar(("pscale_"  + String(i)).c_str(), presetScale[i]);
  }
  preferences.end();
  DBG_TAG("NVS", "settings saved");
}

// ============ Время / логи ============
void syncTime() {
  configTime(0, 0, ntpServer.c_str(), "time.nist.gov");
  int attempts = 0;
  while (time(nullptr) < 100000 && attempts < 20) {
    unsigned long t0 = millis();
    while (millis() - t0 < 500) { ArduinoOTA.handle(); if (otaInProgress) return; delay(10); }
    attempts++;
  }
  if (time(nullptr) < 100000) addLog("NTP sync failed");
  else DBG_TAG("NTP", "synced, local time=" + getLocalTimeString());
}

String getLocalTimeString() {
  time_t now = time(nullptr);
  if (now < 100000) return "—";
  time_t local = now + timezoneOffsetMinutes * 60;
  struct tm tm; localtime_r(&local, &tm);
  char buf[32]; strftime(buf, sizeof(buf), "%d.%m.%Y %H:%M:%S", &tm);
  return String(buf);
}

// Лог: в кольцевой буфер и (опционально) в Serial
void addLog(const String& msg) {
  String ts = getLocalTimeString();
  if (ts == "—") ts = "??:??:??";
  String line = "[" + ts + "] " + msg;
  logBuffer[logIndex] = line;
  logIndex = (logIndex + 1) % LOG_BUFFER_SIZE;
  if (logCount < LOG_BUFFER_SIZE) logCount++;
#ifdef DEBUG_SERIAL
  Serial.println(line);
#endif
}

// ============ ПРЕСЕТЫ ============
int getEffectIndex(const String& name) {
  for (int i = 0; i < EFFECT_COUNT; i++) if (name == EFFECTS[i].id) return i;
  return -1;
}
void loadEffectSettings(int idx) {
  if (idx < 0 || idx >= EFFECT_COUNT) return;
  brightness  = presetBrightness[idx];
  effectSpeed = presetSpeed[idx];
  effectScale = presetScale[idx];
  if (brightness > maxBrightness) brightness = maxBrightness;
}
void saveCurrentEffectSettings() {
  int idx = getEffectIndex(currentEffect);
  if (idx < 0) return;
  presetBrightness[idx] = brightness;
  presetSpeed[idx] = effectSpeed;
  presetScale[idx] = effectScale;
  preferences.begin("lampa", false);
  preferences.putUChar(("pbright_" + String(idx)).c_str(), presetBrightness[idx]);
  preferences.putUChar(("pspeed_"  + String(idx)).c_str(), presetSpeed[idx]);
  preferences.putUChar(("pscale_"  + String(idx)).c_str(), presetScale[idx]);
  preferences.end();
}
void resetEffectState(const String& effect) {
  plasmaPhase = gradientPhase = breathingPhase = strobePhase = 0;
  pacificaTime = auroraTime = radialPhase = heartbeatPhase = 0;
  if (effect == "sunrise") { sunriseTimer = millis(); sunrisePhase = 0; FastLED.setBrightness(0); }
  else if (effect == "matrix") for (int i = 0; i < LED_COUNT; i++) matrixDrops[i] = 0;
  else if (effect == "fire") memset(heat, 0, sizeof(heat));
  else if (effect == "fireplace") { for (int i = 0; i < LED_COUNT; i++) fireplaceHeat[i] = 0; fireplacePhase = 0; fireplaceNoise = 0; }
  else if (effect == "campfire")  { for (int i = 0; i < LED_COUNT; i++) campfireHeat[i]  = 0; campfirePhase  = 0; campfireNoise  = 0; }
  else if (effect == "snow") for (int i = 0; i < SNOW_COUNT; i++) { snowX[i] = random8(ZIGZAG_WIDTH); snowY[i] = random8(ZIGZAG_WIDTH); snowSpeed[i] = random8(1,4); snowPhase[i] = random8(); }
  else if (effect == "twinkle") for (int i = 0; i < LED_COUNT; i++) { twinklePhase[i] = random8(); twinkleSpeed[i] = random8(1,6); }
  else if (effect == "fireworks") { for (int i = 0; i < FIREWORKS_MAX; i++) fireworks[i].life = 0; fireworksTimer = 0; }
  else if (effect == "bouncing_ball") { ballX = ballY = 8; ballDX = (random8()&1)?1:-1; ballDY = (random8()&1)?1:-1; ballHue = random8(); }
  else if (effect == "sinelon") { sinelonPos = 0; sinelonDir = 1; }
  else if (effect == "lightning") lightningFlash = 0;
  else if (effect == "life") {
    memset(lifeGrid, 0, sizeof(lifeGrid)); memset(lifeNext, 0, sizeof(lifeNext));
    for (int i = 0; i < 70; i++) lifeGrid[random8(ZIGZAG_WIDTH)][random8(ZIGZAG_WIDTH)] = 1;
    lifeGenTimer = 0; lifeStep = 0;
  }
  else if (effect == "spectrum") { memset(spectrum, 0, sizeof(spectrum)); memset(spectrumPeak, 0, sizeof(spectrumPeak)); }
  DBG_TAG("EFFECT", "reset state for " + effect);
}

// ============ MQTT ============
String mqttStateText(int st) {
  switch (st) {
    case -4: return "timeout";
    case -3: return "connection lost";
    case -2: return "connect failed (TCP)";
    case -1: return "disconnected";
    case  0: return "connected";
    case  1: return "bad protocol version";
    case  2: return "client id rejected";
    case  3: return "server unavailable";
    case  4: return "bad credentials";
    case  5: return "not authorized";
    default: return "unknown";
  }
}
void setupMQTT() {
  updateMqttTopic();
  strncpy(mqttServerBuf, mqttServer.c_str(), sizeof(mqttServerBuf) - 1);
  mqttServerBuf[sizeof(mqttServerBuf) - 1] = '\0';
  mqttClient.setServer(mqttServerBuf, mqttPort);
  mqttClient.setCallback(mqttCallback);
  if (!mqttClient.setBufferSize(MQTT_BUFFER_SIZE)) addLog("MQTT: setBufferSize failed");
  mqttClient.setKeepAlive(30);
  mqttClient.setSocketTimeout(5);
}

bool applyHaJsonCommand(const String& message) {
  StaticJsonDocument<512> j;
  if (deserializeJson(j, message)) { addLog("MQTT: некорректный JSON"); return false; }
  bool handled = false;
  if (j.containsKey("state")) { const char* s = j["state"]; if (s) { processMQTTCommand(String(s)); handled = true; } }
  if (j.containsKey("effect")) { const char* e = j["effect"]; if (e) { processMQTTCommand("EFFECT:" + String(e)); handled = true; } }
  if (j.containsKey("color")) {
    JsonObject c = j["color"];
    if (!c.isNull()) {
      processMQTTCommand("COLOR:" + String((int)(c["r"]|0)) + "," + String((int)(c["g"]|0)) + "," + String((int)(c["b"]|0)));
      handled = true;
    }
  }
  if (j.containsKey("brightness")) { processMQTTCommand("BRIGHTNESS:" + String((int)(j["brightness"]|0))); handled = true; }
  if (!handled && j.containsKey("command")) { const char* c = j["command"]; if (c) { processMQTTCommand(String(c)); handled = true; } }
  if (j.containsKey("speed")) { processMQTTCommand("SPEED:" + String((int)(j["speed"]|50))); handled = true; }
  if (j.containsKey("scale")) { processMQTTCommand("SCALE:" + String((int)(j["scale"]|50))); handled = true; }
  if (!handled) addLog("MQTT: JSON без известных полей");
  return handled;
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message; message.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++) message += (char)payload[i];
  message.trim();
  String t = topic;
  DBG_TAG("MQTT<", t + " => " + message);
  if (t == HA_STATUS_TOPIC) {
    if (message.equalsIgnoreCase("online")) {
      addLog("HA online -> republish discovery");
      needPublishDiscovery = true; needPublishState = true; needPublishAlarms = true;
    }
    return;
  }
  if (message.isEmpty()) return;
  if (t == topicState || t == topicAlarms || t == topicAvail) return;
  addLog("MQTT msg [" + t + "]: " + message);
  if (message.startsWith("{")) applyHaJsonCommand(message);
  else processMQTTCommand(message);
}

void reconnectMQTT() {
  if (!mqttEnabled || mqttClient.connected() || WiFi.status() != WL_CONNECTED) return;
  unsigned long interval = 5000UL * (mqttFailCount > 5 ? 6 : (mqttFailCount + 1));
  if (interval > 30000UL) interval = 30000UL;
  unsigned long now = millis();
  if (mqttForceReconnect) mqttForceReconnect = false;
  else if (mqttHadAttempt && (now - mqttLastAttempt) < interval) return;
  mqttLastAttempt = now; mqttHadAttempt = true;
  addLog("MQTT connecting to " + String(mqttServerBuf) + ":" + String(mqttPort) + "...");
  const char* user = mqttUser.length() ? mqttUser.c_str() : NULL;
  const char* pass = mqttPassword.length() ? mqttPassword.c_str() : NULL;
  if (mqttClient.connect(mqttClientId.c_str(), user, pass, topicAvail.c_str(), 0, true, "offline")) {
    mqttFailCount = 0;
    addLog("MQTT connected as " + mqttClientId);
    publishAvailability("online");
    if (mqttClient.subscribe(topicSet.c_str())) { mqttSubscribedTopic = topicSet; addLog("MQTT subscribed to " + topicSet); }
    else { mqttSubscribedTopic = ""; addLog("MQTT subscribe FAILED for " + topicSet); }
    if (haDiscovery) mqttClient.subscribe(HA_STATUS_TOPIC);
    needPublishDiscovery = haDiscovery; needPublishState = true; needPublishAlarms = true;
    DBG_TAG("MQTT", "connected, subscribed=" + topicSet);
  } else {
    if (mqttFailCount < 250) mqttFailCount++;
    int st = mqttClient.state();
    addLog("MQTT failed, rc=" + String(st) + " (" + mqttStateText(st) + "), retry " + String(interval/1000) + "s");
  }
}

void applyMqttSettings() {
  mqttSubscribedTopic = "";
  if (mqttClient.connected()) publishAvailability("offline");
  mqttClient.disconnect();
  strncpy(mqttServerBuf, mqttServer.c_str(), sizeof(mqttServerBuf) - 1);
  mqttServerBuf[sizeof(mqttServerBuf) - 1] = '\0';
  mqttClient.setServer(mqttServerBuf, mqttPort);
  mqttFailCount = 0;
  mqttHadAttempt = false;
  if (mqttEnabled) {
    addLog("Applying MQTT settings, reconnecting...");
    mqttForceReconnect = true;
    reconnectMQTT();
  } else {
    addLog("MQTT settings applied, but MQTT is disabled");
  }
}

void scheduleMqttRestart() {
  mqttRestartPending = true;
  mqttRestartAt = millis() + MQTT_APPLY_DELAY_MS;
}

void updateMqttTopic() {
  String p = mqttPrefix; p.trim();
  while (p.endsWith("/")) p.remove(p.length() - 1);
  if (p.isEmpty()) p = "lampa";
  mqttPrefix = p;
  String newSet = p + "/set";
  if (newSet == topicSet) return;
  if (mqttClient.connected() && mqttSubscribedTopic.length()) {
    mqttClient.unsubscribe(mqttSubscribedTopic.c_str()); mqttSubscribedTopic = "";
  }
  topicState = p + "/state"; topicSet = newSet;
  topicAlarms = p + "/alarms"; topicAvail = p + "/availability";
  addLog("MQTT topics: " + topicState + ", " + topicSet + ", " + topicAvail);
  if (mqttClient.connected()) {
    if (mqttClient.subscribe(topicSet.c_str())) mqttSubscribedTopic = topicSet;
    publishAvailability("online");
    needPublishDiscovery = haDiscovery; needPublishState = true; needPublishAlarms = true;
  }
}
String minutesToHHMM(int m) {
  if (m < 0 || m > 1439) return "";
  char b[6]; snprintf(b, sizeof(b), "%02d:%02d", m / 60, m % 60); return String(b);
}
int parseHHMM(const String& s) {
  String t = s; t.trim();
  if (!t.length() || t == "--:--" || t.equalsIgnoreCase("none") ||
      t.equalsIgnoreCase("off") || t == "-") return -1;
  int c = t.indexOf(':'); if (c < 1) return -2;
  int h = t.substring(0, c).toInt(); int m = t.substring(c + 1).toInt();
  if (h < 0 || h > 23 || m < 0 || m > 59) return -2;
  return h * 60 + m;
}
bool mqttPublishJson(const String& topic, const String& json, const char* what) {
  uint16_t b = mqttClient.getBufferSize();
  if (json.length() + topic.length() + 5 > b) { addLog(String(what) + ": JSON не влезает в буфер"); return false; }
  if (mqttClient.publish(topic.c_str(), (const uint8_t*)json.c_str(), json.length(), true)) {
    DBG_TAG("MQTT>", String(what) + " -> " + topic + " (" + String(json.length()) + "B)");
    return true;
  }
  int st = mqttClient.state();
  addLog(String(what) + ": FAILED rc=" + String(st) + " (" + mqttStateText(st) + ")"); return false;
}
void publishAvailability(const char* p) {
  if (!mqttClient.connected()) return;
  mqttClient.publish(topicAvail.c_str(), (const uint8_t*)p, strlen(p), true);
  DBG_TAG("MQTT>", String("availability=") + p);
}

// ============ HA DISCOVERY ============
static void fillHaDevice(JsonObject dev) {
  JsonArray ids = dev.createNestedArray("identifiers");
  ids.add(mqttClientId);
  dev["name"] = deviceHostname;
  dev["model"] = DEVICE_MODEL;
  dev["manufacturer"] = DEVICE_MANUFACTURER;
  dev["sw_version"] = FIRMWARE_VERSION;
  dev["configuration_url"] = "http://" + WiFi.localIP().toString();
}

static void haPublishConfig(const char* component, const String& uid, JsonDocument& d) {
  d["unique_id"] = mqttClientId + "_" + uid;
  d["availability_topic"] = topicAvail;
  d["payload_available"] = "online";
  d["payload_not_available"] = "offline";
  d["qos"] = 0;
  fillHaDevice(d.createNestedObject("device"));
  if (d.overflowed()) { addLog("Discovery " + uid + ": JSON overflow"); return; }
  String json; serializeJson(d, json);
  String topic = String(HA_DISCOVERY_PREFIX) + "/" + component + "/" + mqttClientId + "_" + uid + "/config";
  mqttPublishJson(topic, json, "Discovery");
  mqttClient.loop();
}

struct HaNum {
  const char* uid; const char* name; const char* icon;
  const char* vTpl; const char* cmdTpl;
  float mn, mx, step; const char* mode; bool config;
};
static const HaNum HA_NUMS[] = {
  {"speed",          "Скорость эффекта",    "mdi:speedometer",       "{{ value_json.speed }}",           "SPEED:{{ value | int }}",           1, 100, 1, "slider", false},
  {"scale",          "Масштаб эффекта",     "mdi:relative-scale",    "{{ value_json.scale }}",           "SCALE:{{ value | int }}",           1, 100, 1, "slider", false},
  {"brightness_now", "Яркость (число)",     "mdi:brightness-6",      "{{ value_json.brightness }}",      "BRIGHTNESS:{{ value | int }}",      1, 255, 1, "slider", false},
  {"max_brightness", "Максимальная яркость","mdi:brightness-percent","{{ value_json.max_brightness }}",  "MAX_BRIGHTNESS:{{ value | int }}",  1, 255, 1, "slider", true},
  {"timezone",       "Часовой пояс",        "mdi:map-clock",         "{{ value_json.timezone_hours }}",  "TIMEZONE:{{ value }}",            -12,  14, 0.5f, "box",   true},
};

struct HaText {
  const char* uid; const char* name; const char* icon;
  const char* topicKind;
  const char* vTpl; const char* cmdTpl; const char* pattern; int maxLen;
};
static const HaText HA_TEXTS[] = {
  {"ntp",         "NTP-сервер",  "mdi:server-network", "state",  "{{ value_json.ntp_server }}",  "NTP:{{ value }}",         nullptr,                            64},
  {"hostname",    "Сетевое имя", "mdi:lan",            "state",  "{{ value_json.hostname }}",    "HOSTNAME:{{ value }}",    "^[A-Za-z0-9-]{1,32}$",             32},
  {"mqtt_prefix", "MQTT-префикс","mdi:label-outline",  "state",  "{{ value_json.mqtt_prefix }}", "MQTT_PREFIX:{{ value }}", "^[A-Za-z0-9_/-]{1,32}$",           32},
};

struct HaBtn { const char* uid; const char* name; const char* payload; const char* icon; const char* devClass; bool config; };
static const HaBtn HA_BTNS[] = {
  {"reboot",       "Перезагрузка",           "REBOOT",       "mdi:restart",         "restart", true},
  {"ntp_sync",     "Синхронизировать время", "NTP_SYNC",     "mdi:clock-check",     nullptr,   true},
  {"next_effect",  "Следующий эффект",       "NEXT_EFFECT",  "mdi:skip-next",       nullptr,   false},
  {"prev_effect",  "Предыдущий эффект",      "PREV_EFFECT",  "mdi:skip-previous",   nullptr,   false},
  {"alarms_clear", "Очистить все будильники","ALARM:CLEAR_ALL","mdi:alarm-off",     nullptr,   true},
  {"state_refresh","Обновить состояние",     "STATUS",       "mdi:refresh",         nullptr,   true},
};

struct HaSens { const char* uid; const char* name; const char* vTpl; const char* unit; const char* devClass; const char* stateClass; const char* icon; };
static const HaSens HA_SENSORS[] = {
  {"heap",   "Свободная память",       "{{ value_json.free_heap }}", "B",   nullptr,           "measurement", "mdi:memory"},
  {"uptime", "Время работы",           "{{ value_json.uptime_s }}",  "s",   "duration",        "measurement", "mdi:timer-outline"},
  {"rssi",   "Уровень сигнала Wi-Fi",  "{{ value_json.rssi }}",      "dBm", "signal_strength", "measurement", nullptr},
  {"ip",     "IP-адрес",               "{{ value_json.ip }}",        nullptr,nullptr,          nullptr,       "mdi:ip-network"},
};

static const char* const DAY_NAMES_RU[7] = {"Вс","Пн","Вт","Ср","Чт","Пт","Сб"};
static const int DAY_ORDER[7] = {1,2,3,4,5,6,0};

void publishDiscovery() {
  if (!mqttEnabled || !haDiscovery || !mqttClient.connected()) return;

  {
    StaticJsonDocument<2048> d;
    d["name"] = "Lampa";
    d["unique_id"] = mqttClientId + "_light";
    d["object_id"] = deviceHostname;
    d["schema"] = "json";
    d["state_topic"] = topicState;
    d["command_topic"] = topicSet;
    d["availability_topic"] = topicAvail;
    d["payload_available"] = "online";
    d["payload_not_available"] = "offline";
    d["brightness"] = true;
    d["brightness_scale"] = 255;
    JsonArray modes = d.createNestedArray("supported_color_modes"); modes.add("rgb");
    d["effect"] = true;
    JsonArray fx = d.createNestedArray("effect_list");
    for (int i = 0; i < EFFECT_COUNT; i++) fx.add(EFFECTS[i].id);
    d["icon"] = "mdi:led-strip-variant";
    d["qos"] = 0; d["retain"] = false;
    fillHaDevice(d.createNestedObject("device"));
    String json; serializeJson(d, json);
    mqttPublishJson(String(HA_DISCOVERY_PREFIX) + "/light/" + mqttClientId + "/config", json, "Discovery light");
    mqttClient.loop();
  }

  {
    StaticJsonDocument<1792> d;
    d["name"] = "Эффект";
    d["state_topic"] = topicState;
    d["value_template"] = "{{ value_json.effect }}";
    d["command_topic"] = topicSet;
    d["command_template"] = "EFFECT:{{ value }}";
    JsonArray opts = d.createNestedArray("options");
    for (int i = 0; i < EFFECT_COUNT; i++) opts.add(EFFECTS[i].id);
    d["icon"] = "mdi:auto-fix";
    haPublishConfig("select", "effect", d);
  }

  for (auto& r : HA_NUMS) {
    StaticJsonDocument<1024> d;
    d["name"] = r.name;
    d["state_topic"] = topicState;
    d["value_template"] = r.vTpl;
    d["command_topic"] = topicSet;
    d["command_template"] = r.cmdTpl;
    d["min"] = r.mn; d["max"] = r.mx; d["step"] = r.step;
    d["mode"] = r.mode; d["icon"] = r.icon;
    if (r.config) d["entity_category"] = "config";
    haPublishConfig("number", r.uid, d);
  }

  for (auto& r : HA_TEXTS) {
    StaticJsonDocument<1024> d;
    d["name"] = r.name;
    d["state_topic"] = (strcmp(r.topicKind, "alarms") == 0) ? topicAlarms : topicState;
    d["value_template"] = r.vTpl;
    d["command_topic"] = topicSet;
    d["command_template"] = r.cmdTpl;
    d["min"] = 0; d["max"] = r.maxLen;
    if (r.pattern) d["pattern"] = r.pattern;
    d["icon"] = r.icon;
    d["entity_category"] = "config";
    haPublishConfig("text", r.uid, d);
  }

  for (int k = 0; k < 7; k++) {
    int day = DAY_ORDER[k];
    String ds = String(day);
    String pfx = String(k + 1) + ". ";
    {
      StaticJsonDocument<1024> d;
      d["name"] = pfx + "Будильник " + DAY_NAMES_RU[day];
      d["state_topic"] = topicAlarms;
      d["value_template"] = "{{ 'ON' if value_json.alarms[" + ds + "].enabled else 'OFF' }}";
      d["state_on"] = "ON"; d["state_off"] = "OFF";
      d["command_topic"] = topicSet;
      d["payload_on"]  = "ALARM:ENABLE:" + ds + ":1";
      d["payload_off"] = "ALARM:ENABLE:" + ds + ":0";
      d["icon"] = "mdi:alarm";
      d["entity_category"] = "config";
      haPublishConfig("switch", "alarm_en_" + ds, d);
    }
    const char* tPat = "^$|^([01][0-9]|2[0-3]):[0-5][0-9]$";
    const char* onNames[2] = {" включение", " выключение"};
    const char* vTpls[2]   = {"{{ value_json.alarms[%d].on_time }}",  "{{ value_json.alarms[%d].off_time }}"};
    const char* cTpls[2]   = {"ALARM_ON:%d:{{ value }}",              "ALARM_OFF:%d:{{ value }}"};
    const char* icons[2]   = {"mdi:weather-sunset-up",                "mdi:weather-sunset-down"};
    for (int k2 = 0; k2 < 2; k2++) {
      StaticJsonDocument<1024> d;
      d["name"] = pfx + DAY_NAMES_RU[day] + onNames[k2];
      d["state_topic"] = topicAlarms;
      String vt = vTpls[k2]; vt.replace("%d", ds);
      String ct = cTpls[k2]; ct.replace("%d", ds);
      d["value_template"] = vt;
      d["command_topic"] = topicSet;
      d["command_template"] = ct;
      d["min"] = 0; d["max"] = 5;
      d["pattern"] = tPat;
      d["icon"] = icons[k2];
      d["entity_category"] = "config";
      String uid = (k2 == 0 ? "alarm_on_" : "alarm_off_") + ds;
      haPublishConfig("text", uid, d);
    }
  }

  for (auto& r : HA_BTNS) {
    StaticJsonDocument<896> d;
    d["name"] = r.name;
    d["command_topic"] = topicSet;
    d["payload_press"] = r.payload;
    d["icon"] = r.icon;
    if (r.devClass) d["device_class"] = r.devClass;
    if (r.config) d["entity_category"] = "config";
    haPublishConfig("button", r.uid, d);
  }

  for (auto& r : HA_SENSORS) {
    StaticJsonDocument<1024> d;
    d["name"] = r.name;
    d["state_topic"] = topicState;
    d["value_template"] = r.vTpl;
    if (r.unit) d["unit_of_measurement"] = r.unit;
    if (r.devClass) d["device_class"] = r.devClass;
    if (r.stateClass) d["state_class"] = r.stateClass;
    if (r.icon) d["icon"] = r.icon;
    d["entity_category"] = "diagnostic";
    haPublishConfig("sensor", r.uid, d);
  }

  {
    StaticJsonDocument<1024> d;
    d["name"] = "Реакция на звук";
    d["state_topic"] = topicState;
    d["value_template"] = "{{ 'ON' if value_json.sound_enabled else 'OFF' }}";
    d["state_on"]  = "ON";
    d["state_off"] = "OFF";
    d["command_topic"] = topicSet;
    d["payload_on"]  = "SOUND_ENABLE:1";
    d["payload_off"] = "SOUND_ENABLE:0";
    d["icon"] = "mdi:microphone";
    d["entity_category"] = "config";
    haPublishConfig("switch", "sound_enable", d);
  }
  {
    StaticJsonDocument<1024> d;
    d["name"] = "Чувствительность микрофона";
    d["state_topic"] = topicState;
    d["value_template"] = "{{ value_json.sound_sensitivity }}";
    d["command_topic"] = topicSet;
    d["command_template"] = "SOUND_SENSITIVITY:{{ value | int }}";
    d["min"] = 1; d["max"] = 100; d["step"] = 1;
    d["mode"] = "slider";
    d["icon"] = "mdi:volume-high";
    d["entity_category"] = "config";
    haPublishConfig("number", "sound_sensitivity", d);
  }
  {
    StaticJsonDocument<1024> d;
    d["name"] = "Режим влияния звука";
    d["state_topic"] = topicState;
    d["value_template"] = "{{ value_json.sound_mode }}";
    d["command_topic"] = topicSet;
    d["command_template"] = "SOUND_MODE:{{ value }}";
    JsonArray opts = d.createNestedArray("options");
    opts.add(0); opts.add(1); opts.add(2);
    d["icon"] = "mdi:tune";
    d["entity_category"] = "config";
    haPublishConfig("select", "sound_mode", d);
  }
  {
    StaticJsonDocument<1024> d;
    d["name"] = "Уровень звука";
    d["state_topic"] = topicState;
    d["value_template"] = "{{ value_json.sound_level }}";
    d["icon"] = "mdi:waveform";
    d["state_class"] = "measurement";
    d["entity_category"] = "diagnostic";
    haPublishConfig("sensor", "sound_level", d);
  }

  addLog("HA discovery published");
}

void removeDiscovery() {
  if (!mqttClient.connected()) return;
  auto clr = [](const String& t){ mqttClient.publish(t.c_str(), (const uint8_t*)"", 0, true); mqttClient.loop(); };
  String base = String(HA_DISCOVERY_PREFIX) + "/";
  clr(base + "light/"    + mqttClientId + "/config");
  clr(base + "select/"   + mqttClientId + "_effect/config");
  for (auto& r : HA_NUMS)    clr(base + "number/" + mqttClientId + "_" + r.uid + "/config");
  for (auto& r : HA_TEXTS)   clr(base + "text/"   + mqttClientId + "_" + r.uid + "/config");
  for (auto& r : HA_BTNS)    clr(base + "button/" + mqttClientId + "_" + r.uid + "/config");
  for (auto& r : HA_SENSORS) clr(base + "sensor/" + mqttClientId + "_" + r.uid + "/config");
  for (int d = 0; d < 7; d++) {
    String ds = String(d);
    clr(base + "switch/" + mqttClientId + "_alarm_en_"  + ds + "/config");
    clr(base + "text/"   + mqttClientId + "_alarm_on_"  + ds + "/config");
    clr(base + "text/"   + mqttClientId + "_alarm_off_" + ds + "/config");
  }
  clr(base + "sensor/" + mqttClientId + "_local_time/config");
  clr(base + "switch/" + mqttClientId + "_sound_enable/config");
  clr(base + "number/" + mqttClientId + "_sound_sensitivity/config");
  clr(base + "select/" + mqttClientId + "_sound_mode/config");
  clr(base + "sensor/" + mqttClientId + "_sound_level/config");
  addLog("HA discovery config removed");
}

void publishState() {
  if (!mqttEnabled || !mqttClient.connected()) { addLog("publishState: not connected"); return; }
  lastStatePublish = millis();
  StaticJsonDocument<2048> doc;
  doc["state"] = lampOn ? "ON" : "OFF";
  doc["brightness"] = brightness;
  doc["color_mode"] = "rgb";
  JsonObject col = doc.createNestedObject("color");
  col["r"] = solidColor.r; col["g"] = solidColor.g; col["b"] = solidColor.b;
  doc["effect"] = currentEffect;
  doc["type"] = "status";
  doc["on"] = lampOn;
  doc["max_brightness"] = maxBrightness;
  doc["color_rgb"] = String(solidColor.r) + "," + String(solidColor.g) + "," + String(solidColor.b);
  doc["speed"] = effectSpeed;
  doc["scale"] = effectScale;
  doc["timezone_minutes"] = timezoneOffsetMinutes;
  doc["timezone_hours"] = timezoneOffsetMinutes / 60.0;
  doc["ntp_server"] = ntpServer;
  doc["led_count"] = LED_COUNT;
  doc["zigzag_width"] = ZIGZAG_WIDTH;
  doc["mqtt_enabled"] = mqttEnabled;
  doc["mqtt_server"] = mqttServer;
  doc["mqtt_port"] = mqttPort;
  doc["mqtt_user"] = mqttUser;
  doc["mqtt_client_id"] = mqttClientId;
  doc["mqtt_prefix"] = mqttPrefix;
  doc["model"] = DEVICE_MODEL;
  doc["manufacturer"] = DEVICE_MANUFACTURER;
  doc["hostname"] = deviceHostname;
  doc["sw_version"] = FIRMWARE_VERSION;
  doc["ha_discovery"] = haDiscovery;
  doc["free_heap"] = ESP.getFreeHeap();
  doc["uptime_s"] = (uint32_t)(millis() / 1000UL);
  doc["rssi"] = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;
  doc["ip"] = WiFi.localIP().toString();
  doc["sound_enabled"]     = soundEnabled;
  doc["sound_sensitivity"] = soundSensitivity;
  doc["sound_mode"]        = soundMode;
  doc["sound_level"]       = soundLevel;
  if (doc.overflowed()) addLog("publishState: JSON overflow");
  String json; size_t len = serializeJson(doc, json);
  if (len > MQTT_MAX_PACKET_SIZE) { addLog("State JSON too big: " + String(len)); return; }
  mqttPublishJson(topicState, json, "State");
}

void publishAlarmState() {
  if (!mqttEnabled || !mqttClient.connected()) { addLog("publishAlarmState: not connected"); return; }
  StaticJsonDocument<3072> doc;
  doc["type"] = "alarms";
  doc["timezone_minutes"] = timezoneOffsetMinutes;
  doc["timezone_hours"] = timezoneOffsetMinutes / 60.0;
  doc["ntp_server"] = ntpServer;
  doc["hostname"] = deviceHostname;
  JsonArray alarms = doc.createNestedArray("alarms");
  for (int i = 0; i < 7; i++) {
    JsonObject a = alarms.createNestedObject();
    a["day"] = i; a["enabled"] = alarmEnabled[i];
    a["on"] = alarmOnMinutes[i]; a["off"] = alarmOffMinutes[i];
    a["on_time"] = minutesToHHMM(alarmOnMinutes[i]);
    a["off_time"] = minutesToHHMM(alarmOffMinutes[i]);
  }
  if (doc.overflowed()) addLog("publishAlarmState: JSON overflow");
  String json; serializeJson(doc, json);
  mqttPublishJson(topicAlarms, json, "Alarms");
}

// ============ КОМАНДЫ ============
static void cmdBrightness(const String& a) { setBrightness(a.toInt()); saveCurrentEffectSettings(); }
static void cmdMaxBrightness(const String& a) {
  int v = constrain(a.toInt(), (int)MIN_BRIGHTNESS, 255);
  maxBrightness = v; saveSettings();
  if (brightness > maxBrightness) { setBrightness(maxBrightness); saveCurrentEffectSettings(); }
  addLog("Max brightness=" + String(maxBrightness));
}
static void cmdColor(const String& a) {
  int r, g, b;
  if (a.length() == 6) {
    long n = strtol(a.c_str(), NULL, 16);
    r = (n >> 16) & 0xFF; g = (n >> 8) & 0xFF; b = n & 0xFF;
  } else {
    int c1 = a.indexOf(','), c2 = a.lastIndexOf(',');
    if (c1 < 0 || c2 < 0 || c1 == c2) { addLog("COLOR: bad format"); return; }
    r = a.substring(0, c1).toInt();
    g = a.substring(c1 + 1, c2).toInt();
    b = a.substring(c2 + 1).toInt();
  }
  setSolidColor(constrain(r,0,255), constrain(g,0,255), constrain(b,0,255));
  currentEffect = "solid";
  int idx = getEffectIndex(currentEffect);
  if (idx >= 0) { loadEffectSettings(idx); resetEffectState(currentEffect); }
}
static void cmdEffect(const String& a) {
  String e; if (!trimArg(a, e)) return;
  int idx = getEffectIndex(e);
  if (idx >= 0) { currentEffect = e; loadEffectSettings(idx); resetEffectState(e); }
  else addLog("Unknown effect: " + e);
}
static void cmdSpeed(const String& a) { effectSpeed = constrain(a.toInt(), 1, 100); saveCurrentEffectSettings(); }
static void cmdScale(const String& a) { effectScale = constrain(a.toInt(), 1, 100); saveCurrentEffectSettings(); }
static void cmdTimezone(const String& a) {
  float h = a.toFloat();
  if (h < -12 || h > 14) return;
  timezoneOffsetMinutes = (int)round(h * 60);
  saveSettings();
  addLog("Timezone set to " + String(h) + "h");
}
static void cmdNtp(const String& a) {
  String s; if (!trimArg(a, s)) return;
  ntpServer = s; saveSettings(); syncTime();
  addLog("NTP server=" + ntpServer);
}
static void cmdAlarmSet(const String& a) {
  int p[5]; int n = 0; int start = 0;
  for (int i = 0; i <= (int)a.length() && n < 5; i++) {
    if (i == (int)a.length() || a[i] == ':') { p[n++] = a.substring(start, i).toInt(); start = i + 1; }
  }
  if (n != 5) { addLog("ALARM:SET: bad format"); return; }
  int day = p[0], onH = p[1], onM = p[2], offH = p[3], offM = p[4];
  if (day < 0 || day > 6 || onH < 0 || onH > 23 || onM < 0 || onM > 59 ||
      offH < 0 || offH > 23 || offM < 0 || offM > 59) return;
  alarmOnMinutes[day] = onH * 60 + onM;
  alarmOffMinutes[day] = offH * 60 + offM;
  alarmEnabled[day] = true;
  saveSettings(); needPublishAlarms = true;
  addLog("Alarm set day " + String(day));
}
static void cmdAlarmEnable(const String& a) {
  int c = a.indexOf(':'); if (c < 0) return;
  int day = a.substring(0, c).toInt();
  int en = a.substring(c + 1).toInt();
  if (day < 0 || day > 6) return;
  alarmEnabled[day] = (en == 1);
  saveSettings(); needPublishAlarms = true;
  addLog("Alarm day " + String(day) + " enabled=" + String(en));
}
static void cmdAlarmTime(const String& a, bool isOn) {
  int c = a.indexOf(':'); if (c < 0) { addLog("ALARM time: bad format"); return; }
  int day = a.substring(0, c).toInt();
  String ts = a.substring(c + 1); ts.trim();
  if (day < 0 || day > 6) return;
  int mins = parseHHMM(ts);
  if (mins == -2) { addLog("ALARM bad time"); return; }
  if (isOn) alarmOnMinutes[day] = mins; else alarmOffMinutes[day] = mins;
  if (mins < 0 && alarmOnMinutes[day] < 0 && alarmOffMinutes[day] < 0) alarmEnabled[day] = false;
  saveSettings(); needPublishAlarms = true;
  addLog(String(isOn ? "Alarm ON " : "Alarm OFF ") + "day " + String(day) + "=" + minutesToHHMM(mins));
}
static void cmdAlarmOn(const String& a)  { cmdAlarmTime(a, true);  }
static void cmdAlarmOff(const String& a) { cmdAlarmTime(a, false); }
static void cmdAlarmClear(const String& a) {
  int day = a.toInt();
  if (day < 0 || day > 6) return;
  alarmOnMinutes[day] = -1; alarmOffMinutes[day] = -1; alarmEnabled[day] = false;
  saveSettings(); needPublishAlarms = true;
  addLog("Alarm cleared day " + String(day));
}
static void cmdMqttEnable(const String& a) {
  bool was = mqttEnabled;
  mqttEnabled = (a.toInt() == 1);
  saveSettings();
  if (mqttEnabled && !was) {
    addLog("MQTT enabled");
    setupMQTT(); mqttFailCount = 0; mqttHadAttempt = false;
    scheduleMqttRestart();
  } else if (!mqttEnabled && was) {
    addLog("MQTT disabled");
    mqttSubscribedTopic = ""; publishAvailability("offline"); mqttClient.disconnect();
  }
}
static void cmdHaDiscovery(const String& a) {
  bool was = haDiscovery;
  haDiscovery = (a.toInt() == 1);
  saveSettings();
  if (haDiscovery && !was) {
    addLog("HA discovery enabled");
    if (mqttClient.connected()) { mqttClient.subscribe(HA_STATUS_TOPIC); needPublishDiscovery = true; }
  } else if (!haDiscovery && was) {
    addLog("HA discovery disabled");
    if (mqttClient.connected()) { mqttClient.unsubscribe(HA_STATUS_TOPIC); removeDiscovery(); }
  }
}
static void cmdMqttServer(const String& a)   { String s; if (!trimArg(a, s)) return; mqttServer = s;   saveSettings(); scheduleMqttRestart(); addLog("MQTT server=" + s); }
static void cmdMqttPort(const String& a)     { int p = a.toInt(); if (p <= 0 || p > 65535) return; mqttPort = p; saveSettings(); scheduleMqttRestart(); addLog("MQTT port=" + String(p)); }
static void cmdMqttUser(const String& a)     { String s; if (!trimArg(a, s)) return; mqttUser = s; saveSettings(); scheduleMqttRestart(); addLog("MQTT user=" + s); }
static void cmdMqttPassword(const String& a) { String s; if (!trimArg(a, s)) return; mqttPassword = s; saveSettings(); scheduleMqttRestart(); addLog("MQTT password updated"); }
static void cmdMqttPrefix(const String& a) {
  String s; if (!trimArg(a, s)) return;
  mqttPrefix = s; updateMqttTopic(); saveSettings(); addLog("MQTT prefix=" + s);
}
static void cmdHostname(const String& a) {
  String s; if (!trimArg(a, s)) return;
  String clean = sanitizeHostname(s);
  if (clean == deviceHostname) { addLog("Hostname unchanged"); return; }
  deviceHostname = clean;
  saveSettings(); applyHostname();
  wifiRestartPending = true; wifiRestartAt = millis() + 2000;
  addLog("Hostname scheduled: " + deviceHostname);
  needPublishDiscovery = haDiscovery; needPublishState = true;
}

static void cmdSoundEnable(const String& a) {
  bool was = soundEnabled;
  soundEnabled = (a.toInt() == 1);
  if (!soundEnabled && was) { soundLevel = 0; soundBeatFlash = 0; }
  saveSettings();
  addLog(String("Sound ") + (soundEnabled ? "enabled" : "disabled"));
}
static void cmdSoundSensitivity(const String& a) {
  int v = constrain(a.toInt(), 1, 100);
  soundSensitivity = (uint8_t)v; saveSettings();
  addLog("Sound sensitivity=" + String(v));
}
static void cmdSoundMode(const String& a) {
  int v = constrain(a.toInt(), 0, 2);
  soundMode = (uint8_t)v; saveSettings();
  addLog("Sound mode=" + String(v) +
         (v == 0 ? " (brightness)" : v == 1 ? " (speed)" : " (scale)"));
}

struct SimpleCmd { const char* name; void (*fn)(); };
static const SimpleCmd SIMPLE_CMDS[] = {
  {"ON",            [](){ lampOn = true; }},
  {"OFF",           [](){ lampOn = false; }},
  {"TOGGLE",        [](){ lampOn = !lampOn; }},
  {"NEXT_EFFECT",   [](){ switchEffectBy(1); }},
  {"PREV_EFFECT",   [](){ switchEffectBy(-1); }},
  {"STATUS",        [](){ needPublishAlarms = true; }},
  {"ALARM:STATUS",  [](){ needPublishAlarms = true; }},
  {"NTP_SYNC",      [](){ addLog("NTP sync requested"); syncTime(); }},
  {"ALARM:CLEAR_ALL", [](){
    for (int i = 0; i < 7; i++) { alarmOnMinutes[i] = -1; alarmOffMinutes[i] = -1; alarmEnabled[i] = false; }
    saveSettings(); needPublishAlarms = true; addLog("All alarms cleared");
  }},
};

struct PrefixCmd { const char* prefix; void (*fn)(const String&); };
static const PrefixCmd PREFIX_CMDS[] = {
  {"BRIGHTNESS:",     cmdBrightness},
  {"MAX_BRIGHTNESS:", cmdMaxBrightness},
  {"COLOR:",          cmdColor},
  {"EFFECT:",         cmdEffect},
  {"SPEED:",          cmdSpeed},
  {"SCALE:",          cmdScale},
  {"TIMEZONE:",       cmdTimezone},
  {"NTP:",            cmdNtp},
  {"ALARM:SET:",      cmdAlarmSet},
  {"ALARM:ENABLE:",   cmdAlarmEnable},
  {"ALARM_ON:",       cmdAlarmOn},
  {"ALARM_OFF:",      cmdAlarmOff},
  {"ALARM:CLEAR:",    cmdAlarmClear},
  {"MQTT_ENABLE:",    cmdMqttEnable},
  {"HA_DISCOVERY:",   cmdHaDiscovery},
  {"MQTT_SERVER:",    cmdMqttServer},
  {"MQTT_PORT:",      cmdMqttPort},
  {"MQTT_USER:",      cmdMqttUser},
  {"MQTT_PASSWORD:",  cmdMqttPassword},
  {"MQTT_PREFIX:",    cmdMqttPrefix},
  {"HOSTNAME:",       cmdHostname},
  {"SOUND_ENABLE:",      cmdSoundEnable},
  {"SOUND_SENSITIVITY:", cmdSoundSensitivity},
  {"SOUND_MODE:",        cmdSoundMode},
};

void processMQTTCommand(String command) {
  command.trim();
  DBG_TAG("CMD", command);
  if (command.equalsIgnoreCase("REBOOT")) {
    addLog("Reboot requested");
    publishAvailability("offline");
    rebootPending = true; rebootAt = millis() + 1500;
    return;
  }
  if (command.equalsIgnoreCase("DISCOVERY_PUBLISH")) { needPublishDiscovery = haDiscovery; return; }

  for (auto& c : SIMPLE_CMDS) {
    if (command.equalsIgnoreCase(c.name)) { c.fn(); needPublishState = true; return; }
  }
  for (auto& c : PREFIX_CMDS) {
    if (command.startsWith(c.prefix)) {
      c.fn(command.substring(strlen(c.prefix)));
      needPublishState = true;
      return;
    }
  }
  addLog("Unknown MQTT command: " + command);
}

// ============ КНОПКА ============
void switchEffectBy(int delta) {
  int cur = getEffectIndex(currentEffect); if (cur < 0) cur = 0;
  int next = (cur + delta + EFFECT_COUNT) % EFFECT_COUNT;
  currentEffect = String(EFFECTS[next].id);
  loadEffectSettings(next); resetEffectState(currentEffect);
  addLog("Effect changed to: " + currentEffect);
  needPublishState = true;
}
static void enterAdjustMode(int clicks) {
  switch (clicks) {
    case 0: buttonMode = MODE_BRIGHTNESS; brightnessDirection = -brightnessDirection;
            addLog("Brightness mode (" + String(brightnessDirection > 0 ? "+" : "-") + ")"); break;
    case 1: buttonMode = MODE_SPEED; speedDirection = -speedDirection;
            addLog("Speed mode (" + String(speedDirection > 0 ? "+" : "-") + ")"); break;
    case 2: buttonMode = MODE_SCALE; scaleDirection = -scaleDirection;
            addLog("Scale mode (" + String(scaleDirection > 0 ? "+" : "-") + ")"); break;
    default: buttonMode = MODE_NORMAL; break;
  }
}
static void exitAdjustMode() {
  if (buttonMode == MODE_NORMAL) return;
  saveCurrentEffectSettings();
  addLog(buttonMode == MODE_BRIGHTNESS ? ("Brightness saved: " + String(brightness))
                                        : ("Effect settings saved for " + currentEffect));
  buttonMode = MODE_NORMAL;
  needPublishState = true;
}
void handleButton() {
  static int stableState = LOW, lastReading = LOW;
  static unsigned long lastDebounceTime = 0, lastAdjustTime = 0;
  unsigned long now = millis();
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastReading) { lastReading = reading; lastDebounceTime = now; }
  if ((now - lastDebounceTime) >= DEBOUNCE_DELAY && reading != stableState) {
    stableState = reading;
    if (stableState == HIGH) {
      buttonPressed = true; pressStartTime = now; longPressProcessed = false;
      if (!isInSequence) { isInSequence = true; clickCount = 0; }
      waitingForClick = false;
    } else {
      buttonPressed = false;
      if (longPressProcessed) { exitAdjustMode(); isInSequence = false; waitingForClick = false; clickCount = 0; }
      else { clickCount++; lastButtonRelease = now; waitingForClick = true; }
    }
  }
  if (buttonPressed && !longPressProcessed && (now - pressStartTime) >= LONG_PRESS_TIME) {
    longPressProcessed = true; enterAdjustMode(clickCount); lastAdjustTime = now; clickCount = 0;
  }
  if (!buttonPressed && isInSequence && waitingForClick && (now - lastButtonRelease) > CLICK_TIMEOUT) {
    isInSequence = false; waitingForClick = false;
    if (clickCount == 1) { lampOn = !lampOn; addLog(String("Toggle ON/OFF: ") + (lampOn ? "ON" : "OFF")); needPublishState = true; }
    else if (clickCount == 2) switchEffectBy(1);
    else if (clickCount == 3) switchEffectBy(-1);
    clickCount = 0;
  }
  if (buttonPressed && longPressProcessed && buttonMode != MODE_NORMAL && (now - lastAdjustTime) >= ADJUST_INTERVAL) {
    lastAdjustTime = now;
    if (buttonMode == MODE_BRIGHTNESS) brightness = constrain((int)brightness + brightnessDirection * BRIGHTNESS_STEP, (int)MIN_BRIGHTNESS, (int)maxBrightness);
    else if (buttonMode == MODE_SPEED) effectSpeed = constrain((int)effectSpeed + speedDirection * SPEED_STEP, 1, 100);
    else if (buttonMode == MODE_SCALE) effectScale = constrain((int)effectScale + scaleDirection * SCALE_STEP, 1, 100);
  }
}
void showValueScale(int value, int mn, int mx, CRGB color) {
  FastLED.clear();
  if (mx <= mn) mx = mn + 1;
  value = constrain(value, mn, mx);
  int num = 1 + ((long)(value - mn) * (ZIGZAG_WIDTH - 1)) / (mx - mn);
  for (int i = 0; i < ZIGZAG_WIDTH; i++) {
    int idx = getLedIndex(i);
    if (idx >= 0 && idx < LED_COUNT) leds[idx] = (i < num) ? color : CRGB::Black;
  }
}

// ============ HTTP ============
void handleRoot() {
  String body = R"HTML(
<div class="time-block">Текущее время: <span id="current-time">—</span></div>
<div class="control"><button onclick="cmd('ON')">Включить</button><button onclick="cmd('OFF')">Выключить</button></div>
<div class="control"><label>Яркость:</label>
  <input type="range" min="1" max="255" value="128" oninput="updateBrightness(this.value)">
  <span id="brightness-value" class="value">128</span>
  <span style="font-size:12px;color:#666">(макс: <span id="max-brightness-display">255</span>)</span></div>
<div class="control"><label>Цвет:</label><input type="color" value="#ffffff" onchange="sendColor(this.value)"></div>
<div class="control"><label>Эффект:</label><select id="effect" onchange="sendEffect(this.value)">%EFFECTS%</select></div>
<div class="control"><label>Скорость:</label>
  <input type="range" min="1" max="100" value="50" oninput="updateSpeed(this.value)">
  <span id="speed-value" class="value">50</span></div>
<div class="control"><label>Масштаб:</label>
  <input type="range" min="1" max="100" value="50" oninput="updateScale(this.value)">
  <span id="scale-value" class="value">50</span></div>
<h3>Звук (INMP441)</h3>
<div class="control"><label>Реакция на звук:</label>
  <input type="checkbox" id="sound_enable" onchange="setSoundEnable(this.checked)"></div>
<div class="control"><label>Чувствительность:</label>
  <input type="range" min="1" max="100" value="50" id="sound_sensitivity"
         oninput="updateSoundSensitivity(this.value)" onchange="saveSoundSensitivity(this.value)">
  <span id="sound-sensitivity-value" class="value">50</span></div>
<div class="control"><label>Что модулировать:</label>
  <select id="sound_mode" onchange="setSoundMode(this.value)">
    <option value="0">Яркость</option>
    <option value="1">Скорость</option>
    <option value="2">Масштаб</option>
  </select></div>
<div class="control"><label>Текущий уровень:</label>
  <span id="sound-level" class="value">0</span> / 255
  <div class="level-bar"><div id="sound-level-bar"></div></div>
</div>
<h3>Журнал событий</h3><div id="log-container" class="log-container"></div>
<script>
function updateBrightness(v){cmd('BRIGHTNESS:'+v);document.getElementById('brightness-value').textContent=v}
function sendColor(h){cmd('COLOR:'+h.substring(1))}
function sendEffect(e){cmd('EFFECT:'+e).then(loadStatus)}
function updateSpeed(v){cmd('SPEED:'+v);document.getElementById('speed-value').textContent=v}
function updateScale(v){cmd('SCALE:'+v);document.getElementById('scale-value').textContent=v}
function setSoundEnable(on){cmd('SOUND_ENABLE:'+(on?1:0))}
function updateSoundSensitivity(v){document.getElementById('sound-sensitivity-value').textContent=v}
function saveSoundSensitivity(v){cmd('SOUND_SENSITIVITY:'+v)}
function setSoundMode(v){cmd('SOUND_MODE:'+v)}
function fetchLogs(){fetch('/api/logs').then(r=>r.json()).then(d=>{var c=document.getElementById('log-container');
c.innerHTML=d.map(escHtml).join('<br>');c.scrollTop=c.scrollHeight}).catch(()=>{})}
function loadStatus(){fetch('/api/status').then(r=>r.json()).then(d=>{
var rs=document.querySelectorAll('input[type=range]');
rs[0].value=d.brightness;document.getElementById('brightness-value').textContent=d.brightness;
document.getElementById('max-brightness-display').textContent=d.max_brightness;
rs[1].value=d.speed;document.getElementById('speed-value').textContent=d.speed;
rs[2].value=d.scale;document.getElementById('scale-value').textContent=d.scale;
document.getElementById('effect').value=d.effect;
var c=d.color.split(',');var h='#'+parseInt(c[0]).toString(16).padStart(2,'0')+parseInt(c[1]).toString(16).padStart(2,'0')+parseInt(c[2]).toString(16).padStart(2,'0');
document.querySelector('input[type=color]').value=h;
document.getElementById('sound_enable').checked = !!d.sound_enabled;
document.getElementById('sound_sensitivity').value = d.sound_sensitivity;
document.getElementById('sound-sensitivity-value').textContent = d.sound_sensitivity;
document.getElementById('sound_mode').value = String(d.sound_mode);
document.getElementById('sound-level').textContent = d.sound_level;
document.getElementById('sound-level-bar').style.width = (d.sound_level/255*100)+'%';
}).catch(()=>{})}
window.onload=function(){loadStatus();updateTime();fetchLogs();setInterval(updateTime,1000);setInterval(fetchLogs,3000);setInterval(loadStatus,1000)};
</script>
)HTML";
  String options;
  for (int i = 0; i < EFFECT_COUNT; i++) {
    options += "<option value=\""; options += EFFECTS[i].id; options += "\">";
    options += EFFECTS[i].ru; options += "</option>";
  }
  body.replace("%EFFECTS%", options);
  sendPage("Lampa Control", body);
}

void handleAPI() {
  String c = server.arg("command");
  if (!c.length()) return server.send(400, "text/plain", "Missing command parameter");
  DBG_TAG("HTTP", "GET /api?command=" + c);
  processMQTTCommand(c);
  server.send(200, "text/plain", "OK");
}

void handleStatus() {
  StaticJsonDocument<2048> doc;
  doc["on"] = lampOn;
  doc["brightness"] = brightness;
  doc["max_brightness"] = maxBrightness;
  doc["effect"] = currentEffect;
  doc["speed"] = effectSpeed;
  doc["scale"] = effectScale;
  doc["color"] = String(solidColor.r) + "," + String(solidColor.g) + "," + String(solidColor.b);
  doc["timezone_minutes"] = timezoneOffsetMinutes;
  doc["timezone_hours"] = timezoneOffsetMinutes / 60.0;
  doc["ntp_server"] = ntpServer;
  doc["led_count"] = LED_COUNT; doc["zigzag_width"] = ZIGZAG_WIDTH;
  doc["mqtt_enabled"] = mqttEnabled; doc["mqtt_server"] = mqttServer;
  doc["mqtt_port"] = mqttPort; doc["mqtt_user"] = mqttUser;
  doc["mqtt_client_id"] = mqttClientId; doc["mqtt_prefix"] = mqttPrefix;
  doc["model"] = DEVICE_MODEL; doc["manufacturer"] = DEVICE_MANUFACTURER;
  doc["hostname"] = deviceHostname; doc["ha_discovery"] = haDiscovery;
  doc["mqtt_connected"] = mqttClient.connected();
  doc["topic_state"] = topicState; doc["topic_set"] = topicSet;
  doc["fw_version"] = FIRMWARE_VERSION;
  doc["ota_ready"] = (esp_ota_get_next_update_partition(NULL) != NULL);
  doc["free_heap"] = ESP.getFreeHeap();
  doc["local_time"] = getLocalTimeString();
  doc["ip"] = WiFi.localIP().toString();
  doc["sound_enabled"]     = soundEnabled;
  doc["sound_sensitivity"] = soundSensitivity;
  doc["sound_mode"]        = soundMode;
  doc["sound_level"]       = soundLevel;
  JsonArray al = doc.createNestedArray("alarms");
  for (int i = 0; i < 7; i++) {
    JsonObject a = al.createNestedObject();
    a["day"] = i; a["enabled"] = alarmEnabled[i];
    a["on"] = alarmOnMinutes[i]; a["off"] = alarmOffMinutes[i];
  }
  String json; serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleTime() { server.send(200, "application/json", "{\"time\":\"" + getLocalTimeString() + "\"}"); }

static String jsonEscape(const String& s) {
  String o; o.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    switch (c) {
      case '"':  o += "\\\""; break;
      case '\\': o += "\\\\"; break;
      case '\n': o += "\\n";  break;
      case '\r': o += "\\r";  break;
      case '\t': o += "\\t";  break;
      default:
        if ((uint8_t)c < 0x20) { char b[8]; snprintf(b, sizeof(b), "\\u%04x", c); o += b; }
        else o += c;
    }
  }
  return o;
}

void handleLogs() {
  String json = "[";
  int start = (logCount < LOG_BUFFER_SIZE) ? 0 : logIndex;
  for (int i = 0; i < logCount; i++) {
    if (i) json += ",";
    json += "\"" + jsonEscape(logBuffer[(start + i) % LOG_BUFFER_SIZE]) + "\"";
  }
  json += "]";
  server.send(200, "application/json", json);
}

void handleAlarmPage() {
  String body = R"HTML(
<div>Текущее локальное время: <span id="current-time">—</span></div>
<table>
<tr><th>День</th><th>Включить</th><th>Время вкл.</th><th>Время выкл.</th></tr>
<tr class="day-row" data-day="1"><td>Понедельник</td><td><input type="checkbox" class="en"></td><td><input type="time" class="on"></td><td><input type="time" class="off"></td></tr>
<tr class="day-row" data-day="2"><td>Вторник</td><td><input type="checkbox" class="en"></td><td><input type="time" class="on"></td><td><input type="time" class="off"></td></tr>
<tr class="day-row" data-day="3"><td>Среда</td><td><input type="checkbox" class="en"></td><td><input type="time" class="on"></td><td><input type="time" class="off"></td></tr>
<tr class="day-row" data-day="4"><td>Четверг</td><td><input type="checkbox" class="en"></td><td><input type="time" class="on"></td><td><input type="time" class="off"></td></tr>
<tr class="day-row" data-day="5"><td>Пятница</td><td><input type="checkbox" class="en"></td><td><input type="time" class="on"></td><td><input type="time" class="off"></td></tr>
<tr class="day-row" data-day="6"><td>Суббота</td><td><input type="checkbox" class="en"></td><td><input type="time" class="on"></td><td><input type="time" class="off"></td></tr>
<tr class="day-row" data-day="0"><td>Воскресенье</td><td><input type="checkbox" class="en"></td><td><input type="time" class="on"></td><td><input type="time" class="off"></td></tr>
</table>
<button onclick="saveAlarms()">Сохранить будильники</button>
<button onclick="loadCurrent()">Загрузить текущие</button>
<script>
function loadCurrent(){fetch('/api/status').then(r=>r.json()).then(d=>{d.alarms.forEach(a=>{
var row=document.querySelector('.day-row[data-day="'+a.day+'"]');if(!row)return;
row.querySelector('.en').checked=a.enabled;
if(a.on>=0&&a.off>=0){
var oh=Math.floor(a.on/60).toString().padStart(2,'0'),om=(a.on%60).toString().padStart(2,'0');
var fh=Math.floor(a.off/60).toString().padStart(2,'0'),fm=(a.off%60).toString().padStart(2,'0');
row.querySelector('.on').value=oh+':'+om;row.querySelector('.off').value=fh+':'+fm;
}else{row.querySelector('.on').value='';row.querySelector('.off').value=''}})})}
function saveAlarms(){var ps=[];document.querySelectorAll('.day-row').forEach(row=>{
var day=row.dataset.day,en=row.querySelector('.en').checked;
var on=row.querySelector('.on').value,off=row.querySelector('.off').value;
if(en&&on&&off){var a=on.split(':').map(Number),b=off.split(':').map(Number);
ps.push(cmdText('ALARM:SET:'+day+':'+a[0]+':'+a[1]+':'+b[0]+':'+b[1]))}
else ps.push(cmdText('ALARM:CLEAR:'+day))});
Promise.all(ps).then(()=>{alert('Сохранено');loadCurrent()})}
window.onload=function(){loadCurrent();updateTime();setInterval(updateTime,1000)};
</script>
)HTML";
  sendPage("Будильники", body);
}

void handleSettingsPage() {
  String body = R"HTML(
<div class="info">
  <b>Версия прошивки:</b> <span id="fw-version">—</span><br>
  <b>Hostname:</b> <span id="hostname-info">—</span><br>
  <b>IP:</b> <span id="ip-info">—</span><br>
  <b>Свободная память:</b> <span id="heap-info">—</span> байт<br>
  <b>OTA готово:</b> <span id="ota-info">—</span>
</div>
<h3>Время</h3><div class="section">
  <div class="row"><label>Часовой пояс (UTC):</label><input type="number" id="tz_offset" min="-12" max="14" step="0.5" value="0"></div>
  <div class="row"><label>NTP сервер:</label><input type="text" id="ntp_server" value="pool.ntp.org"></div>
</div>
<h3>Яркость</h3><div class="section">
  <div class="row"><label>Максимум (1–255):</label><input type="number" id="max_brightness" min="1" max="255" style="width:80px"></div>
</div>
<h3>MQTT и сеть</h3><div class="section">
  <div class="row"><label>Включить MQTT:</label><input type="checkbox" id="mqtt_enable"></div>
  <div class="row"><label>Сервер:</label><input type="text" id="mqtt_server"></div>
  <div class="row"><label>Порт:</label><input type="number" id="mqtt_port" min="1" max="65535"></div>
  <div class="row"><label>Пользователь:</label><input type="text" id="mqtt_user"></div>
  <div class="row"><label>Пароль:</label><input type="password" id="mqtt_password" placeholder="(не менять)"></div>
  <div class="row"><label>MQTT Prefix:</label><input type="text" id="mqtt_prefix"></div>
  <div class="row"><label>Сетевое имя:</label><input type="text" id="hostname"></div>
  <div class="row"><label>HA Discovery:</label><input type="checkbox" id="ha_discovery"></div>
  <div class="info" style="margin-top:8px">
    <b>Статус MQTT:</b> <span id="mqtt-conn">—</span><br>
    <b>State:</b> <code id="t-state">—</code><br>
    <b>Set:</b> <code id="t-set">—</code><br>
    <b>Discovery:</b> <code id="t-disc">—</code>
  </div>
</div>
<button onclick="saveAll()">Сохранить</button><button onclick="loadCurrent()">Загрузить</button>
<h3>Обновление прошивки (OTA)</h3><div class="section">
  <div class="warn">⚠️ Не выключайте питание во время обновления!</div>
  <div class="upload-box">
    <p>Файл <b>.bin</b>:</p>
    <input type="file" id="firmware" accept=".bin"><br>
    <button id="btn-upload" onclick="uploadFirmware()">Загрузить</button>
  </div>
  <div id="bar"><div id="fill"></div></div>
  <div id="status"></div>
  <p style="font-size:13px;color:#666">ArduinoOTA: сетевой порт <b id="ota-port">lampa</b> (пароль: <b>admin</b>)</p>
</div>
<h3>Перезагрузка</h3><div class="section">
  <button class="danger" onclick="rebootDevice()">Перезагрузить</button>
</div>
<script>
function loadCurrent(){fetch('/api/status').then(r=>r.json()).then(d=>{
document.getElementById('fw-version').textContent=d.fw_version||'?';
document.getElementById('hostname-info').textContent=d.hostname||'lampa';
document.getElementById('ota-port').textContent=d.hostname||'lampa';
document.getElementById('ip-info').textContent=d.ip||'';
document.getElementById('heap-info').textContent=d.free_heap||'?';
document.getElementById('ota-info').textContent=d.ota_ready?'да':'НЕТ';
document.getElementById('tz_offset').value=d.timezone_hours;
document.getElementById('ntp_server').value=d.ntp_server;
document.getElementById('max_brightness').value=d.max_brightness;
document.getElementById('mqtt_enable').checked=d.mqtt_enabled;
document.getElementById('mqtt_server').value=d.mqtt_server;
document.getElementById('mqtt_port').value=d.mqtt_port;
document.getElementById('mqtt_user').value=d.mqtt_user||'';
document.getElementById('mqtt_prefix').value=d.mqtt_prefix||'lampa';
document.getElementById('hostname').value=d.hostname||'lampa';
document.getElementById('ha_discovery').checked=d.ha_discovery;
document.getElementById('mqtt-conn').textContent=d.mqtt_enabled?(d.mqtt_connected?'подключено':'нет'):'выключено';
document.getElementById('t-state').textContent=d.topic_state||'—';
document.getElementById('t-set').textContent=d.topic_set||'—';
document.getElementById('t-disc').textContent='homeassistant/light/'+(d.mqtt_client_id||'')+'/config'})}
function saveAll(){var ps=[
cmdText('TIMEZONE:'+document.getElementById('tz_offset').value),
cmdText('NTP:'+document.getElementById('ntp_server').value),
cmdText('MAX_BRIGHTNESS:'+document.getElementById('max_brightness').value),
cmdText('MQTT_ENABLE:'+(document.getElementById('mqtt_enable').checked?1:0)),
cmdText('MQTT_SERVER:'+document.getElementById('mqtt_server').value),
cmdText('MQTT_PORT:'+document.getElementById('mqtt_port').value),
cmdText('MQTT_USER:'+document.getElementById('mqtt_user').value),
cmdText('MQTT_PREFIX:'+document.getElementById('mqtt_prefix').value),
cmdText('HA_DISCOVERY:'+(document.getElementById('ha_discovery').checked?1:0)),
cmdText('HOSTNAME:'+document.getElementById('hostname').value)];
var p=document.getElementById('mqtt_password').value;if(p.length)ps.push(cmdText('MQTT_PASSWORD:'+p));
Promise.all(ps).then(()=>{alert('Настройки сохранены.\nПри смене hostname устройство переподключится к WiFi.');loadCurrent()})}
function uploadFirmware(){var f=document.getElementById('firmware');
if(!f.files.length){alert('Выберите файл .bin');return}
var file=f.files[0];if(!file.name.endsWith('.bin')){alert('.bin');return}
var pw=prompt('Пароль OTA:');if(!pw)return;
var fd=new FormData();fd.append('firmware',file,file.name);
var x=new XMLHttpRequest();x.open('POST','/update',true);
x.setRequestHeader('Authorization','Basic '+btoa('admin:'+pw));
document.getElementById('btn-upload').disabled=true;
document.getElementById('bar').style.display='block';
document.getElementById('status').textContent='Загрузка...';
x.upload.onprogress=function(e){if(e.lengthComputable){var p=Math.round(e.loaded/e.total*100);
document.getElementById('fill').style.width=p+'%';document.getElementById('status').textContent=p+'%'}};
x.onload=function(){if(x.status===200){document.getElementById('status').textContent='✅ Готово, перезагрузка...';
document.getElementById('fill').style.background='#27ae60';setTimeout(()=>location.href='/',8000)}
else if(x.status===401){document.getElementById('status').textContent='❌ Неверный пароль';document.getElementById('btn-upload').disabled=false}
else{document.getElementById('status').textContent='❌ '+x.statusText;document.getElementById('btn-upload').disabled=false}};
x.onerror=function(){document.getElementById('status').textContent='❌ Ошибка сети';document.getElementById('btn-upload').disabled=false};
x.send(fd)}
function rebootDevice(){if(!confirm('Перезагрузить?'))return;
cmdText('REBOOT').then(()=>{alert('Перезагрузка...');setTimeout(()=>location.href='/',5000)})}
window.onload=loadCurrent;
</script>
)HTML";
  sendPage("Настройки", body);
}

// ============ LED / эффекты ============
void setSolidColor(int r, int g, int b) { solidColor = CRGB(r, g, b); }
void setBrightness(int v) { brightness = constrain(v, (int)MIN_BRIGHTNESS, (int)maxBrightness); }
int getLedIndex(int i) {
  int row = i / ZIGZAG_WIDTH, pos = i % ZIGZAG_WIDTH;
  return (row % 2 == 0) ? row * ZIGZAG_WIDTH + pos : (row + 1) * ZIGZAG_WIDTH - 1 - pos;
}
int getLedIndexRotatedMinus90(int i) {
  int row = i / ZIGZAG_WIDTH, col = i % ZIGZAG_WIDTH;
  int nr = ZIGZAG_WIDTH - 1 - col, nc = row;
  int ri = nr * ZIGZAG_WIDTH + nc;
  int r = ri / ZIGZAG_WIDTH, pos = ri % ZIGZAG_WIDTH;
  return (r % 2 == 0) ? r * ZIGZAG_WIDTH + pos : (r + 1) * ZIGZAG_WIDTH - 1 - pos;
}

// ============ ЗВУК (INMP441 I2S, новый драйвер) ============
void updateSoundLevel() {
  if (!soundEnabled) { soundLevel = 0; soundBeatFlash = 0; return; }
  unsigned long now = millis();
  if (now - lastSoundSample < SOUND_SAMPLE_MS) return;
  lastSoundSample = now;

  size_t bytesRead = 0;
  esp_err_t err = i2s_channel_read(i2s_rx_handle, i2sBuffer, sizeof(i2sBuffer), &bytesRead, pdMS_TO_TICKS(I2S_READ_TIMEOUT_MS));

  if (bytesRead == 0) {
#ifdef DEBUG_VERBOSE
    static unsigned long lastWarn = 0;
    if (millis() - lastWarn > 2000) {
      lastWarn = millis();
      DBG_VLN("I2S read fail: err=" + String((int)err) + " bytes=" + String(bytesRead));
    }
#endif
    return;
  }

  int samplesRead = bytesRead / sizeof(int32_t);
  int16_t peak = 0;
  int32_t sumAbs = 0;
  for (int i = 0; i < samplesRead; i++) {
    int16_t val = (int16_t)(i2sBuffer[i] >> 16);
    int16_t av = val < 0 ? -val : val;
    sumAbs += av;
    if (av > peak) peak = av;
    if (sampleIndex < FFT_SIZE) sampleBuffer[sampleIndex++] = val;
  }
  int16_t avgAbs = samplesRead ? (int16_t)(sumAbs / samplesRead) : 0;

  static int16_t autoMax = 200;
  if (peak > autoMax) autoMax = peak;
  else autoMax -= (autoMax >> 7);
  if (autoMax < 100)  autoMax = 100;
  if (autoMax > 30000) autoMax = 30000;

  int32_t norm = (int32_t)peak * 255 / autoMax;
  if (norm > 255) norm = 255;

  int32_t gainPct = 100 + (soundSensitivity - 50) * 3;
  if (gainPct < 10)  gainPct = 10;
  if (gainPct > 250) gainPct = 250;
  norm = norm * gainPct / 100;
  if (norm > 255) norm = 255;

  uint8_t target = (uint8_t)((norm * 2 + norm * norm / 255) / 3);

  if (target > soundLevel) soundLevel = (uint8_t)((soundLevel + target) >> 1);
  else                     soundLevel = (uint8_t)((soundLevel * 15 + target) >> 4);

  soundBeatAvg = (uint8_t)((soundBeatAvg * 31 + target) >> 5);
  if (soundBeatCooldown > 0) soundBeatCooldown--;
  else if (target > soundBeatAvg + 50 && target > 100) {
    soundBeatFlash = 255;
    soundBeatCooldown = 10;
  }
  if (soundBeatFlash > 0)
    soundBeatFlash = (soundBeatFlash > 20) ? (soundBeatFlash - 20) : 0;

#ifdef DEBUG_VERBOSE
  static unsigned long lastSoundDbg = 0;
  if (millis() - lastSoundDbg > 500) {
    lastSoundDbg = millis();
    DBG_VF("[SND] err=%d bytes=%d peak=%d avg=%d autoMax=%d lvl=%d beat=%d\n",
           (int)err, (int)bytesRead, (int)peak, (int)avgAbs, (int)autoMax,
           (int)soundLevel, (int)soundBeatFlash);
  }
#endif
}

void updateSpectrum() {
  if (!soundEnabled) { memset(spectrum, 0, sizeof(spectrum)); memset(spectrumPeak, 0, sizeof(spectrumPeak)); return; }
  if (sampleIndex < FFT_SIZE) return;
  unsigned long now = millis();
  if (now - lastFftTime < FFT_UPDATE_MS) return;
  lastFftTime = now;

  for (int i = 0; i < FFT_SIZE; i++) {
    fftReal[i] = (double)sampleBuffer[i];
    fftImag[i] = 0.0;
  }
  sampleIndex = 0;

  soundFFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  soundFFT.Compute(FFT_FORWARD);
  soundFFT.ComplexToMagnitude();

  int sensPct = soundSensitivity;
  for (int b = 0; b < SPECTRUM_BANDS; b++) {
    int startBin = 1 + (b * 30) / SPECTRUM_BANDS;
    int endBin   = 1 + ((b + 1) * 30) / SPECTRUM_BANDS;
    if (endBin <= startBin) endBin = startBin + 1;

    double sum = 0; int cnt = 0;
    for (int i = startBin; i < endBin && i < FFT_SIZE / 2; i++) { sum += fftReal[i]; cnt++; }
    double avg = cnt ? (sum / cnt) : 0.0;

    double db = 20.0 * log10(avg + 1.0);
    int32_t val = (int32_t)(db * 255.0 / 45.0);
    val = val * (50 + sensPct) / 100;
    if (val < 0) val = 0; if (val > 255) val = 255;

    if (val > spectrum[b]) spectrum[b] = (uint8_t)((spectrum[b] + val) >> 1);
    else                   spectrum[b] = (uint8_t)((spectrum[b] * 3 + val) >> 2);

    if (spectrum[b] > spectrumPeak[b]) spectrumPeak[b] = spectrum[b];
    else if (spectrumPeak[b] > 0) spectrumPeak[b] = (spectrumPeak[b] > 2) ? (spectrumPeak[b] - 2) : 0;
  }

#ifdef DEBUG_VERBOSE
  static unsigned long lastFftDbg = 0;
  if (millis() - lastFftDbg > 500) {
    lastFftDbg = millis();
    DBG_V("FFT:");
    for (int b = 0; b < SPECTRUM_BANDS; b++) DBG_VF(" %d", (int)spectrum[b]);
    DBG_VLN("");
  }
#endif
}

// --- Реализации эффектов ---
static void effSolid() { FastLED.setBrightness(brightness); fill_solid(leds, LED_COUNT, solidColor); }
static void effRainbow() {
  FastLED.setBrightness(brightness);
  uint8_t d = map(effectScale, 1, 100, 1, 20);
  for (int i = 0; i < LED_COUNT; i++) leds[getLedIndex(i)] = CHSV(rainbowStep + i * d, 255, 255);
  rainbowStep++;
}
static void effConfetti() {
  FastLED.setBrightness(brightness);
  fadeToBlackBy(leds, LED_COUNT, map(effectScale, 1, 100, 5, 40));
  int n = map(effectScale, 1, 100, 1, 5);
  for (int i = 0; i < n; i++) leds[getLedIndex(random(LED_COUNT))] += CHSV(random8(), 255, 255);
}
static void effFire() {
  FastLED.setBrightness(brightness);
  int cooling = map(effectScale, 1, 100, 60, 12);
  int sparking = map(effectScale, 1, 100, 40, 160);
  for (int y = 0; y < ZIGZAG_WIDTH; y++)
    for (int x = 0; x < ZIGZAG_WIDTH; x++)
      heat[y][x] = qsub8(heat[y][x], random8(0, cooling + 2));
  for (int y = ZIGZAG_WIDTH - 1; y > 0; y--)
    for (int x = 0; x < ZIGZAG_WIDTH; x++) {
      int a = heat[y-1][x];
      int l = (x > 0) ? heat[y-1][x-1] : a;
      int r = (x < ZIGZAG_WIDTH - 1) ? heat[y-1][x+1] : a;
      heat[y][x] = (a * 4 + l + r) / 6;
    }
  if (random8() < sparking) { int x = random8(ZIGZAG_WIDTH); heat[0][x] = qadd8(heat[0][x], random8(160, 255)); }
  if (random8() < sparking / 3) {
    int x = random8(ZIGZAG_WIDTH - 1);
    heat[0][x] = qadd8(heat[0][x], random8(120, 220));
    heat[0][x+1] = qadd8(heat[0][x+1], random8(120, 220));
  }
  for (int y = 0; y < ZIGZAG_WIDTH; y++)
    for (int x = 0; x < ZIGZAG_WIDTH; x++)
      leds[getLedIndex(y * ZIGZAG_WIDTH + x)] = HeatColor(heat[y][x]);
}
static void effFireplace() {
  FastLED.setBrightness(brightness);
  int scale = map(effectScale, 1, 100, 30, 200);
  int speedFactor = map(effectSpeed, 1, 100, 1, 10);
  int sparking = map(effectScale, 1, 100, 40, 160);
  fireplacePhase += speedFactor;
  for (int i = 0; i < LED_COUNT; i++) {
    uint8_t noise = (i * 13 + fireplacePhase * 3) & 0xFF;
    uint8_t flick = (sin8(fireplacePhase + i * 7) + sin8(fireplacePhase * 2 + i * 3)) / 2;
    uint8_t height = map(scale, 30, 200, 30, 200);
    int physRow = getLedIndex(i) / ZIGZAG_WIDTH;
    uint8_t base = 255 - (uint8_t)(((float)physRow / (ZIGZAG_WIDTH - 1)) * 255);
    uint8_t hv = qadd8(base, (noise + flick) / 4);
    hv = map(hv, 0, 255, 0, height);
    if (random8() < sparking / 2) hv = qadd8(hv, random8(80, 200));
    fireplaceHeat[i] = hv;
  }
  for (int i = 0; i < LED_COUNT; i++) {
    uint8_t t = fireplaceHeat[i];
    CRGB c = (t < 64)  ? CRGB(t * 4, 0, 0)
           : (t < 128) ? CRGB(255, (t - 64) * 4, 0)
           : (t < 192) ? CRGB(255, 255, (t - 128) * 4)
                       : CRGB(255, 255, 255);
    leds[getLedIndex(i)] = c;
  }
}
static void effCampfire() {
  FastLED.setBrightness(brightness);
  int scale = map(effectScale, 1, 100, 50, 255);
  int speedFactor = map(effectSpeed, 1, 100, 2, 20);
  int sparking = map(effectScale, 1, 100, 60, 200);
  campfirePhase += speedFactor;
  for (int i = 0; i < LED_COUNT; i++) {
    uint8_t n1 = (i * 7 + campfirePhase * 5) & 0xFF;
    uint8_t n2 = (i * 13 + campfirePhase * 3 + 50) & 0xFF;
    uint8_t fl = (sin8(campfirePhase + i * 11) + sin8(campfirePhase * 3 + i * 7) + sin8(campfirePhase * 5 + i * 3)) / 3;
    uint8_t height = map(scale, 50, 255, 40, 200);
    int physRow = getLedIndex(i) / ZIGZAG_WIDTH;
    uint8_t base = 255 - (uint8_t)(((float)physRow / (ZIGZAG_WIDTH - 1)) * 255);
    uint8_t hv = qadd8(base, (n1 + n2 + fl) / 3);
    hv = map(hv, 0, 255, 0, height);
    if (random8() < sparking / 1.5) { hv = qadd8(hv, random8(120, 255)); if (random8() < 30) hv = 255; }
    if (random8() < 10) hv = random8(0, 20);
    campfireHeat[i] = hv;
  }
  for (int i = 0; i < LED_COUNT; i++) {
    uint8_t t = campfireHeat[i];
    CRGB c = (t < 30)  ? CRGB(0, 0, 0)
           : (t < 80)  ? CRGB(t * 3, t / 2, 0)
           : (t < 140) ? CRGB(255, (t - 80) * 3, 0)
           : (t < 200) ? CRGB(255, 255, (t - 140) * 2)
                       : CRGB(255, 255, 255);
    leds[getLedIndex(i)] = c;
  }
}
static void effRunningLights() {
  FastLED.setBrightness(brightness);
  fadeToBlackBy(leds, LED_COUNT, map(effectScale, 1, 100, 10, 255));
  for (int i = 0; i < 3; i++) {
    int pos = (runningPos + i * (LED_COUNT / 3)) % LED_COUNT;
    leds[getLedIndexRotatedMinus90(pos)] = CHSV(hue + i * 40, 255, 255);
  }
  runningPos = (runningPos + 1) % LED_COUNT;
  hue++;
}
static void effMatrix() {
  FastLED.setBrightness(brightness);
  int maxLen = map(effectScale, 1, 100, 1, 10);
  fadeToBlackBy(leds, LED_COUNT, map(effectScale, 1, 100, 200, 30));
  if (random(100) < (10 + effectSpeed / 5)) matrixDrops[random(LED_COUNT)] = random(1, maxLen);
  for (int i = LED_COUNT - 1; i >= 0; i--) {
    if (matrixDrops[i] > 0) {
      leds[getLedIndexRotatedMinus90(i)] = CRGB(0, 255, 0);
      for (int j = 1; j <= matrixDrops[i]; j++)
        if (i - j >= 0) leds[getLedIndexRotatedMinus90(i - j)] = CRGB(0, 255 / (j + 1), 0);
      matrixDrops[i]--;
      if (i + 1 < LED_COUNT && matrixDrops[i] == 0) matrixDrops[i + 1] = 1;
    }
  }
}
static void effSunrise() {
  unsigned long elapsed = millis() - sunriseTimer;
  unsigned long cycle = 20000UL * 100 / effectSpeed;
  if (elapsed > cycle) { sunriseTimer = millis(); elapsed = 0; sunrisePhase = 0; }
  sunrisePhase = map(elapsed, 0, cycle, 0, 255);
  FastLED.setBrightness(map(sunrisePhase, 0, 255, 0, brightness));
  CRGB c = (sunrisePhase < 85)  ? CRGB(255, map(sunrisePhase, 0, 85, 0, 165), 0)
         : (sunrisePhase < 170) ? CRGB(255, 255, map(sunrisePhase, 85, 170, 0, 255))
                                : CRGB::White;
  fill_solid(leds, LED_COUNT, c);
}
static void effPlasma() {
  FastLED.setBrightness(brightness);
  plasmaPhase += 2;
  for (int y = 0; y < ZIGZAG_WIDTH; y++)
    for (int x = 0; x < ZIGZAG_WIDTH; x++) {
      uint8_t v = sin8(x * 16 + plasmaPhase) + sin8(y * 16 + plasmaPhase * 2) + sin8((x + y) * 12 + plasmaPhase * 3);
      leds[getLedIndex(y * ZIGZAG_WIDTH + x)] = CHSV(v, 255, 255);
    }
}
static void effGradient() {
  FastLED.setBrightness(brightness);
  gradientPhase += 2;
  uint8_t span = map(effectScale, 1, 100, 48, 200);
  for (int i = 0; i < LED_COUNT; i++) {
    uint8_t v = sin8(gradientPhase + i * 8);
    leds[getLedIndex(i)] = CHSV((uint16_t)v * span / 255, 255, 255);
  }
}
static void effBreathing() {
  breathingPhase += 2;
  uint8_t b = sin8(breathingPhase);
  FastLED.setBrightness((uint16_t)brightness * b / 255);
  fill_solid(leds, LED_COUNT, solidColor);
}
static void effStrobe() {
  strobePhase += map(effectSpeed, 1, 100, 30, 120);
  if (strobePhase < 128) { FastLED.setBrightness(brightness); fill_solid(leds, LED_COUNT, solidColor); }
  else { FastLED.setBrightness(0); FastLED.clear(); }
}
static void effPacifica() {
  FastLED.setBrightness(brightness);
  pacificaTime += 2;
  for (int y = 0; y < ZIGZAG_WIDTH; y++)
    for (int x = 0; x < ZIGZAG_WIDTH; x++) {
      uint8_t v = sin8(x * 16 + pacificaTime) + sin8(y * 12 + pacificaTime * 2);
      leds[getLedIndex(y * ZIGZAG_WIDTH + x)] = CHSV(160 + v / 4, 255, 200 + v / 8);
    }
}
static void effSparkle() {
  FastLED.setBrightness(brightness);
  fadeToBlackBy(leds, LED_COUNT, map(effectScale, 1, 100, 20, 80));
  int n = map(effectScale, 1, 100, 1, 8);
  for (int i = 0; i < n; i++) if (random8() < 30) leds[getLedIndex(random(LED_COUNT))] = solidColor;
}
static void effSnow() {
  FastLED.setBrightness(brightness);
  fadeToBlackBy(leds, LED_COUNT, 64);
  uint8_t step = map(effectSpeed, 1, 100, 1, 3);
  for (int i = 0; i < SNOW_COUNT; i++) {
    snowY[i] += snowSpeed[i] * step;
    if (snowY[i] >= ZIGZAG_WIDTH) { snowY[i] = 0; snowX[i] = random8(ZIGZAG_WIDTH); }
    int8_t wob = (int8_t)(sin8(snowPhase[i]) / 32) - 4;
    snowPhase[i] += 4;
    int x = constrain((int)snowX[i] + wob, 0, ZIGZAG_WIDTH - 1);
    leds[getLedIndex(snowY[i] * ZIGZAG_WIDTH + x)] = CRGB(200, 200, 255);
  }
}
static void effTwinkle() {
  FastLED.setBrightness(brightness);
  uint8_t step = map(effectSpeed, 1, 100, 1, 3);
  for (int i = 0; i < LED_COUNT; i++) {
    twinklePhase[i] += twinkleSpeed[i] * step;
    leds[getLedIndex(i)] = CHSV(hue + i, 120, sin8(twinklePhase[i]));
  }
  hue += map(effectScale, 1, 100, 1, 4);
}
static void effAurora() {
  FastLED.setBrightness(brightness);
  auroraTime += 2;
  for (int y = 0; y < ZIGZAG_WIDTH; y++)
    for (int x = 0; x < ZIGZAG_WIDTH; x++) {
      uint8_t v = sin8(x * 12 + auroraTime) + sin8(y * 20 + auroraTime * 2) + sin8((x + y) * 8 + auroraTime * 3);
      uint8_t h = map(v, 0, 255, 96, 200);
      leds[getLedIndex(y * ZIGZAG_WIDTH + x)] = CHSV(h, 200, 128 + v / 3);
    }
}
static void effFireworks() {
  FastLED.setBrightness(brightness);
  fadeToBlackBy(leds, LED_COUNT, 40);
  uint16_t interval = map(effectSpeed, 1, 100, 60, 12);
  if (++fireworksTimer >= interval) {
    fireworksTimer = 0;
    int cx = random8(4, 12), cy = random8(4, 12);
    int count = map(effectScale, 1, 100, 8, 22);
    for (int i = 0; i < FIREWORKS_MAX && count > 0; i++) {
      if (fireworks[i].life == 0) {
        uint8_t a = random8();
        int8_t dx = ((int16_t)cos8(a) - 128) >> 4;
        int8_t dy = ((int16_t)sin8(a) - 128) >> 4;
        if (dx == 0 && dy == 0) dx = 1;
        fireworks[i] = { (int8_t)cx, (int8_t)cy, dx, dy, (uint8_t)random8(12, 26) };
        count--;
      }
    }
  }
  for (int i = 0; i < FIREWORKS_MAX; i++) {
    if (fireworks[i].life == 0) continue;
    fireworks[i].x += fireworks[i].dx;
    fireworks[i].y += fireworks[i].dy;
    fireworks[i].dy += 1;
    fireworks[i].life--;
    if (fireworks[i].x < 0 || fireworks[i].x >= ZIGZAG_WIDTH ||
        fireworks[i].y < 0 || fireworks[i].y >= ZIGZAG_WIDTH) { fireworks[i].life = 0; continue; }
    uint8_t v = fireworks[i].life * 10;
    leds[getLedIndex(fireworks[i].y * ZIGZAG_WIDTH + fireworks[i].x)] =
      CHSV((fireworks[i].life * 12) & 0xFF, 255, v);
  }
}
static void effLightning() {
  if (lightningFlash > 0) {
    uint8_t b = lightningFlash;
    lightningFlash = qsub8(lightningFlash, 40);
    FastLED.setBrightness((uint16_t)brightness * b / 255);
    fill_solid(leds, LED_COUNT, CRGB(180, 200, 255));
  } else {
    FastLED.setBrightness(brightness);
    fadeToBlackBy(leds, LED_COUNT, 50);
    if (random8(100) < map(effectSpeed, 1, 100, 1, 12)) lightningFlash = 255;
  }
}
static void effBouncingBall() {
  FastLED.setBrightness(brightness);
  fadeToBlackBy(leds, LED_COUNT, map(effectScale, 1, 100, 90, 30));
  uint8_t step = map(effectSpeed, 1, 100, 1, 3);
  for (uint8_t s = 0; s < step; s++) {
    ballX += ballDX; ballY += ballDY;
    if (ballX <= 0 || ballX >= ZIGZAG_WIDTH - 1) { ballDX = -ballDX; ballX += ballDX * 2; }
    if (ballY <= 0 || ballY >= ZIGZAG_WIDTH - 1) { ballDY = -ballDY; ballY += ballDY * 2; }
  }
  ballHue += 2;
  leds[getLedIndex(ballY * ZIGZAG_WIDTH + ballX)] = CHSV(ballHue, 255, 255);
}
static void effRadialPulse() {
  FastLED.setBrightness(brightness);
  radialPhase += 4;
  for (int y = 0; y < ZIGZAG_WIDTH; y++)
    for (int x = 0; x < ZIGZAG_WIDTH; x++) {
      int dx = x - 7, dy = y - 7;
      uint8_t dist = (uint8_t)(sqrtf((float)(dx * dx + dy * dy)) * 16.0f);
      uint8_t v = sin8(dist * 2 - radialPhase);
      leds[getLedIndex(y * ZIGZAG_WIDTH + x)] = CHSV(v / 2 + 160, 255, v);
    }
}
static void effSinelon() {
  FastLED.setBrightness(brightness);
  fadeToBlackBy(leds, LED_COUNT, map(effectScale, 1, 100, 30, 100));
  uint8_t step = map(effectSpeed, 1, 100, 1, 4);
  sinelonPos += sinelonDir * step;
  if (sinelonPos >= LED_COUNT - 1) { sinelonPos = LED_COUNT - 1; sinelonDir = -1; }
  if (sinelonPos <= 0) { sinelonPos = 0; sinelonDir = 1; }
  leds[getLedIndex(sinelonPos)] = solidColor;
}
static void effHeartbeat() {
  heartbeatPhase += 2;
  if (heartbeatPhase > 255) heartbeatPhase = 0;
  uint8_t p = (uint8_t)heartbeatPhase;
  uint8_t b1 = (p < 30) ? sin8(p * 8) : 0;
  uint8_t b2 = (p >= 45 && p < 75) ? sin8((p - 45) * 8) * 2 / 3 : 0;
  FastLED.setBrightness((uint16_t)brightness * max(b1, b2) / 255);
  fill_solid(leds, LED_COUNT, CRGB::Red);
}
static void effLife() {
  FastLED.setBrightness(brightness);
  uint16_t interval = map(effectSpeed, 1, 100, 40, 6);
  if (++lifeGenTimer >= interval) {
    lifeGenTimer = 0;
    for (int y = 0; y < ZIGZAG_WIDTH; y++)
      for (int x = 0; x < ZIGZAG_WIDTH; x++) {
        int n = 0;
        for (int dy = -1; dy <= 1; dy++)
          for (int dx = -1; dx <= 1; dx++) {
            if (!dx && !dy) continue;
            n += lifeGrid[(y + dy + ZIGZAG_WIDTH) % ZIGZAG_WIDTH][(x + dx + ZIGZAG_WIDTH) % ZIGZAG_WIDTH];
          }
        lifeNext[y][x] = lifeGrid[y][x] ? ((n == 2 || n == 3) ? 1 : 0) : ((n == 3) ? 1 : 0);
      }
    int alive = 0;
    for (int y = 0; y < ZIGZAG_WIDTH; y++) for (int x = 0; x < ZIGZAG_WIDTH; x++) alive += lifeNext[y][x];
    if (alive < 4)
      for (int y = 0; y < ZIGZAG_WIDTH; y++) for (int x = 0; x < ZIGZAG_WIDTH; x++) lifeNext[y][x] = (random8() < 20);
    memcpy(lifeGrid, lifeNext, sizeof(lifeGrid));
    lifeStep++;
  }
  uint8_t cellHue = (lifeStep * 3) & 0xFF;
  for (int y = 0; y < ZIGZAG_WIDTH; y++)
    for (int x = 0; x < ZIGZAG_WIDTH; x++) {
      int idx = getLedIndex(y * ZIGZAG_WIDTH + x);
      leds[idx] = lifeGrid[y][x] ? CRGB(CHSV(cellHue + (x + y) * 4, 200, 255)) : CRGB::Black;
    }
}
static void effSpectrum() {
  FastLED.setBrightness(brightness);
  FastLED.clear();
  int heightScale = 50 + effectScale;
  int peakDecay = map(effectSpeed, 1, 100, 1, 10);
  for (int b = 0; b < SPECTRUM_BANDS; b++) {
    int h = (int)spectrum[b] * ZIGZAG_WIDTH * heightScale / (255 * 100);
    if (h > ZIGZAG_WIDTH) h = ZIGZAG_WIDTH;
    int peakH = (int)spectrumPeak[b] * ZIGZAG_WIDTH / 255;
    if (peakH > ZIGZAG_WIDTH) peakH = ZIGZAG_WIDTH;
    uint8_t bandHue = b * 255 / SPECTRUM_BANDS;
    for (int y = 0; y < h; y++) {
      int row = y;
      uint8_t v = 200 + (y * 55 / ZIGZAG_WIDTH);
      leds[getLedIndex(row * ZIGZAG_WIDTH + b)] = CHSV(bandHue, 255, v);
    }
    if (peakH > h && peakH <= ZIGZAG_WIDTH) {
      leds[getLedIndex((peakH - 1) * ZIGZAG_WIDTH + b)] = CRGB::White;
    }
    if (spectrumPeak[b] > 0 && random8(peakDecay) == 0) spectrumPeak[b]--;
  }
}

typedef void (*EffectFn)();
static const EffectFn EFFECT_FNS[EFFECT_COUNT] = {
  effSolid, effRainbow, effConfetti, effFire, effFireplace,
  effCampfire, effRunningLights, effMatrix, effSunrise,
  effPlasma, effGradient, effBreathing, effStrobe, effPacifica,
  effSparkle, effSnow, effTwinkle, effAurora, effFireworks,
  effLightning, effBouncingBall, effRadialPulse, effSinelon,
  effHeartbeat, effLife, effSpectrum
};

void applyEffect() {
  if (buttonPressed && buttonMode != MODE_NORMAL) {
    FastLED.setBrightness(brightness);
    if (buttonMode == MODE_BRIGHTNESS) showValueScale(brightness, MIN_BRIGHTNESS, maxBrightness, CRGB(0, 255, 255));
    else if (buttonMode == MODE_SPEED) showValueScale(effectSpeed, 1, 100, CRGB(0, 255, 0));
    else showValueScale(effectScale, 1, 100, CRGB(160, 0, 255));
    return;
  }
  if (!lampOn) { FastLED.setBrightness(0); FastLED.clear(); return; }

  uint8_t savedSpeed = effectSpeed;
  uint8_t savedScale = effectScale;

  if (soundEnabled && soundLevel > 0 && currentEffect != "spectrum") {
    if (soundMode == 1) {
      int boost = (int)effectSpeed * soundLevel / 150;
      effectSpeed = constrain((int)effectSpeed + boost, 1, 100);
    } else if (soundMode == 2) {
      int boost = (int)effectScale * soundLevel / 150;
      effectScale = constrain((int)effectScale + boost, 1, 100);
    }
  }

  int idx = getEffectIndex(currentEffect);
  if (idx >= 0) EFFECT_FNS[idx]();

  effectSpeed = savedSpeed;
  effectScale = savedScale;

  if (soundEnabled && soundMode == 0 && currentEffect != "spectrum") {
    uint8_t baseB = FastLED.getBrightness();
    uint32_t boosted = (uint32_t)baseB * (100 + (uint32_t)soundLevel * 200 / 255) / 100;
    if (soundBeatFlash > 0) boosted += (uint32_t)baseB * soundBeatFlash / 170;
    if (boosted > maxBrightness) boosted = maxBrightness;
    if (boosted > 255) boosted = 255;
    FastLED.setBrightness((uint8_t)boosted);
  }
}

// ============ Будильник ============
void checkAlarm() {
  time_t now = time(nullptr);
  if (now < 100000) {
    static unsigned long last = 0;
    if (WiFi.status() == WL_CONNECTED && (!last || millis() - last > 60000)) { last = millis(); syncTime(); }
    return;
  }
  time_t localNow = now + timezoneOffsetMinutes * 60;
  struct tm tm; localtime_r(&localNow, &tm);
  int day = tm.tm_wday, mins = tm.tm_hour * 60 + tm.tm_min;
  if (day != lastDay) { lastDay = day; alarmTriggeredToday = false; }
  if (!alarmEnabled[day]) return;
  int onMin = alarmOnMinutes[day], offMin = alarmOffMinutes[day];
  if (onMin >= 0 && !alarmTriggeredToday && mins == onMin) {
    addLog("Alarm triggered: sunrise");
    lampOn = true; currentEffect = "sunrise";
    int idx = getEffectIndex(currentEffect);
    if (idx >= 0) loadEffectSettings(idx);
    resetEffectState(currentEffect);
    alarmTriggeredToday = true;
    needPublishState = true;
  }
  if (offMin >= 0 && alarmTriggeredToday && mins == offMin) {
    addLog("Alarm off");
    lampOn = false; alarmTriggeredToday = false;
    needPublishState = true;
  }
}