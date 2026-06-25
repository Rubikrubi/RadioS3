/*
 * ============================================================================
 *  RadioS3 v2.0  autorskie radio internetowe na ESP32-S3-DEV-KIT-N16R8-M
 * ============================================================================
 *  Single-file firmware. Architektura: nieblokujaca petla kooperatywna,
 *  dedykowane struktury stanu, jeden helper-scheduler `due()` dla zadan
 *  cyklicznych. 
 *
 *  Funkcje (parytet):
 *   - Stacje MP3/AAC/FLAC/OGG/M3U8, do 50 pozycji, edycja przez WWW.
 *   - Enkoder obrotowy: glosnosc / wybor stacji / zegar / pogoda.
 *   - Long-press (2 s) -> menu startowe (Volume/Stacja/Zegar/Pogoda/Restart).
 *   - WWW: lista klikalna, dodaj/usun/przelacz, /status (JSON), /restart.
 *   - WiFi event-driven + auto-reconnect, NTP (CET/CEST), ArduinoOTA.
 *   - Pogoda IMGW (Radom), lekki skan strumienia (bez duzego bufora).
 *   - Straznik pamieci, health-check streamu, autosave konfiguracji.
 *   - Animowany ekran startowy "Fale sygnalu".
 *
 *  v2.0 (refaktoryzacja - bez zmiany dzialania):
 *   - Stale nazwane zamiast magicznych liczb (CFG_MAGIC, EQ_MIN/MAX, OLED_W,
 *     progi NTP/backoffu); helpery: drawCenteredAt, webRedirectHome, wrapIndex,
 *     localIpStr, restoreFromBak (mniej duplikacji).
 *   - drawStation: bezpieczne kopiowanie (safeCopy zamiast strcpy/strcat).
 *   - initTime nie blokuje juz loop() (koniec ~3 s czekania na NTP przy starcie);
 *     czas dociaga SNTP w tle + sysHealth co 5 min dopoki nieustawiony.
 *
 *  v1.1 (niezawodnosc):
 *   - Boot od razu w trybie pracy (menu tylko z long-press) - komunikaty
 *     startowe (WiFi, IP, "Odtwarzam") sa teraz widoczne.
 *   - Ekran odswieza sie po wygasnieciu komunikatu (koniec "zamrozonych" napisow).
 *   - Stream nigdy sie nie poddaje: po MAX_RETRY dalsze proby co 60 s.
 *   - EOF/pad streamu tuz po starcie -> rosnacy backoff (2s..60s) zamiast
 *     mlotkowania serwera co 1 s; licznik zerowany po 30 s stabilnego grania.
 *   - OTA: stop audio na starcie, postep na OLED, restart przy bledzie.
 *   - NTP: ponowna synchronizacja co 5 min dopoki czas nieustawiony.
 *   - Pogoda: odswiezanie bez migotania ("Pobieram..." tylko gdy brak danych),
 *     brak nowych pobran podczas OTA/restartu.
 *
 *  Storage: tekstowy plik /radio.cfg (linia naglowka + "nazwa<TAB>url"/wiersz),
 *  odporny na zmiany struktur - w przeciwienstwie do binarnego zrzutu struct.
 *
 *  Stack bibliotek (zweryfikowane):
 *   - ESP32-audioI2S 3.4.5  (UWAGA: wymaga PSRAM; nowy callback msg_t)
 *   - U8g2 2.36.19
 *   - RotaryEncoder 1.6.0
 *
 *  USTAWIENIA PLYTKI (Arduino IDE / arduino-cli):
 *   - Board: ESP32S3 Dev Module
 *   - Flash Size: 16MB (128Mb)
 *   - Partition Scheme: "8M with spiffs (3MB APP/1.5MB SPIFFS)" (default_8MB)
 *   - PSRAM: OPI PSRAM            <-- WYMAGANE przez ESP32-audioI2S 3.4.5
 *   FQBN: esp32:esp32:esp32s3:PartitionScheme=default_8MB,PSRAM=opi,FlashSize=16M
 * ============================================================================
 */

#include <Arduino.h>
#include <Audio.h>
#include <ArduinoOTA.h>
#include <RotaryEncoder.h>
#include <SPI.h>
#include <LittleFS.h>
#include <U8g2lib.h>
#include <WebServer.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <esp_wifi.h>
#include <time.h>
#include <cctype>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <strings.h>
#include <math.h>
#include <climits>
#include <memory>
#include <new>
#include <freertos/queue.h>

// ============================ KONFIGURACJA ==================================
constexpr char WIFI_SSID[]     = "";
constexpr char WIFI_PASSWORD[] = "";
constexpr char NTP_SERVER[]    = "tempus1.gum.gov.pl";
constexpr char TZ_INFO[]       = "CET-1CEST,M3.5.0/02:00:00,M10.5.0/03:00:00";
constexpr time_t TIME_VALID_EPOCH = 1700000000L;  // czas uznany za zsynchronizowany powyzej tej epoki
constexpr char OTA_PASSWORD[]  = "";
constexpr char DEVICE_NAME[]   = "RadioS3";
constexpr char FW_VERSION[]    = "2.0";

// Piny (jak w sprawdzonym ukladzie na tej plytce)
constexpr uint8_t OLED_MOSI = 39, OLED_CLK = 38, OLED_DC = 40, OLED_CS = 42, OLED_RST = 41;
constexpr int     OLED_W    = 128;   // szerokosc panelu OLED w px (wysokosc 64 nie jest uzywana liczbowo)
constexpr uint8_t I2S_BCLK = 12, I2S_LRC = 14, I2S_DOUT = 13;
constexpr uint8_t PIN_ENC_A = 5, PIN_ENC_B = 6, PIN_ENC_SW = 4;

// Limity stacji
constexpr size_t MAX_STATIONS = 50;
constexpr size_t ST_NAME_MAX  = 50;
constexpr size_t ST_URL_MAX   = 200;

// Czasy (ms)
constexpr unsigned long DEBOUNCE_MS        = 50;
constexpr unsigned long LONG_PRESS_MS      = 2000;
constexpr unsigned long POST_LONG_IGNORE   = 700;
constexpr unsigned long DRAW_MS            = 100;
constexpr unsigned long FLAC_DRAW_MS       = 500;
constexpr unsigned long MENU_ANIM_MS       = 250;
constexpr unsigned long RESTART_MSG_MS     = 3000;
constexpr unsigned long REBOOT_DISPLAY_MS  = 3000;
constexpr unsigned long STREAM_CHECK_MS    = 60000;
constexpr unsigned long STREAM_STALL_MS    = 120000;
constexpr unsigned long SYS_CHECK_MS       = 30000;
constexpr unsigned long AUTOSAVE_MS        = 300000;
constexpr unsigned long AUTOSAVE_RETRY_MS  = 30000;
constexpr unsigned long WEATHER_REFRESH_MS = 120000;
constexpr unsigned long WEATHER_RETRY_MS   = 60000;
constexpr unsigned long WEATHER_FORCE_MS   = 30000;
constexpr unsigned long WEATHER_HTTP_MS    = 5000;
constexpr unsigned long NTP_RESYNC_MS      = 300000;   // ponow synchronizacje NTP co 5 min dopoki brak czasu
constexpr uint32_t      WEATHER_TASK_STACK = 10240;
constexpr uint32_t      ENCODER_TASK_STACK = 2048;
constexpr UBaseType_t   BUTTON_QUEUE_LEN   = 12;

// WiFi
constexpr int           WIFI_MAX_RECONNECT = 10;
constexpr unsigned long WIFI_RECONNECT_MS  = 10000;
constexpr unsigned long WIFI_TIMEOUT_MS    = 30000;
constexpr unsigned long WIFI_PROGRESS_MS   = 500;

// Streaming
constexpr uint8_t       MAX_RETRY          = 5;
constexpr unsigned long RETRY_BASE_MS      = 2000UL;
constexpr unsigned long RETRY_MAX_MS       = 60000UL;
constexpr unsigned long EARLY_FAIL_MS        = 15000UL;  // okno "stream padl tuz po starcie" -> backoff
constexpr unsigned long STREAM_STABLE_MS     = 30000UL;  // po tylu ms stabilnego grania zeruj licznik retry
constexpr unsigned long EOF_RESTART_BASE_MS  = 1000UL;   // baza opoznienia wznowienia po EOF
constexpr unsigned long NET_WEB_MS         = 25;
constexpr unsigned long NET_OTA_MS         = 50;
constexpr unsigned long FLAC_NET_WEB_MS    = 150;
constexpr unsigned long FLAC_NET_OTA_MS    = 250;

// Pamiec
constexpr size_t HEAP_MIN      = 20000;
constexpr size_t HEAP_CRITICAL = 10000;
constexpr size_t FS_MIN_FREE   = 1024;    // min wolnego miejsca w LittleFS dla zapisu

// Audio
constexpr int VOL_MIN       = 0;
constexpr int VOL_MAX       = 100;
constexpr int AUDIO_MAX_VOL = 21;   // zakres biblioteki
constexpr int EQ_MIN        = -12;  // zakres korektora (srednie/wysokie)
constexpr int EQ_MAX        =  12;
constexpr uint16_t AUDIO_HTTP_TIMEOUT_MS       = 1500;
constexpr uint16_t AUDIO_HTTPS_TIMEOUT_MS      = 5000;
constexpr uint16_t FLAC_AUDIO_HTTP_TIMEOUT_MS  = 2500;
constexpr uint16_t FLAC_AUDIO_HTTPS_TIMEOUT_MS = 7000;

// Pogoda IMGW (Radom): https://meteo.imgw.pl/pogoda/?lat=51.400059&lon=21.158253
constexpr char WEATHER_URL[] =
  "https://meteo.imgw.pl/api/v1/forecast/fcapi"
  "?token=p4DXKjsYadfBV21TYrDk&lat=51.400059&lon=21.158253&m=hybrid";

constexpr char CFG_MAGIC[] = "RADIO1";        // naglowek/wersja pliku konfiguracji
constexpr char CFG_PATH[] = "/radio.cfg";
constexpr char CFG_TMP_PATH[] = "/radio.tmp";
constexpr char CFG_BAK_PATH[] = "/radio.bak";

// ============================ DOMYSLNE STACJE ===============================
struct DefStation { const char* name; const char* url; };
const DefStation DEFAULT_STATIONS[] = {
  {"RMF FM",          "http://rmfstream1.interia.pl:8000/rmf_fm"},
  {"Radio ZET",       "http://n-4-1.dcs.redcdn.pl/sc/o2/Eurozet/live/audio.livx?audio=5"},
  {"Polskie Radio 1", "http://mp3.polskieradio.pl:8900/"},
  {"Radio Pogoda",    "http://stream10.radioagora.pl:80/tuba144-1.mp3"},
  {"Smoothjazz FLAC", "http://bcast.vigormultimedia.com:8888/sjcomplflac"},
  {"Radio Radom",     "https://stream6.nadaje.com:15293/radioradompl"},
  {"Radio Maryja",    "https://radiomaryja.fastcast4u.com/proxy/radiomaryja?mp=/1"},
  {"Radio Wawa",      "https://pl-play.adtonos.com/wawa/wawa.m3u8"},
  {"Antyradio",       "https://an01-cdn.eska.pl/radio/antyradio/live/antyradio.stream/playlist.m3u8"}
};

// ============================ TYPY I STAN ===================================
enum class Mode  : uint8_t { VOLUME, STATION, CLOCK, WEATHER, COUNT };
enum class Input : uint8_t { NORMAL, MENU };
enum class State : uint8_t { BOOT, WIFI, READY, PLAY, ERR, REBOOT };
enum class MenuAct : uint8_t { ENTER, REBOOT };
enum class BtnEventType : uint8_t { PRESS, RELEASE, LONG_PRESS };

struct Station { char name[ST_NAME_MAX + 1]; char url[ST_URL_MAX + 1]; };
struct BtnEvent { BtnEventType type; unsigned long at; };

struct MenuItem { const char* label; MenuAct act; Mode mode; };
const MenuItem MENU_ITEMS[] = {
  {"Volume",  MenuAct::ENTER,  Mode::VOLUME},
  {"Stacja",  MenuAct::ENTER,  Mode::STATION},
  {"Zegar",   MenuAct::ENTER,  Mode::CLOCK},
  {"Pogoda",  MenuAct::ENTER,  Mode::WEATHER},
  {"Restart", MenuAct::REBOOT, Mode::VOLUME}
};
constexpr uint8_t MENU_COUNT = sizeof(MENU_ITEMS) / sizeof(MENU_ITEMS[0]);

struct Sys {
  State         state        = State::BOOT;
  int           current      = 0;    // stacja wskazana przez uzytkownika
  int           playing      = -1;   // stacja faktycznie odtwarzana
  int           connectTarget = -1;  // cel biezacego laczenia/retry
  int           restartTarget = -1;  // cel restartu streamu
  int           volume       = 12;
  int           eqMid        = 0;
  int           eqHigh       = 0;
  uint8_t       retry        = 0;
  unsigned long retryAt      = 0;
  unsigned long connectAt    = 0;
  unsigned long restartAt    = 0;
  unsigned long rebootAt     = 0;
  bool          restartPending = false;
  unsigned long restartMsgUntil = 0;
  bool          configDirty  = false;
  unsigned long lastSave     = 0;
  unsigned long lastSysCheck = 0;
  bool          eofFlag      = false;
  int           lastEncPos   = 0;
};

struct Ui {
  Mode          mode        = Mode::VOLUME;
  Input         input       = Input::NORMAL;
  bool          menuActive  = false;
  bool          menuIntro   = false;
  uint8_t       menuSel     = 0;
  unsigned long menuIdle    = 0;
  volatile bool dirty       = true;
  unsigned long lastDraw    = 0;
  unsigned long msgUntil    = 0;
  unsigned long lastInteract = 0;
  char          msg1[24]    = "";
  char          msg2[24]    = "";
};

struct Net {
  bool          up           = false;
  bool          connecting   = false;
  int           reconnect    = 0;
  unsigned long connectStart = 0;
  unsigned long lastProgress = 0;
  unsigned long lastReconnect = 0;
  int8_t        rssi         = 0;
};

struct Stats {
  uint32_t      connects   = 0;
  uint32_t      errors     = 0;
  uint64_t      uptime     = 0;
  unsigned long sessionStart = 0;
  unsigned long lastData   = 0;
  unsigned long lastCheck  = 0;
  unsigned long lastHealth = 0;
  uint32_t      lastAudioTime = 0;
  uint32_t      lastBufferFill = 0;
  bool          healthy    = true;
};

struct Weather {
  bool          has      = false;
  bool          loading  = false;
  int           tenths   = 0;
  unsigned long lastFetch = 0;
  unsigned long lastAttempt = 0;
  char          err[24]  = "";
  volatile bool taskDone = false;
  volatile bool taskOk = false;
  volatile int  taskTenths = 0;
};

struct Btn {
  volatile bool level   = HIGH;
  volatile bool changed = false;
  unsigned long lastChange = 0;
  unsigned long pressTime  = 0;
  unsigned long ignoreUntil = 0;
  bool          pressed  = false;
  bool          ignoreTillRelease = false;
  bool          longDone = false;
};

// ============================ GLOBALE =======================================
U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI display(U8G2_R0, OLED_CS, OLED_DC, OLED_RST);
Audio          audio;
WebServer      server(80);
RotaryEncoder  encoder(PIN_ENC_A, PIN_ENC_B, RotaryEncoder::LatchMode::FOUR3);

Station  stations[MAX_STATIONS];
size_t   stationCount = 0;

Sys      sys;
Ui       ui;
Net      net;
Stats    stats;
Weather  weather;
Btn      btn;
// Model watkow (kazdy mux chroni male, scisle okreslone pole stanu):
//   - loop()        : rdzen 1, glowna petla kooperatywna (UI, siec, storage, health)
//   - encoderTask   : rdzen 1, czyta enkoder/przycisk -> buttonQueue (lub ISR onButton -> buttonMux)
//   - weatherTask   : rdzen 1, jednorazowy fetch HTTPS -> pola weather.task* pod weatherMux
//   - audio task    : rdzen 0 (audioI2S), callback onAudioInfo -> sys.eofFlag pod eventMux
//   - onWifiEvent   : watek zdarzen WiFi -> wifiGotIp/wifiLost pod eventMux
// weatherMux: weather.task* (Done/Ok/Tenths). eventMux: sys.eofFlag, wifiGotIp, wifiLost.
// buttonMux: btn.level/btn.changed (tylko sciezka ISR, gdy brak encoderTask/kolejki).
portMUX_TYPE weatherMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE eventMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE buttonMux = portMUX_INITIALIZER_UNLOCKED;
QueueHandle_t buttonQueue = nullptr;

bool          timeReady = false;
bool          otaReady  = false;
bool          encoderTaskReady = false;
bool          wifiGotIp = false;
bool          wifiLost  = false;

// ============================ PROTOTYPY ======================================
void  serviceAudio();
void  handleNet(unsigned long now);
void  handleWifi();
void  initWifi();
void  startWifi();
void  initTime();
void  initOta();
void  applyVolume();
void  encoderTask(void* parameter);
void  handleEncoder();
void  handleButton(unsigned long now);
void  handleShortPress(unsigned long now);
void  openMenu(unsigned long now);
void  keepMenuIdle(unsigned long now);
void  activateMenu(unsigned long now);
void  enterMode(Mode m, unsigned long now);
bool  menuOpen();
void  startPlay(bool manual);
void  startPlayIndex(int index, bool manual);
void  doConnect();
void  handleHealth();
void  freezeUptime();
void  drawDisplay();
void  drawBoot();
void  drawMenu();
void  drawCentered(const char* text);
void  drawCenteredAt(const char* text, int y);
void  drawClock();
void  drawWeather();
void  drawVolume();
void  drawStation();
void  drawSysInfo();
void  updateWeather(unsigned long now);
void  weatherTask(void* parameter);
bool  fetchTemp(int& outTenths);
bool  parseTemp(Stream& s, int& outTenths, unsigned long timeoutMs);
void  fmtTemp(int tenths, char* out, size_t len);
void  notify(const char* l1, const char* l2, unsigned long ms);
void  loadDefaults();
bool  loadStore();
bool  restoreFromBak();
bool  saveStore();
void  autoSave(unsigned long now);
void  sysHealth();
void  scheduleReboot();
void  doReboot();
void  fmtUptime(unsigned long ms, char* out, size_t len);
bool  reached(unsigned long now, unsigned long deadline);
bool  due(unsigned long& last, unsigned long interval, unsigned long now);
unsigned long deadlineAfter(unsigned long now, unsigned long delayMs);
bool  streaming();
bool  validIndex();
bool  validPlayingIndex();
bool  validStationIndex(int index);
bool  currentIsFlac();
bool  stationIsFlac(int index);
bool  containsCI(const char* text, const char* needle);
bool  parseId(const String& a, int& out);
String escHtml(const String& in);
String escJson(const String& in);
void  safeCopy(char* dst, const char* src, size_t len);
void  localIpStr(char* out, size_t len);
int   encDelta();
int   wrapIndex(int x, int n);
unsigned long backoff(uint8_t r);
bool  heapOk();
bool  validStation(const char* name, const char* url);
void  initWeb();
void  webRedirectHome();
void  webRoot();
void  webAdd();
void  webRemove();
void  webStatus();
void  webPlay();
void  onButton();
void  onWifiEvent(arduino_event_id_t event, arduino_event_info_t info);
void  onAudioInfo(Audio::msg_t m);

// ============================ SETUP =========================================
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.printf("\n=== RadioS3 v%s (ESP32-S3 N16R8) ===\n", FW_VERSION);
  Serial.printf("Kompilacja: %s %s\n", __DATE__, __TIME__);
  if (!psramFound()) {
    Serial.println("UWAGA: brak PSRAM! ESP32-audioI2S go wymaga. Wlacz OPI PSRAM.");
  }

  if (!LittleFS.begin(true)) {
    Serial.println("BLAD: montaz LittleFS nieudany!");
    sys.state = State::ERR;
    delay(3000);
    ESP.restart();
  }
  Serial.printf("LittleFS: %lu/%lu B\n",
                (unsigned long)LittleFS.usedBytes(), (unsigned long)LittleFS.totalBytes());

  SPI.begin(OLED_CLK, -1, OLED_MOSI, OLED_CS);
  display.begin();
  display.setContrast(0);
  drawBoot();

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_SW, INPUT_PULLUP);
  btn.level = digitalRead(PIN_ENC_SW);
  btn.lastChange = millis();
  encoder.setPosition(0);
  sys.lastEncPos = 0;
  buttonQueue = xQueueCreate(BUTTON_QUEUE_LEN, sizeof(BtnEvent));
  encoderTaskReady = (xTaskCreatePinnedToCore(
    encoderTask, "encoder", ENCODER_TASK_STACK, nullptr, 2, nullptr, 1) == pdPASS);
  if (!encoderTaskReady) Serial.println("UWAGA: encoder task nie wystartowal, uzywam loop().");
  if (!encoderTaskReady && buttonQueue) {
    vQueueDelete(buttonQueue);
    buttonQueue = nullptr;
  }
  if (!buttonQueue) Serial.println("UWAGA: kolejka przycisku niedostepna, uzywam przerwania.");

  Audio::audio_info_callback = onAudioInfo;   // NOWE API v3.4.x
  audio.setAudioTaskCore(0);
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setConnectionTimeout(AUDIO_HTTP_TIMEOUT_MS, AUDIO_HTTPS_TIMEOUT_MS);
  audio.setVolumeSteps(AUDIO_MAX_VOL);
  applyVolume();
  audio.setTone(6.0f, (float)sys.eqMid, (float)sys.eqHigh);

  if (!loadStore()) {
    Serial.println("Wczytuje domyslne stacje...");
    loadDefaults();
    if (!saveStore()) Serial.println("BLAD: zapis domyslnych stacji nieudany.");
  }
  Serial.printf("Stacji: %lu\n", (unsigned long)stationCount);

  sys.state = State::WIFI;
  initWifi();
  initWeb();

  if (!encoderTaskReady || !buttonQueue) {
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_SW), onButton, CHANGE);
  }

  if (net.up) sys.state = State::READY;

  unsigned long now = millis();
  ui.lastInteract = now;
  ui.input        = Input::NORMAL;  // boot w trybie pracy; menu tylko z long-press
  ui.menuActive   = false;
  ui.menuIntro    = false;
  ui.menuIdle     = 0;
  ui.dirty        = true;

  if (stationCount > 0 && net.up) startPlay(false);

  Serial.println("System gotowy.");
}

// ============================ LOOP ==========================================
void loop() {
  unsigned long now = millis();

  serviceAudio();
  handleNet(now);
  handleWifi();
  handleEncoder();
  handleButton(now);
  keepMenuIdle(now);

  if (sys.rebootAt != 0 && reached(now, sys.rebootAt)) doReboot();
  if (sys.connectAt != 0 && reached(now, sys.connectAt)) { sys.connectAt = 0; doConnect(); }
  if (sys.restartAt != 0 && reached(now, sys.restartAt)) {
    sys.restartAt = 0;
    startPlayIndex(sys.restartTarget, false);
  }
  bool eof;
  portENTER_CRITICAL(&eventMux);
  eof = sys.eofFlag;
  sys.eofFlag = false;
  portEXIT_CRITICAL(&eventMux);
  if (eof) {
    if (sys.state == State::PLAY && sys.restartAt == 0) {
      sys.restartTarget = validPlayingIndex() ? sys.playing : sys.connectTarget;
      // Stream padajacy tuz po starcie -> rosnacy backoff zamiast mlotkowania
      // serwera co sekunde. Licznik zeruje handleHealth po 30 s stabilnego grania.
      unsigned long sessionMs = now - stats.sessionStart;
      unsigned long delayMs = EOF_RESTART_BASE_MS;
      if (sessionMs < EARLY_FAIL_MS) {
        if (sys.retry < 255) sys.retry++;
        delayMs = backoff(sys.retry);
      }
      sys.restartAt = deadlineAfter(now, delayMs);
    }
  }

  updateWeather(now);

  // Po wygasnieciu komunikatu wymus odswiezenie - inaczej napis zostaje
  // "zamrozony" na ekranie do najblizszej interakcji.
  if (ui.msgUntil != 0 && reached(now, ui.msgUntil)) {
    ui.msgUntil = 0;
    ui.dirty = true;
  }

  bool clockMode = (ui.mode == Mode::CLOCK);
  bool menuAnim  = menuOpen() && (now - ui.lastDraw > MENU_ANIM_MS);
  if (ui.dirty || (clockMode && now - ui.lastDraw > 1000) || menuAnim) {
    bool flac = streaming() && currentIsFlac();
    unsigned long iv = flac ? FLAC_DRAW_MS : DRAW_MS;
    if (now - ui.lastDraw > iv) {
      drawDisplay();
      serviceAudio();
    }
  }

  handleHealth();
  autoSave(now);

  if (due(sys.lastSysCheck, SYS_CHECK_MS, now)) sysHealth();

  if (sys.retryAt != 0 && reached(now, sys.retryAt)) {
    if (!net.up) {
      sys.retryAt = deadlineAfter(now, WIFI_RECONNECT_MS); // nie spinuj petli bez sieci
    } else {
      // Radio nigdy sie nie poddaje: po MAX_RETRY dalsze proby co RETRY_MAX_MS,
      // wiec stream wraca sam gdy serwer ozyje.
      if (sys.retry == MAX_RETRY + 1) notify("Blad streamu", "ponawiam co 60s", 5000);
      sys.retryAt = 0;
      startPlayIndex(sys.connectTarget, false);
    }
  }

  serviceAudio();
}

// ============================ AUDIO SERWIS ==================================
void serviceAudio() {
  audio.loop();
  if (!streaming()) return;
  if (currentIsFlac()) audio.loop(); // FLAC szybciej oproznia bufor wejsciowy
  vTaskDelay(1);                     // oddaj czas WiFi/lwIP - inaczej glodowanie bufora (FLAC)
}

bool streaming() {
  return sys.state == State::PLAY && audio.isRunning();
}

bool validIndex() {
  return validStationIndex(sys.current);
}

bool validPlayingIndex() {
  return validStationIndex(sys.playing);
}

bool validStationIndex(int index) {
  return stationCount > 0 && index >= 0 && index < (int)stationCount;
}

bool stationIsFlac(int index) {
  if (!validStationIndex(index)) return false;
  const Station& s = stations[index];
  return containsCI(s.url, "flac") || containsCI(s.name, "flac") || containsCI(s.name, "smoothjazz");
}

bool currentIsFlac() {
  if (streaming() && validPlayingIndex()) return stationIsFlac(sys.playing);
  if (validStationIndex(sys.connectTarget)) return stationIsFlac(sys.connectTarget);
  return stationIsFlac(sys.current);
}

// ============================ HELPERY CZASU =================================
// Odporne na rollover millis() (po ~49.7 dnia) dzieki arytmetyce unsigned.
bool reached(unsigned long now, unsigned long deadline) {
  return (int32_t)(now - deadline) >= 0;
}

// Jednolity wzorzec zadan cyklicznych. Zwraca true gdy minal interwal i
// przesuwa znacznik. Rollover-safe.
bool due(unsigned long& last, unsigned long interval, unsigned long now) {
  if (now - last >= interval) { last = now; return true; }
  return false;
}

unsigned long deadlineAfter(unsigned long now, unsigned long delayMs) {
  unsigned long deadline = now + delayMs;
  return deadline == 0 ? 1 : deadline;
}

// ============================ GLOSNOSC ======================================
void applyVolume() {
  if (sys.volume <= 0) { audio.setVolume(0); return; }
  float r = (float)sys.volume / (float)VOL_MAX;
  int   t = (int)(r * r * AUDIO_MAX_VOL);   // krzywa perceptualna (kwadratowa)
  if (t < 1) t = 1;
  audio.setVolume((uint8_t)t);
}

// ============================ ENKODER =======================================
void encoderTask(void* parameter) {
  (void)parameter;
  bool rawButton = digitalRead(PIN_ENC_SW);
  bool stableButton = rawButton;
  bool longSent = false;
  unsigned long rawChangedAt = millis();
  unsigned long pressedAt = rawChangedAt;

  if (stableButton == LOW && buttonQueue) {
    BtnEvent event{BtnEventType::PRESS, pressedAt};
    xQueueSend(buttonQueue, &event, 0);
  }

  for (;;) {
    encoder.tick();
    unsigned long now = millis();
    bool level = digitalRead(PIN_ENC_SW);

    if (level != rawButton) {
      rawButton = level;
      rawChangedAt = now;
    }

    if (rawButton != stableButton && now - rawChangedAt >= DEBOUNCE_MS) {
      stableButton = rawButton;
      BtnEvent event{
        stableButton == LOW ? BtnEventType::PRESS : BtnEventType::RELEASE,
        now
      };
      if (buttonQueue) xQueueSend(buttonQueue, &event, 0);
      if (stableButton == LOW) {
        pressedAt = now;
        longSent = false;
      }
    }

    if (stableButton == LOW && !longSent && now - pressedAt >= LONG_PRESS_MS) {
      BtnEvent event{BtnEventType::LONG_PRESS, now};
      if (buttonQueue) xQueueSend(buttonQueue, &event, 0);
      longSent = true;
    }

    vTaskDelay(1);
  }
}

int encDelta() {
  int pos = encoder.getPosition();
  int d = pos - sys.lastEncPos;
  sys.lastEncPos = pos;
  return d;
}

void handleEncoder() {
  if (!encoderTaskReady) encoder.tick();
  int d = encDelta();
  if (d == 0) return;
  ui.lastInteract = millis();

  if (menuOpen()) {
    int step = (d > 0) ? 1 : -1;
    if (ui.menuIntro) {
      ui.menuIntro = false;
      ui.menuSel = (step > 0) ? 0 : (MENU_COUNT - 1);
    } else {
      ui.menuSel = (uint8_t)wrapIndex(ui.menuSel + step, MENU_COUNT);
    }
    ui.menuIdle = ui.lastInteract;
    ui.dirty = true;
    return;
  }

  switch (ui.mode) {
    case Mode::VOLUME:
      sys.volume = constrain(sys.volume + d, VOL_MIN, VOL_MAX);
      applyVolume();
      sys.configDirty = true;
      break;
    case Mode::STATION:
      if (stationCount > 0) {
        int n = (int)stationCount;
        sys.current = wrapIndex(sys.current + d, n);
        sys.configDirty = true;
      }
      break;
    default:
      break;
  }
  ui.dirty = true;
}

// ============================ PRZYCISK ======================================
// Krotkie nacisniecie: nawigacja/wybor. Dlugie (2 s): otwarcie menu startowego.
void handleShortPress(unsigned long now) {
  ui.lastInteract = now;
  if (menuOpen()) {
    activateMenu(now);
  } else if (ui.mode == Mode::STATION && validIndex()) {
    startPlay(true);
  } else {
    int next = (static_cast<int>(ui.mode) + 1) % static_cast<int>(Mode::COUNT);
    enterMode(static_cast<Mode>(next), now);
  }
  ui.dirty = true;
}

void handleButton(unsigned long now) {
  if (encoderTaskReady && buttonQueue) {
    BtnEvent event;
    while (xQueueReceive(buttonQueue, &event, 0) == pdTRUE) {
      if (btn.ignoreUntil != 0 && reached(event.at, btn.ignoreUntil)) {
        btn.ignoreUntil = 0;
      }

      switch (event.type) {
        case BtnEventType::PRESS:
          if (btn.ignoreUntil != 0) {
            btn.ignoreTillRelease = true;
            break;
          }
          btn.pressed = true;
          btn.pressTime = event.at;
          btn.longDone = false;
          ui.lastInteract = now;
          break;

        case BtnEventType::LONG_PRESS:
          if (!btn.pressed || btn.longDone || btn.ignoreTillRelease) break;
          btn.longDone = true;
          btn.ignoreTillRelease = true;
          btn.ignoreUntil = deadlineAfter(event.at, POST_LONG_IGNORE);
          openMenu(now);
          break;

        case BtnEventType::RELEASE:
          if (btn.ignoreTillRelease) {
            btn.ignoreTillRelease = false;
            btn.pressed = false;
            break;
          }
          if (!btn.pressed) break;
          btn.pressed = false;
          if (!btn.longDone) handleShortPress(now);
          break;
      }
    }
    return;
  }

  bool chg, level;
  portENTER_CRITICAL(&buttonMux);
  chg = btn.changed;
  if (chg) btn.changed = false;
  level = btn.level;
  portEXIT_CRITICAL(&buttonMux);

  if (chg) btn.lastChange = now;
  if (now - btn.lastChange < DEBOUNCE_MS) return;

  bool down = (level == LOW);

  if (btn.ignoreTillRelease) {
    if (!down) { btn.ignoreTillRelease = false; btn.pressed = false; }
    return;
  }
  if (btn.ignoreUntil != 0) {
    if (reached(now, btn.ignoreUntil)) btn.ignoreUntil = 0;
    else { if (!down) btn.pressed = false; return; }
  }

  if (down && !btn.pressed) {            // nacisniecie
    btn.pressed = true;
    btn.pressTime = now;
    btn.longDone = false;
    ui.lastInteract = now;
  }

  if (btn.pressed && !btn.longDone && now - btn.pressTime >= LONG_PRESS_MS) {  // long-press
    btn.longDone = true;
    btn.ignoreTillRelease = true;
    btn.ignoreUntil = deadlineAfter(now, POST_LONG_IGNORE);
    openMenu(now);
  }

  if (!down && btn.pressed) {             // zwolnienie
    btn.pressed = false;
    if (!btn.longDone) {
      handleShortPress(now);
    }
  }
}

// ============================ MENU STARTOWE =================================
void openMenu(unsigned long now) {
  sys.lastEncPos = encoder.getPosition();
  ui.input = Input::MENU;
  ui.menuActive = true;
  ui.menuIntro = true;
  ui.menuIdle = now;
  ui.lastInteract = now;
  ui.msgUntil = 0;
  for (uint8_t i = 0; i < MENU_COUNT; i++) {
    if (MENU_ITEMS[i].act == MenuAct::ENTER && MENU_ITEMS[i].mode == ui.mode) {
      ui.menuSel = i;
      break;
    }
  }
  ui.dirty = true;
}

// Menu NIE zamyka sie samo (swiadomy wybor) - tylko utrzymuje znacznik idle.
void keepMenuIdle(unsigned long now) {
  if (!menuOpen()) return;
  if (btn.pressed || (ui.msgUntil != 0 && !reached(now, ui.msgUntil))) {
    ui.menuIdle = now;
    return;
  }
  if (ui.menuIdle == 0) ui.menuIdle = now;
}

void activateMenu(unsigned long now) {
  if (ui.menuSel >= MENU_COUNT) ui.menuSel = 0;
  if (ui.menuIntro) { ui.menuIntro = false; ui.dirty = true; return; }

  const MenuItem& it = MENU_ITEMS[ui.menuSel];
  sys.lastEncPos = encoder.getPosition();
  ui.input = Input::NORMAL;
  ui.menuActive = false;
  ui.menuIntro = false;
  ui.menuIdle = 0;

  if (it.act == MenuAct::REBOOT) {
    scheduleReboot();
    ui.dirty = true;
    return;
  }

  enterMode(it.mode, now);
  if (it.mode == Mode::VOLUME && stationCount > 0 && !audio.isRunning()) {
    startPlay(true);
  }
  ui.dirty = true;
}

void enterMode(Mode m, unsigned long now) {
  Mode prev = ui.mode;
  ui.mode = m;
  if (m == Mode::WEATHER && prev != Mode::WEATHER) {
    if (!weather.has) weather.lastAttempt = 0;                       // wymus pobranie
    else if (now - weather.lastFetch >= WEATHER_FORCE_MS) weather.lastFetch = 0;  // odswiez
  }
}

bool menuOpen() {
  return ui.input == Input::MENU || ui.menuActive;
}

// ============================ RADIO =========================================
void startPlay(bool manual) {
  startPlayIndex(sys.current, manual);
}

void startPlayIndex(int index, bool manual) {
  if (!validStationIndex(index)) {
    sys.connectAt = 0;
    if (manual) notify("Brak stacji", "dodaj przez WWW", 3000);
    return;
  }
  // Zapamietaj wybor takze bez sieci; po powrocie WiFi ruszy wlasciwa stacja.
  bool newRequest = manual || index != sys.connectTarget;
  if (newRequest) sys.retry = 0;
  sys.connectTarget = index;
  sys.connectAt = 0;
  sys.retryAt = 0;
  sys.restartTarget = -1;
  sys.restartAt = 0;
  sys.restartPending = false;
  if (!net.up) {
    if (manual) notify("Brak WiFi", "nie uruchamiam", 2000);
    return;
  }
  audio.stopSong();
  portENTER_CRITICAL(&eventMux);
  sys.eofFlag = false;
  portEXIT_CRITICAL(&eventMux);
  sys.playing = -1;
  sys.connectAt = deadlineAfter(millis(), 100); // odroczone laczenie
  stats.lastCheck = millis();
  stats.lastHealth = millis();          // nie zeruj do 0 - inaczej falszywy "stream padl"
}

void doConnect() {
  int target = sys.connectTarget;
  if (!validStationIndex(target)) {
    sys.state = State::READY;
    sys.connectAt = 0; sys.retryAt = 0;
    return;
  }
  if (!net.up || WiFi.status() != WL_CONNECTED) {
    sys.state = State::WIFI;
    if (sys.retry < 255) sys.retry++;
    sys.retryAt = deadlineAfter(millis(), backoff(sys.retry));
    return;
  }

  bool flacMode = stationIsFlac(target);
  audio.setConnectionTimeout(flacMode ? FLAC_AUDIO_HTTP_TIMEOUT_MS : AUDIO_HTTP_TIMEOUT_MS,
                             flacMode ? FLAC_AUDIO_HTTPS_TIMEOUT_MS : AUDIO_HTTPS_TIMEOUT_MS);
  applyVolume();
  audio.setTone(6.0f, (float)sys.eqMid, (float)sys.eqHigh);

  const char* url  = stations[target].url;
  const char* name = stations[target].name;
  Serial.printf("Laczenie: %s\n", name);
  if (flacMode) Serial.println("Tryb FLAC: rzadsze WWW/OTA/OLED/pogoda.");

  if (audio.connecttohost(url)) {
    sys.playing = target;
    portENTER_CRITICAL(&eventMux);
    sys.eofFlag = false;
    portEXIT_CRITICAL(&eventMux);
    sys.restartAt = 0;
    sys.restartTarget = -1;
    sys.state = State::PLAY;
    stats.connects++;
    stats.sessionStart = millis();
    stats.lastData = millis();
    stats.lastCheck = millis();
    stats.lastAudioTime = audio.getAudioCurrentTime();
    stats.lastBufferFill = audio.inBufferFilled();
    sys.retryAt = 0;   // licznik sys.retry zeruje handleHealth po 30 s stabilnego grania
    Serial.println("Stream OK");
    notify("Odtwarzam", name, 2000);
  } else {
    sys.playing = -1;
    Serial.println("Blad laczenia streamu");
    sys.state = State::READY;
    stats.errors++;
    notify("Blad streamu", "ponawiam...", 2000);
    if (sys.retry < 255) sys.retry++;
    sys.retryAt = deadlineAfter(millis(), backoff(sys.retry));
  }
}

unsigned long backoff(uint8_t r) {
  uint8_t n = constrain(r, 1, 10);
  unsigned long d = RETRY_BASE_MS * (1UL << (n - 1));
  return constrain(d, RETRY_BASE_MS, RETRY_MAX_MS);
}

void handleHealth() {
  unsigned long now = millis();
  bool running = audio.isRunning();

  // Stream gra stabilnie >30 s -> wyzeruj licznik backoffu wznawiania.
  if (sys.state == State::PLAY && running && sys.retry != 0 &&
      now - stats.sessionStart > STREAM_STABLE_MS) {
    sys.retry = 0;
  }

  if (sys.state == State::PLAY && running) {
    uint32_t audioTime = audio.getAudioCurrentTime();
    uint32_t bufferFill = audio.inBufferFilled();
    if (audioTime != stats.lastAudioTime || bufferFill != stats.lastBufferFill) {
      stats.lastData = now;
    }
    stats.lastAudioTime = audioTime;
    stats.lastBufferFill = bufferFill;
  }

  if (sys.restartPending && reached(now, sys.restartMsgUntil)) {
    sys.restartPending = false;
    startPlayIndex(sys.restartTarget, false);
  }

  if (now - stats.lastHealth < STREAM_CHECK_MS) return;
  stats.lastHealth = now;

  bool reconnecting = (sys.connectAt != 0) || (sys.retryAt != 0);
  if (sys.state == State::PLAY && !running && !reconnecting) {
    Serial.println("Stream zatrzymany - restart...");
    notify("Stream", "restart...", 2000);
    sys.restartTarget = validPlayingIndex() ? sys.playing : sys.connectTarget;
    sys.restartPending = true;
    sys.restartMsgUntil = deadlineAfter(now, RESTART_MSG_MS);
  }

  if (sys.state == State::PLAY) {
    stats.healthy = running && (now - stats.lastData < STREAM_STALL_MS);
    if (running && !stats.healthy && !reconnecting && !sys.restartPending) {
      Serial.println("Brak postepu danych streamu - restart...");
      notify("Stream stoi", "restart...", 2000);
      sys.restartTarget = validPlayingIndex() ? sys.playing : sys.connectTarget;
      sys.restartPending = true;
      sys.restartMsgUntil = deadlineAfter(now, RESTART_MSG_MS);
    }
  } else {
    stats.healthy = true;
  }
}

void freezeUptime() {
  if (sys.state == State::PLAY && stats.sessionStart > 0) {
    unsigned long now = millis();
    stats.uptime += now - stats.sessionStart;
    stats.sessionStart = now;
  }
}

// ============================ WIFI ==========================================
void initWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);          // nie zapisuj kazdego reconnectu do NVS
  WiFi.setHostname(DEVICE_NAME);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.onEvent(onWifiEvent);
  esp_wifi_set_ps(WIFI_PS_NONE);
  startWifi();
}

void startWifi() {
  if (WiFi.status() == WL_CONNECTED || net.connecting) return;
  Serial.printf("Laczenie z WiFi: %s\n", WIFI_SSID);
  sys.state = State::WIFI;
  net.up = false;
  net.connecting = true;
  net.connectStart = millis();
  net.lastProgress = 0;
  net.lastReconnect = net.connectStart;
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  notify("WiFi", "laczenie...", 1000);
  ui.dirty = true;
}

void handleWifi() {
  static bool linkUp = false;
  bool gotIp, lost;
  portENTER_CRITICAL(&eventMux);
  gotIp = wifiGotIp; wifiGotIp = false;
  lost  = wifiLost;  wifiLost  = false;
  portEXIT_CRITICAL(&eventMux);

  if (lost) {
    Serial.println("WiFi rozlaczone");
    if (sys.state != State::REBOOT && sys.state != State::ERR) {
      int resumeTarget = validPlayingIndex() ? sys.playing :
                         (validStationIndex(sys.connectTarget) ? sys.connectTarget : sys.current);
      if (audio.isRunning()) audio.stopSong();
      sys.playing = -1;
      sys.connectTarget = validStationIndex(resumeTarget) ? resumeTarget : -1;
      sys.connectAt = 0;
      sys.retryAt = 0;
      sys.restartAt = 0;
      sys.restartTarget = -1;
      sys.restartPending = false;
      sys.state = State::WIFI;
    }
  }
  if (gotIp) {
    net.reconnect = 0;
    net.rssi = WiFi.RSSI();
    sys.retry = 0;
  }

  if (WiFi.status() == WL_CONNECTED) {
    bool justUp = !linkUp || net.connecting;
    linkUp = true;
    net.up = true;
    net.rssi = WiFi.RSSI();
    if (!justUp) return;

    net.connecting = false;
    net.reconnect = 0;
    net.lastReconnect = millis();
    sys.state = audio.isRunning() ? State::PLAY : State::READY;
    Serial.printf("\nWiFi OK: %s\n", WiFi.localIP().toString().c_str());
    notify("WiFi OK", WiFi.localIP().toString().c_str(), 3000);

    if (!timeReady) initTime();
    initOta();
    if (stationCount > 0 && !audio.isRunning()) {
      int target = validStationIndex(sys.connectTarget) ? sys.connectTarget : sys.current;
      startPlayIndex(target, false);
    }
    return;
  }

  linkUp = false;
  net.up = false;
  if (!net.connecting) return;

  unsigned long now = millis();
  if (now - net.lastProgress >= WIFI_PROGRESS_MS) {
    net.lastProgress = now;
    Serial.print(".");
    notify("WiFi", "laczenie...", 1000);
  }
  if (now - net.connectStart >= WIFI_TIMEOUT_MS) {
    net.up = false;
    net.connecting = false;
    WiFi.disconnect(false);
    Serial.println("\nBlad polaczenia WiFi!");
    notify("WiFi blad", "sprawdz haslo", 5000);
    if (++net.reconnect >= WIFI_MAX_RECONNECT) scheduleReboot();
  }
}

// ============================ CZAS / OTA ====================================
void initTime() {
  configTzTime(TZ_INFO, NTP_SERVER);
  timeReady = true;
  Serial.println("Strefa: Polska (CET/CEST).");
  if (WiFi.status() != WL_CONNECTED) return;

  // NIE blokuj loop() (wczesniej petla do 3 s zamrazala audio/WWW/OTA/UI przy
  // kazdym uzyskaniu IP). Jedna szybka proba tylko do logu bootowego; wlasciwa
  // synchronizacja dociaga SNTP w tle, a sysHealth() ponawia configTzTime co
  // NTP_RESYNC_MS dopoki czas nieustawiony; drawClock pokazuje "Oczekuje NTP..."
  // do tego czasu.
  struct tm tinfo;
  if (getLocalTime(&tinfo, 10)) {
    Serial.printf("Czas: %02d:%02d %02d.%02d.%04d\n",
                  tinfo.tm_hour, tinfo.tm_min,
                  tinfo.tm_mday, tinfo.tm_mon + 1, tinfo.tm_year + 1900);
  }
}

void initOta() {
  if (otaReady || WiFi.status() != WL_CONNECTED) return;
  ArduinoOTA.setHostname(DEVICE_NAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  // Flashowanie rownolegle z dekodowaniem audio = niestabilny OTA.
  // Na starcie aktualizacji zatrzymaj stream i zablokuj health-restart.
  ArduinoOTA.onStart([]() {
    audio.stopSong();
    sys.connectAt = 0;
    sys.retryAt = 0;
    sys.restartAt = 0;
    sys.restartPending = false;
    sys.state = State::REBOOT;          // blokuje wznawianie streamu podczas OTA
    notify("OTA", "aktualizacja...", 120000);
    drawDisplay();                       // loop() stoi podczas uploadu - rysuj recznie
    Serial.println("OTA: start, stream zatrzymany.");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static unsigned int lastPct = 101;
    unsigned int pct = total ? (progress * 100U) / total : 0;
    if (pct != lastPct && pct % 5 == 0) {
      lastPct = pct;
      char b[16];
      snprintf(b, sizeof(b), "%u%%", pct);
      notify("OTA", b, 120000);
      drawDisplay();
    }
  });
  ArduinoOTA.onEnd([]() {
    notify("OTA OK", "restart...", 10000);
    drawDisplay();
    Serial.println("OTA: zakonczone.");
  });
  ArduinoOTA.onError([](ota_error_t e) {
    Serial.printf("OTA: blad %u\n", (unsigned)e);
    notify("OTA blad", "restart...", 5000);
    drawDisplay();
    scheduleReboot();                    // czysty stan po nieudanym OTA
  });

  ArduinoOTA.begin();
  otaReady = true;
  Serial.println("ArduinoOTA aktywne.");
}

// ============================ SIEC (web + OTA serwis) =======================
void handleNet(unsigned long now) {
  static unsigned long lastWeb = 0;
  static unsigned long lastOta = 0;
  bool s = streaming();
  bool flac = s && currentIsFlac();
  unsigned long webMs = flac ? FLAC_NET_WEB_MS : NET_WEB_MS;
  unsigned long otaMs = flac ? FLAC_NET_OTA_MS : NET_OTA_MS;
  if (!s || now - lastWeb >= webMs) {
    lastWeb = now;
    server.handleClient();
  }
  if (otaReady && (!s || now - lastOta >= otaMs)) {
    lastOta = now;
    ArduinoOTA.handle();
  }
}

// ============================ POGODA ========================================
void updateWeather(unsigned long now) {
  if (weather.loading) {
    bool done;
    bool ok = false;
    int tenths = 0;
    portENTER_CRITICAL(&weatherMux);
    done = weather.taskDone;
    if (done) {
      ok = weather.taskOk;
      tenths = weather.taskTenths;
      weather.taskDone = false;
    }
    portEXIT_CRITICAL(&weatherMux);

    if (!done) return;
    weather.loading = false;
    if (ok) {
      weather.has = true;
      weather.tenths = tenths;
      weather.lastFetch = now;
      weather.lastAttempt = 0;
      weather.err[0] = '\0';
    } else {
      safeCopy(weather.err, WiFi.status() == WL_CONNECTED ? "Blad IMGW" : "Brak WiFi",
               sizeof(weather.err));
    }
    ui.dirty = true;
    return;
  }

  if (sys.state == State::REBOOT) return;   // nie startuj pobierania podczas OTA/restartu
  if (streaming() && currentIsFlac()) return;
  if (ui.mode != Mode::WEATHER) return;

  bool attemptDue = (weather.lastAttempt == 0 ||
                     now - weather.lastAttempt >= WEATHER_RETRY_MS);
  bool retryDue = !weather.has && attemptDue;
  bool refreshDue = weather.has &&
                    (weather.lastFetch == 0 || now - weather.lastFetch >= WEATHER_REFRESH_MS) &&
                    attemptDue;
  if (!retryDue && !refreshDue) return;

  weather.lastAttempt = now;
  weather.loading = true;
  weather.err[0] = '\0';
  ui.dirty = true;

  portENTER_CRITICAL(&weatherMux);
  weather.taskDone = false;
  weather.taskOk = false;
  weather.taskTenths = 0;
  portEXIT_CRITICAL(&weatherMux);

  BaseType_t started = xTaskCreatePinnedToCore(
    weatherTask, "weather", WEATHER_TASK_STACK, nullptr, 1, nullptr, 1);
  if (started != pdPASS) {
    weather.loading = false;
    safeCopy(weather.err, "Brak RAM", sizeof(weather.err));
    ui.dirty = true;
  }
}

// Uruchamiany na osobnym watku (rdzen 1), jednorazowy. UWAGA: NIE dotykaj tu
// bezposrednio sys/ui/stats/stations - komunikacja tylko przez pola weather.task*
// pod weatherMux; loop()/updateWeather przepisuje wynik do stanu glownego.
void weatherTask(void* parameter) {
  (void)parameter;
  int tenths = 0;
  bool ok = fetchTemp(tenths);

  portENTER_CRITICAL(&weatherMux);
  weather.taskTenths = tenths;
  weather.taskOk = ok;
  weather.taskDone = true;
  portEXIT_CRITICAL(&weatherMux);

  vTaskDelete(nullptr);
}

bool fetchTemp(int& outTenths) {
  if (WiFi.status() != WL_CONNECTED) return false;

  // WiFiClientSecure na stercie (mbedTLS alokuje duze bufory) - nie na stosie loop().
  auto client = std::unique_ptr<WiFiClientSecure>(new (std::nothrow) WiFiClientSecure());
  if (!client) return false;
  client->setInsecure();
  client->setHandshakeTimeout((WEATHER_HTTP_MS + 999) / 1000);

  HTTPClient http;
  http.setConnectTimeout(WEATHER_HTTP_MS);
  http.setTimeout(WEATHER_HTTP_MS);
  if (!http.begin(*client, WEATHER_URL)) return false;

  int code = http.GET();
  if (code != HTTP_CODE_OK) { http.end(); return false; }

  bool ok = parseTemp(*http.getStreamPtr(), outTenths, WEATHER_HTTP_MS);
  http.end();
  return ok;
}

// Lekki skan strumienia: szuka "Temperature":"<kelvin>" bez buforowania calosci.
bool parseTemp(Stream& s, int& outTenths, unsigned long timeoutMs) {
  const char key[] = "\"Temperature\":\"";
  size_t matched = 0;
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    while (s.available()) {
      char c = (char)s.read();
      if (c == key[matched]) {
        matched++;
        if (key[matched] == '\0') {
          char val[12];
          size_t len = 0;
          unsigned long vStart = millis();
          while (millis() - vStart < 1000 && len < sizeof(val) - 1) {
            if (!s.available()) { delay(1); vTaskDelay(1); continue; }
            char v = (char)s.read();
            if (v == '"') {
              val[len] = '\0';
              float kelvin = atof(val);
              if (kelvin < 150.0f || kelvin > 350.0f) return false;
              outTenths = (int)lroundf((kelvin - 273.15f) * 10.0f);
              return true;
            }
            if ((v >= '0' && v <= '9') || v == '-' || v == '.') val[len++] = v;
            else return false;
          }
          return false;
        }
      } else {
        matched = (c == key[0]) ? 1 : 0;
      }
    }
    delay(1);
    vTaskDelay(1);
  }
  return false;
}

void fmtTemp(int tenths, char* out, size_t len) {
  if (!out || len == 0) return;
  int a = abs(tenths);
  snprintf(out, len, "%s%d,%d", tenths < 0 ? "-" : "", a / 10, a % 10);
}

// ============================ WYSWIETLACZ ===================================
void drawDisplay() {
  display.firstPage();
  do {
    unsigned long now = millis();
    bool showMsg = (ui.msgUntil != 0) && !reached(now, ui.msgUntil);
    if (menuOpen()) {
      if (ui.msgUntil != 0) ui.msgUntil = 0;
      drawMenu();
    } else if (showMsg) {
      display.setFont(u8g2_font_luBS10_tr);
      if (ui.msg1[0]) {
        int w = display.getStrWidth(ui.msg1);
        display.drawStr((OLED_W - w) / 2, 25, ui.msg1);
      }
      if (ui.msg2[0]) {
        int w = display.getStrWidth(ui.msg2);
        display.drawStr((OLED_W - w) / 2, 45, ui.msg2);
      }
    } else {
      if (ui.msgUntil != 0) ui.msgUntil = 0;
      switch (ui.mode) {
        case Mode::CLOCK:   drawClock();   break;
        case Mode::WEATHER: drawWeather(); break;
        case Mode::VOLUME:  drawVolume();  break;
        case Mode::STATION: drawStation(); break;
        default:            drawSysInfo(); break;
      }
    }
  } while (display.nextPage());
  ui.dirty = false;
  ui.lastDraw = millis();
}

// ------------------------- EKRAN STARTOWY ----------------------------------
// "Fale sygnalu": koncentryczne pierscienie pulsujace na zewnatrz od srodka
// (efekt nadawania), wordmark RadioS3 w badge, stopka "vX.Y * OTA" i pasek
// postepu. Animacja trwa ok. 2 s i jest wykonywana tylko w setup().
void drawBoot() {
  const int     cx = OLED_W / 2, cy = 27;
  const uint8_t FRAMES = 26;
  const uint8_t FRAME_DELAY_MS = 75;
  const int     ringSpan = 46;

  display.setFont(u8g2_font_fub17_tr);
  const char* word = "RadioS3";
  int wordW = display.getStrWidth(word);
  const uint8_t* wordFont = u8g2_font_fub17_tr;
  if (wordW > 104) {
    wordFont = u8g2_font_luBS14_tr;
    display.setFont(wordFont);
    wordW = display.getStrWidth(word);
  }

  const int boxW = wordW + 16;
  const int boxH = 26;
  const int bx = (OLED_W - boxW) / 2;
  const int by = cy - boxH / 2;
  const int wordBase = cy + 6;

  char ver[10];
  snprintf(ver, sizeof(ver), "v%s", FW_VERSION);
  const char* ota = "OTA";

  for (uint8_t f = 0; f < FRAMES; f++) {
    display.firstPage();
    do {
      for (uint8_t k = 0; k < 3; k++) {
        int r = 6 + ((f * 3 + k * 16) % ringSpan);
        display.drawCircle(cx, cy, r, U8G2_DRAW_ALL);
      }

      display.setDrawColor(0);
      display.drawBox(bx - 2, by - 2, boxW + 4, boxH + 4);
      display.setDrawColor(1);
      display.drawRFrame(bx, by, boxW, boxH, 4);
      display.setFont(wordFont);
      display.drawStr((OLED_W - wordW) / 2, wordBase, word);

      display.setFont(u8g2_font_5x8_tr);
      int wv = display.getStrWidth(ver);
      int wo = display.getStrWidth(ota);
      const int gap = 9;
      int total = wv + gap + wo;
      int fx = (OLED_W - total) / 2;
      int fy = 58;
      display.setDrawColor(0);
      display.drawBox(fx - 3, fy - 8, total + 6, 11);
      display.setDrawColor(1);
      display.drawStr(fx, fy, ver);
      display.drawDisc(fx + wv + gap / 2, fy - 3, 1);
      display.drawStr(fx + wv + gap, fy, ota);

      int barX = 24, barW = 80, barY = 61;
      display.setDrawColor(0);
      display.drawBox(barX - 1, barY - 1, barW + 2, 4);
      display.setDrawColor(1);
      display.drawFrame(barX, barY, barW, 2);
      int fill = (int)((long)barW * (f + 1) / FRAMES);
      if (fill > 0) display.drawBox(barX, barY, fill, 2);
    } while (display.nextPage());
    delay(FRAME_DELAY_MS);
  }
}

// Rysuje tekst wysrodkowany w poziomie (szerokosc OLED 128 px) na wysokosci y.
void drawCenteredAt(const char* text, int y) {
  display.drawStr((OLED_W - display.getStrWidth(text)) / 2, y, text);
}

void drawCentered(const char* text) {
  if (!text) return;
  display.setFont(u8g2_font_logisoso22_tr);
  int w = display.getStrWidth(text);
  int base = 42;
  if (w > 124) { display.setFont(u8g2_font_logisoso18_tr); w = display.getStrWidth(text); base = 40; }
  if (w > 124) { display.setFont(u8g2_font_luBS14_tr);      w = display.getStrWidth(text); base = 39; }
  int x = (OLED_W - w) / 2;
  if (x < 0) x = 0;
  display.drawStr(x, base, text);
}

void drawMenu() {
  const char* label = ui.menuIntro ? "Ustawienia" : MENU_ITEMS[ui.menuSel].label;
  drawCentered(label);
  if (!ui.menuIntro) {
    int total = MENU_COUNT * 6 - 2;
    int x0 = (OLED_W - total) / 2;
    for (uint8_t i = 0; i < MENU_COUNT; i++) {
      int x = x0 + i * 6;
      if (i == ui.menuSel) display.drawBox(x, 56, 4, 4);
      else                 display.drawFrame(x + 1, 57, 2, 2);
    }
  }
}

void drawClock() {
  struct tm t;
  if (getLocalTime(&t, 10)) {
    char s[8];   // HH:MM = 5 znakow + NUL; margines na ew. zmiane formatu
    snprintf(s, sizeof(s), "%02d:%02d", t.tm_hour, t.tm_min);
    display.setFont(u8g2_font_logisoso32_tn);
    int w = display.getStrWidth(s);
    display.drawStr((OLED_W - w) / 2, 50, s);
  } else {
    display.setFont(u8g2_font_luBS10_tr);
    const char* m = "Oczekuje NTP...";
    drawCenteredAt(m, 35);
  }
}

void drawWeather() {
  display.setFont(u8g2_font_luBS10_tr);
  const char* title = "Radom IMGW";
  drawCenteredAt(title, 12);

  // "Pobieram..." tylko gdy nie ma zadnych danych; przy odswiezaniu pokazuj
  // ostatnia temperature - bez migotania ekranu co 2 minuty.
  if (weather.loading && !weather.has) {
    const char* m = "Pobieram...";
    drawCenteredAt(m, 37);
    return;
  }
  if (!weather.has) {
    const char* m = weather.err[0] ? weather.err : "Brak danych";
    drawCenteredAt(m, 37);
    return;
  }

  char ts[16];
  fmtTemp(weather.tenths, ts, sizeof(ts));
  display.setFont(u8g2_font_logisoso32_tn);
  int valW = display.getStrWidth(ts);
  display.setFont(u8g2_font_luBS10_tr);
  int unitW = display.getStrWidth("C");
  int totalW = valW + 12 + unitW;
  int valX = (OLED_W - totalW) / 2;
  if (valX < 0) valX = 0;

  display.setFont(u8g2_font_logisoso32_tn);
  display.drawStr(valX, 51, ts);
  display.setFont(u8g2_font_luBS10_tr);
  int unitX = valX + valW + 3;
  display.drawCircle(unitX + 2, 29, 2);
  display.drawStr(unitX + 8, 38, "C");
}

void drawVolume() {
  display.setFont(u8g2_font_luBS10_tr);
  const char* title = "Volume";
  drawCenteredAt(title, 15);
  display.drawFrame(10, 25, 108, 12);
  int fill = map(sys.volume, VOL_MIN, VOL_MAX, 0, 108);
  if (fill > 0) display.drawBox(10, 25, fill, 12);
  char v[8];
  snprintf(v, sizeof(v), "%d%%", sys.volume);
  drawCenteredAt(v, 55);
}

void drawStation() {
  if (!validIndex()) {
    display.setFont(u8g2_font_luBS10_tr);
    const char* m = "Brak stacji";
    drawCenteredAt(m, 25);
    const char* m2 = "Dodaj przez WWW";
    drawCenteredAt(m2, 45);
    return;
  }
  display.setFont(u8g2_font_luBS10_tr);
  const char* name = stations[sys.current].name;
  char shown[25];
  if (strlen(name) > 20) {
    safeCopy(shown, name, 18);                        // 17 znakow + NUL
    safeCopy(shown + 17, "...", sizeof(shown) - 17);  // dopisz wielokropek -> lacznie 20 znakow
  } else {
    safeCopy(shown, name, sizeof(shown));
  }
  drawCenteredAt(shown, 25);

  char info[16];
  snprintf(info, sizeof(info), "%d/%d", sys.current + 1, (int)stationCount);
  drawCenteredAt(info, 45);

  bool selectedIsPlaying = audio.isRunning() && validPlayingIndex() &&
                           sys.current == sys.playing;
  const char* status = selectedIsPlaying ? "GRAJ" :
                       (audio.isRunning() ? "WYBOR" : "STOP");
  drawCenteredAt(status, 60);
}

void drawSysInfo() {
  display.setFont(u8g2_font_luBS10_tr);
  char ip[20];
  localIpStr(ip, sizeof(ip));
  display.drawStr(5, 15, "IP:");
  display.drawStr(25, 15, ip);

  char buf[32];
  snprintf(buf, sizeof(buf), "RAM: %luKB", (unsigned long)(ESP.getFreeHeap() / 1024));
  display.drawStr(5, 30, buf);
  snprintf(buf, sizeof(buf), "WiFi: %ddBm", WiFi.RSSI());
  display.drawStr(5, 45, buf);

  char up[24];
  fmtUptime(millis(), up, sizeof(up));
  snprintf(buf, sizeof(buf), "Up: %s", up);
  display.drawStr(5, 60, buf);
}

void notify(const char* l1, const char* l2, unsigned long ms) {
  safeCopy(ui.msg1, l1 ? l1 : "", sizeof(ui.msg1));
  safeCopy(ui.msg2, l2 ? l2 : "", sizeof(ui.msg2));
  ui.msgUntil = deadlineAfter(millis(), ms);
  ui.dirty = true;
}

// ============================ STACJE / STORAGE =============================
void loadDefaults() {
  size_t n = sizeof(DEFAULT_STATIONS) / sizeof(DEFAULT_STATIONS[0]);
  if (n > MAX_STATIONS) n = MAX_STATIONS;
  stationCount = n;
  for (size_t i = 0; i < n; i++) {
    safeCopy(stations[i].name, DEFAULT_STATIONS[i].name, sizeof(stations[i].name));
    safeCopy(stations[i].url,  DEFAULT_STATIONS[i].url,  sizeof(stations[i].url));
  }
  sys.current = 0;
}

// Format /radio.cfg (tekstowy, odporny na zmiany struktur):
//   linia 0: "RADIO1"                       (magic+wersja)
//   linia 1: "<volume> <current> <eqMid> <eqHigh> [stationCount]"
//            (stationCount jest opcjonalny dla zgodnosci ze starszym plikiem)
//   linie+ : "<nazwa>\t<url>"               (po jednej stacji)
// Po niekompletnym zapisie (zanik zasilania miedzy rename) odtworz CFG z kopii .bak.
// Zwraca true, gdy CFG_PATH zostal przywrocony i warto ponowic odczyt loadStore().
bool restoreFromBak() {
  if (!LittleFS.exists(CFG_BAK_PATH)) return false;
  LittleFS.remove(CFG_PATH);
  return LittleFS.rename(CFG_BAK_PATH, CFG_PATH);
}

bool loadStore() {
  // Po zaniku zasilania pomiedzy rename() odtworz ostatnia kompletna wersje.
  if (!LittleFS.exists(CFG_PATH)) {
    if (LittleFS.exists(CFG_BAK_PATH)) {
      LittleFS.rename(CFG_BAK_PATH, CFG_PATH);
    } else if (LittleFS.exists(CFG_TMP_PATH)) {
      LittleFS.rename(CFG_TMP_PATH, CFG_PATH);
    }
  }
  if (!LittleFS.exists(CFG_PATH)) return false;
  File f = LittleFS.open(CFG_PATH, "r");
  if (!f) {
    if (restoreFromBak()) return loadStore();
    return false;
  }

  String magic = f.readStringUntil('\n'); magic.trim();
  if (magic != CFG_MAGIC) {
    f.close();
    if (restoreFromBak()) return loadStore();
    return false;
  }

  String cfg = f.readStringUntil('\n');
  int v = 12, c = 0, m = 0, h = 0, expectedCount = -1;
  int fields = sscanf(cfg.c_str(), "%d %d %d %d %d", &v, &c, &m, &h, &expectedCount);
  if (fields < 4 || (fields >= 5 && (expectedCount < 0 || expectedCount > (int)MAX_STATIONS))) {
    f.close();
    if (restoreFromBak()) return loadStore();
    return false;
  }
  sys.volume = v; sys.current = c; sys.eqMid = m; sys.eqHigh = h;

  stationCount = 0;
  while (f.available() && stationCount < MAX_STATIONS) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    int tab = line.indexOf('\t');
    if (tab <= 0) continue;
    String nm = line.substring(0, tab);
    String ur = line.substring(tab + 1);
    nm.trim(); ur.trim();
    if (!validStation(nm.c_str(), ur.c_str())) continue;
    safeCopy(stations[stationCount].name, nm.c_str(), sizeof(stations[stationCount].name));
    safeCopy(stations[stationCount].url,  ur.c_str(), sizeof(stations[stationCount].url));
    stationCount++;
  }
  f.close();

  if (fields >= 5 && stationCount != (size_t)expectedCount) {
    if (restoreFromBak()) return loadStore();
    return false;
  }

  sys.volume = constrain(sys.volume, VOL_MIN, VOL_MAX);
  sys.eqMid = constrain(sys.eqMid, EQ_MIN, EQ_MAX);
  sys.eqHigh = constrain(sys.eqHigh, EQ_MIN, EQ_MAX);
  if (stationCount == 0) sys.current = 0;
  else sys.current = constrain(sys.current, 0, (int)stationCount - 1);
  if (LittleFS.exists(CFG_TMP_PATH)) LittleFS.remove(CFG_TMP_PATH);
  if (LittleFS.exists(CFG_BAK_PATH)) LittleFS.remove(CFG_BAK_PATH);
  return true;
}

bool saveStore() {
  if (LittleFS.exists(CFG_TMP_PATH)) LittleFS.remove(CFG_TMP_PATH);
  File f = LittleFS.open(CFG_TMP_PATH, "w");
  if (!f) return false;
  f.println(CFG_MAGIC);
  f.printf("%d %d %d %d %u\n", sys.volume, sys.current, sys.eqMid, sys.eqHigh,
           (unsigned)stationCount);
  for (size_t i = 0; i < stationCount; i++) {
    f.print(stations[i].name);
    f.print('\t');
    f.println(stations[i].url);
  }
  f.flush();
  bool writeOk = (f.getWriteError() == 0);
  f.close();
  if (!writeOk) {
    LittleFS.remove(CFG_TMP_PATH);
    return false;
  }

  bool hadConfig = LittleFS.exists(CFG_PATH);
  if (hadConfig) {
    if (LittleFS.exists(CFG_BAK_PATH)) LittleFS.remove(CFG_BAK_PATH);
    if (!LittleFS.rename(CFG_PATH, CFG_BAK_PATH)) {
      LittleFS.remove(CFG_TMP_PATH);
      return false;
    }
  }
  if (!LittleFS.rename(CFG_TMP_PATH, CFG_PATH)) {
    if (hadConfig) LittleFS.rename(CFG_BAK_PATH, CFG_PATH);
    LittleFS.remove(CFG_TMP_PATH);
    return false;
  }
  if (LittleFS.exists(CFG_BAK_PATH)) LittleFS.remove(CFG_BAK_PATH);

  sys.configDirty = false;
  return true;
}

void autoSave(unsigned long now) {
  if (sys.configDirty && (now - sys.lastSave > AUTOSAVE_MS)) {
    sys.lastSave = now;
    if (!saveStore()) {
      Serial.println("BLAD: autosave nieudany.");
      sys.lastSave = now - (AUTOSAVE_MS - AUTOSAVE_RETRY_MS);
    }
  }
}

// ============================ HEALTH / RESTART =============================
void sysHealth() {
  size_t freeHeap = ESP.getFreeHeap();
  if (freeHeap < HEAP_CRITICAL) {
    Serial.printf("KRYTYCZNY HEAP: %lu B!\n", (unsigned long)freeHeap);
    notify("Malo RAM!", "restart...", 3000);
    scheduleReboot();
    return;
  }
  if (!net.up && millis() - net.lastReconnect > WIFI_RECONNECT_MS) {
    Serial.println("WiFi health reconnect");
    net.lastReconnect = millis();
    startWifi();
  }
  // NTP nie zsynchronizowal czasu (np. start bez internetu) -> ponow co 5 min.
  // SNTP sam z siebie ponawia dopiero po godzinie.
  static unsigned long lastNtpKick = 0;
  if (timeReady && net.up && time(nullptr) < TIME_VALID_EPOCH &&
      millis() - lastNtpKick > NTP_RESYNC_MS) {
    lastNtpKick = millis();
    configTzTime(TZ_INFO, NTP_SERVER);
    Serial.println("NTP: ponowna proba synchronizacji.");
  }
  freezeUptime();
}

void scheduleReboot() {
  if (sys.rebootAt == 0) {
    sys.state = State::REBOOT;
    notify("RESTART", "SYSTEMU", REBOOT_DISPLAY_MS);
    sys.rebootAt = deadlineAfter(millis(), REBOOT_DISPLAY_MS);
    Serial.println("Zaplanowano restart.");
  }
}

void doReboot() {
  Serial.println("Restart...");
  if (sys.configDirty && !saveStore()) Serial.println("BLAD: zapis przed restartem.");
  audio.stopSong();
  server.stop();
  WiFi.disconnect(true);
  delay(500);
  ESP.restart();
}

void fmtUptime(unsigned long ms, char* out, size_t len) {
  if (!out || len == 0) return;
  unsigned long s = ms / 1000, m = s / 60, h = m / 60, d = h / 24;
  if (d > 0)      snprintf(out, len, "%lud %luh", d, h % 24);
  else if (h > 0) snprintf(out, len, "%luh %lum", h, m % 60);
  else            snprintf(out, len, "%lum %lus", m, s % 60);
}

// ============================ SERWER WWW ====================================
void initWeb() {
  server.on("/",        HTTP_GET,  webRoot);
  server.on("/add",     HTTP_POST, webAdd);
  server.on("/remove",  HTTP_GET,  webRemove);
  server.on("/status",  HTTP_GET,  webStatus);
  server.on("/play",    HTTP_GET,  webPlay);
  server.on("/restart", HTTP_GET, []() {
    scheduleReboot();
    server.send(200, "text/plain", "Restart scheduled");
  });
  server.begin();
  Serial.println("HTTP serwer uruchomiony.");
}

// Przekierowanie 303 na strone glowna (po akcjach zmieniajacych stan).
void webRedirectHome() {
  server.sendHeader("Location", "/");
  server.send(303);
}

void webRoot() {
  String s;
  s.reserve(6144);

  char ip[20];
  localIpStr(ip, sizeof(ip));
  char up[24];
  fmtUptime(millis(), up, sizeof(up));

  s = F("<!doctype html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>RadioS3</title><style>"
        "body{font-family:sans-serif;padding:10px;max-width:520px}"
        "input{width:100%;box-sizing:border-box;margin-bottom:8px;padding:6px}"
        "button,a.btn{display:block;padding:10px;width:100%;background:#1a73e8;"
        "color:#fff;border:none;border-radius:4px;text-decoration:none;"
        "text-align:center;margin-top:6px;cursor:pointer;font-size:1em}"
        "ol{padding-left:20px} li{margin:5px 0}"
        "a.play{text-decoration:none;color:#333}"
        "a.play.active{font-weight:bold;color:#1a73e8}"
        ".del{font-size:0.8em;color:#c00;margin-left:8px}"
        ".info{font-size:0.82em;color:#666;margin-bottom:8px}"
        "</style></head><body>");

  s += "<h2>RadioS3 v"; s += FW_VERSION; s += "</h2>";
  s += "<p class='info'>IP: "; s += ip;
  s += " &nbsp;|&nbsp; RAM: "; s += String(ESP.getFreeHeap() / 1024); s += " KB";
  s += " &nbsp;|&nbsp; "; s += up; s += "</p>";

  if (streaming() && validPlayingIndex()) {
    s += "<p>&#9654;&nbsp;<strong>"; s += escHtml(stations[sys.playing].name); s += "</strong></p>";
  }

  if (stationCount < MAX_STATIONS) {
    s += F("<form action='/add' method='post'>"
           "Nazwa stacji:<br><input name='name' maxlength='50' required><br>"
           "URL streamu:<br><input name='url' maxlength='200' required><br>"
           "<button type='submit'>Dodaj stacje</button></form><hr>");
  } else {
    s += F("<p>Limit 50 stacji osiagniety.</p><hr>");
  }

  s += F("<ol>");
  for (size_t i = 0; i < stationCount; i++) {
    bool active = validPlayingIndex() && i == (size_t)sys.playing && audio.isRunning();
    s += "<li><a href='/play?id="; s += i;
    s += "' class='play"; if (active) s += " active"; s += "'>";
    s += escHtml(stations[i].name);
    if (active) s += " &#9654;";
    s += "</a><a href='/remove?id="; s += i; s += "' class='del'>[usun]</a></li>";
  }
  s += F("</ol><hr><a href='/restart' class='btn' "
         "onclick=\"return confirm('Na pewno restart?')\">Restart systemu</a>"
         "</body></html>");

  server.send(200, "text/html; charset=utf-8", s);
}

void webAdd() {
  if (!server.hasArg("name") || !server.hasArg("url") || stationCount >= MAX_STATIONS) {
    webRedirectHome(); return;
  }
  String name = server.arg("name"); name.trim();
  String url  = server.arg("url");  url.trim();
  if (!validStation(name.c_str(), url.c_str()) || !heapOk()) {
    webRedirectHome(); return;
  }
  int oldCurrent = sys.current;
  bool oldDirty = sys.configDirty;
  size_t idx = stationCount;
  safeCopy(stations[idx].name, name.c_str(), sizeof(stations[idx].name));
  safeCopy(stations[idx].url,  url.c_str(),  sizeof(stations[idx].url));
  sys.current = idx;
  stationCount++;
  sys.configDirty = true;
  if (!saveStore()) {
    stationCount--;
    sys.current = oldCurrent;
    sys.configDirty = oldDirty;
    notify("Blad zapisu", "LittleFS", 3000);
    server.send(500, "text/plain", "Blad zapisu");
    return;
  }
  startPlay(true);
  webRedirectHome();
}

void webRemove() {
  if (!server.hasArg("id")) { webRedirectHome(); return; }
  int id = -1;
  if (!parseId(server.arg("id"), id) || id < 0 || id >= (int)stationCount) {
    webRedirectHome(); return;
  }
  Station removed = stations[id];
  int oldCurrent = sys.current;
  int oldPlaying = sys.playing;
  int oldConnectTarget = sys.connectTarget;
  int oldRestartTarget = sys.restartTarget;
  State oldState = sys.state;
  uint8_t oldRetry = sys.retry;
  unsigned long oldRetryAt = sys.retryAt;
  unsigned long oldConnectAt = sys.connectAt;
  unsigned long oldRestartAt = sys.restartAt;
  bool oldRestartPending = sys.restartPending;
  bool oldDirty = sys.configDirty;

  bool removedPlaying = (id == oldPlaying);
  bool wasPlaying = removedPlaying && audio.isRunning();
  bool resumeAfterRemove = removedPlaying &&
                           (wasPlaying || sys.state == State::PLAY ||
                            sys.restartPending || sys.restartAt != 0);

  for (int i = id; i < (int)stationCount - 1; i++) stations[i] = stations[i + 1];
  stationCount--;

  if (stationCount == 0) {
    sys.current = 0;
    sys.playing = -1;
    sys.connectTarget = -1;
    sys.restartTarget = -1;
    sys.state = State::READY;
    sys.retry = 0;
    sys.retryAt = 0;
    sys.connectAt = 0;
    sys.restartAt = 0;
    sys.restartPending = false;
  } else {
    if (sys.current > id) {
      sys.current--;
    } else if (sys.current == id && sys.current >= (int)stationCount) {
      sys.current = (int)stationCount - 1;
    }

    auto adjustRemovedIndex = [id](int& index) {
      if (index == id) index = -1;
      else if (index > id) index--;
    };
    adjustRemovedIndex(sys.playing);
    adjustRemovedIndex(sys.connectTarget);
    adjustRemovedIndex(sys.restartTarget);

    if (sys.connectTarget < 0) {
      sys.connectAt = 0;
      sys.retryAt = 0;
    }
    if (sys.restartTarget < 0) {
      sys.restartAt = 0;
      sys.restartPending = false;
    }
  }

  sys.configDirty = true;
  if (!saveStore()) {
    for (int i = (int)stationCount; i > id; i--) stations[i] = stations[i - 1];
    stations[id] = removed;
    stationCount++;
    sys.current = oldCurrent;
    sys.playing = oldPlaying;
    sys.connectTarget = oldConnectTarget;
    sys.restartTarget = oldRestartTarget;
    sys.state = oldState;
    sys.retry = oldRetry;
    sys.retryAt = oldRetryAt;
    sys.connectAt = oldConnectAt;
    sys.restartAt = oldRestartAt;
    sys.restartPending = oldRestartPending;
    sys.configDirty = oldDirty;
    notify("Blad zapisu", "LittleFS", 3000);
    server.send(500, "text/plain", "Blad zapisu");
    return;
  }
  if (stationCount == 0 || wasPlaying) audio.stopSong();
  if (resumeAfterRemove && stationCount > 0) startPlay(true);

  webRedirectHome();
}

void webStatus() {
  char ip[20];
  localIpStr(ip, sizeof(ip));

  bool isPlaying = audio.isRunning() && validPlayingIndex();
  String j = "{";
  j.reserve(512);   // ogranicz realokacje przy budowaniu JSON (wynik identyczny)
  j += "\"version\":\""; j += FW_VERSION; j += "\",";
  j += "\"ip\":\""; j += ip; j += "\",";
  j += "\"rssi\":"; j += WiFi.RSSI(); j += ",";
  j += "\"heap\":"; j += ESP.getFreeHeap(); j += ",";
  j += "\"uptime_ms\":"; j += millis(); j += ",";
  j += "\"streaming\":"; j += isPlaying ? "true" : "false"; j += ",";
  j += "\"codec\":\""; j += escJson(String(audio.getCodecname())); j += "\",";
  j += "\"flac_mode\":"; j += (isPlaying && stationIsFlac(sys.playing)) ? "true" : "false"; j += ",";
  j += "\"buf_filled\":"; j += audio.inBufferFilled(); j += ",";
  j += "\"buf_free\":"; j += audio.inBufferFree(); j += ",";
  j += "\"buf_size\":"; j += audio.getInBufferSize(); j += ",";
  j += "\"station_idx\":"; j += (isPlaying ? sys.playing : -1); j += ",";
  j += "\"selected_idx\":"; j += (validIndex() ? sys.current : -1); j += ",";
  j += "\"playing_idx\":"; j += (isPlaying ? sys.playing : -1); j += ",";
  j += "\"station_count\":"; j += stationCount; j += ",";
  j += "\"volume\":"; j += sys.volume;
  if (isPlaying) {
    j += ",\"station_name\":\""; j += escJson(String(stations[sys.playing].name)); j += "\"";
  }
  if (validIndex()) {
    j += ",\"selected_name\":\""; j += escJson(String(stations[sys.current].name)); j += "\"";
  }
  j += "}";
  server.send(200, "application/json", j);
}

void webPlay() {
  if (!server.hasArg("id")) { webRedirectHome(); return; }
  int id = -1;
  if (!parseId(server.arg("id"), id) || id < 0 || id >= (int)stationCount) {
    webRedirectHome(); return;
  }
  sys.current = id;
  sys.configDirty = true;
  startPlay(true);
  webRedirectHome();
}

// ============================ POMOCNICZE ====================================
// Indeks zawiniety do zakresu [0, n) - modulo dajace zawsze wynik nieujemny.
int wrapIndex(int x, int n) {
  return ((x % n) + n) % n;
}

bool containsCI(const char* text, const char* needle) {
  if (!text || !needle || !needle[0]) return false;
  size_t nl = strlen(needle);
  for (const char* p = text; *p; ++p) {
    size_t i = 0;
    while (i < nl && p[i] && tolower((unsigned char)p[i]) == tolower((unsigned char)needle[i])) i++;
    if (i == nl) return true;
  }
  return false;
}

String escHtml(const String& in) {
  String o = in;
  o.replace("&", "&amp;");
  o.replace("\"", "&quot;");
  o.replace("<", "&lt;");
  o.replace(">", "&gt;");
  return o;
}

String escJson(const String& in) {
  String o = in;
  o.replace("\\", "\\\\");
  o.replace("\"", "\\\"");
  o.replace("\n", "\\n");
  o.replace("\r", "\\r");
  o.replace("\t", "\\t");
  return o;
}

bool parseId(const String& a, int& out) {
  if (a.length() == 0) return false;
  for (size_t i = 0; i < (size_t)a.length(); i++)
    if (!isDigit((unsigned char)a[i])) return false;
  long p = a.toInt();
  if (p < 0 || p > INT_MAX) return false;
  out = (int)p;
  return true;
}

// Kopiuje max len-1 znakow, zawsze terminuje. len = pelny rozmiar bufora.
void safeCopy(char* dst, const char* src, size_t len) {
  if (!dst || len == 0) return;
  if (!src) { dst[0] = '\0'; return; }
  size_t n = strnlen(src, len - 1);
  memcpy(dst, src, n);
  dst[n] = '\0';
}

// Kopiuje biezacy adres IP (jako tekst) do bufora out o rozmiarze len, z terminacja.
void localIpStr(char* out, size_t len) {
  if (!out || len == 0) return;
  strncpy(out, WiFi.localIP().toString().c_str(), len - 1);
  out[len - 1] = '\0';
}

bool heapOk() {
  size_t heap = ESP.getFreeHeap();
  size_t total = LittleFS.totalBytes();
  size_t used = LittleFS.usedBytes();
  size_t freeFs = (used < total) ? (total - used) : 0;
  return heap > HEAP_MIN && freeFs > FS_MIN_FREE;
}

bool validStation(const char* name, const char* url) {
  if (!name || !url) return false;
  size_t nl = strlen(name), ul = strlen(url);
  if (nl == 0 || nl > ST_NAME_MAX || ul == 0 || ul > ST_URL_MAX) return false;
  if (strpbrk(name, "\t\r\n") || strpbrk(url, "\t\r\n")) return false;
  for (const unsigned char* p = (const unsigned char*)name; *p; ++p)
    if (*p < 0x20) return false;
  for (const unsigned char* p = (const unsigned char*)url; *p; ++p)
    if (*p < 0x20) return false;
  if (strncmp(url, "http://", 7) != 0 && strncmp(url, "https://", 8) != 0) return false;
  for (size_t i = 0; i < stationCount; i++) {
    if (strcasecmp(stations[i].name, name) == 0 || strcmp(stations[i].url, url) == 0) return false;
  }
  return true;
}

// ============================ PRZERWANIA / CALLBACKI =======================
// Kazda z tych funkcji dziala POZA loop() - w kontekscie przerwania lub innego
// watku. Stad krotkie sekcje krytyczne i tylko ustawianie flag; ciezka praca
// (restart streamu, reconnect) odbywa sie pozniej w loop().

// Kontekst: ISR (przerwanie GPIO). Tylko zapis btn.level/changed pod buttonMux.
void IRAM_ATTR onButton() {
  portENTER_CRITICAL_ISR(&buttonMux);
  btn.level = digitalRead(PIN_ENC_SW);
  btn.changed = true;
  portEXIT_CRITICAL_ISR(&buttonMux);
}

// Kontekst: watek zdarzen WiFi. Tylko flagi wifiGotIp/wifiLost pod eventMux.
void onWifiEvent(arduino_event_id_t event, arduino_event_info_t info) {
  (void)info;
  portENTER_CRITICAL(&eventMux);
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:       wifiGotIp = true; break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: wifiLost  = true; break;
    default: break;
  }
  portEXIT_CRITICAL(&eventMux);
}

// Nowy callback ESP32-audioI2S 3.4.x (stare audio_eof_mp3()/audio_info() martwe).
// Kontekst: watek audio (rdzen 0). Tylko ustawia sys.eofFlag pod eventMux.
void onAudioInfo(Audio::msg_t m) {
  switch (m.e) {
    case Audio::evt_eof:
      portENTER_CRITICAL(&eventMux);
      sys.eofFlag = true;                       // koniec strumienia -> restart w loop()
      portEXIT_CRITICAL(&eventMux);
      break;
    case Audio::evt_name:
      if (m.msg) Serial.printf("Stacja: %s\n", m.msg);
      break;
    case Audio::evt_streamtitle:
      if (m.msg) Serial.printf("Tytul: %s\n", m.msg);
      break;
    case Audio::evt_lasthost:
      if (m.msg) Serial.printf("Host: %s\n", m.msg);
      break;
    default:
      break;
  }
}

// =================================== KONIEC =================================
