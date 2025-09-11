#include <TFT_eSPI.h>          // TFT display library
#include <SPI.h>               // SPI communication
#include <WiFi.h>              // WiFi connectivity
#include <WebServer.h>         // Web server for configuration interface
#include <Preferences.h>       // Non-volatile storage for settings
#include <ESPmDNS.h>           // mDNS for network discovery
#include <Adafruit_ADS1X15.h>  // ADC for voltage measurements
#include <ArduinoJson.h>       // JSON handling for web interface
#include <algorithm>           // Standard algorithms (sorting)
#include <Update.h>            // OTA update functionality

// ========== Configuration ==========
// Define color constants for TFT display
#define TFT_BG TFT_NAVY        // Background color
#define TFT_FRAME TFT_WHITE    // Frame color
#define TFT_FWD_BAR 0xBFE0     // Forward power bar color (light green)
#define TFT_REV_BAR 0xFD20     // Reverse power bar color (orange)
#define TFT_BAR_BG 0xC618      // Bar background color (gray)
#define TFT_FWD_TEXT TFT_GREEN // Forward text color
#define TFT_REV_TEXT TFT_ORANGE// Reverse text color

// Screen dimensions and bar graph parameters
#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 320
#define BAR_BARS 25            // Number of bars in graph
#define BAR_WIDTH 12           // Width of each bar
#define BAR_HEIGHT 36          // Height of each bar
#define BAR_SPACING 2          // Spacing between bars

// Initialize components
TFT_eSPI tft = TFT_eSPI();     // TFT display instance
Adafruit_ADS1115 ads;           // ADC converter
WebServer server(80);           // Web server on port 80
Preferences preferences;        // Non-volatile storage

// WiFi configuration variables
const char* ap_ssid = "SWR_Meter_Config"; // AP mode SSID
const char* ap_password = "12345678";     // AP mode password
String wifi_ssid = "";          // Client mode SSID
String wifi_password = "";       // Client mode password
bool wifi_connected = false;     // Connection status
int counter_i = 0;               // Display update counter

// Configuration structure with default values
struct Config {
  float power_reference = 2000.0;// Full-scale reference power (Watts)
  int avg_window_ch0 = 5;        // Moving average window size
  int avg_window_ch1 = 5;        // Moving average window size
  int median_window_ch0 = 11;    // Median filter window size (odd)
  int median_window_ch1 = 11;    // Median filter window size (odd)
  float peak_hold_time = 1.0;    // Peak hold duration in seconds
  int sample_rate = 860;         // ADC sample rate (SPS)
  int max_counter_i = 10;        // Max display update delay 
  
  // Neue Kalibrierpunkte für beide Kanäle (in Volt)
  float cal_points_ch0[11] = {0.0}; // Für -40,-30,-20,-10,0,10,20,30,40,50,60 dBm
  float cal_points_ch1[11] = {0.0}; // Für -40,-30,-20,-10,0,10,20,30,40,50,60 dBm

  float wifi_tx_power = 20.5; // WiFi-Sendeleistung in dBm (Standard 20.5 dBm = 100 mW)
} config;

// Measurement data structure
struct MeasurementData {
  float voltage_ch0 = 0.0;       // Channel 0 voltage (V)
  float voltage_ch1 = 0.0;       // Channel 1 voltage (V)
  float power_forward_w = 0.0;   // Forward power (Watts)
  float power_reverse_w = 0.0;   // Reverse power (Watts)
  float power_forward_dbm = 0.0; // Forward power (dBm)
  float power_reverse_dbm = 0.0; // Reverse power (dBm)
  float swr = 1.0;               // Standing Wave Ratio
  float return_loss_db = 0.0;    // Return loss (dB)
  float swr_alt = 1.00;          // Alternative SWR storage for display
} measurement;

// ========== Filter Classes ==========
// Moving Average Filter
class MovingAverage {
private:
  float* buffer;      // Data buffer
  int window_size;     // Size of moving window
  int index;           // Current buffer index
  bool filled;         // Buffer filled flag
  float sum;           // Running sum

public:
  // Constructor
  MovingAverage(int size) : window_size(size), index(0), filled(false), sum(0) {
    buffer = new float[size];
    for(int i = 0; i < size; i++) buffer[i] = 0;
  }
  
  // Destructor
  ~MovingAverage() { delete[] buffer; }
  
  // Update filter with new value
  float update(float value) {
    sum -= buffer[index];        // Remove oldest value
    buffer[index] = value;       // Add new value
    sum += value;
    index = (index + 1) % window_size;
    if(index == 0) filled = true;
    return sum / (filled ? window_size : index + 1);
  }
  
  // Resize filter window
  void resize(int new_size) {
    delete[] buffer;
    window_size = new_size;
    buffer = new float[new_size];
    index = 0;
    filled = false;
    sum = 0;
    for(int i = 0; i < new_size; i++) buffer[i] = 0;
  }
};

// Median Filter
class MedianFilter {
private:
  float* buffer;      // Data buffer
  int window_size;     // Size of window
  int index;           // Current buffer index
  bool filled;         // Buffer filled flag

public:
  // Constructor
  MedianFilter(int size) : window_size(size), index(0), filled(false) {
    buffer = new float[size];
    for(int i = 0; i < size; i++) buffer[i] = 0;
  }
  
  // Destructor
  ~MedianFilter() { delete[] buffer; }
  
  // Update filter with new value
  float update(float value) {
    buffer[index] = value;
    index = (index + 1) % window_size;
    if(index == 0) filled = true;
    int n = filled ? window_size : index;
    float* temp = new float[n];  // Temporary array for sorting
    for(int i = 0; i < n; i++) temp[i] = buffer[i];
    std::sort(temp, temp + n);   // Sort values
    // Calculate median
    float median = (n % 2 == 0) ? 
                  (temp[n/2-1] + temp[n/2]) / 2.0 : 
                  temp[n/2];
    delete[] temp;
    return median;
  }
  
  // Resize filter window
  void resize(int new_size) {
    delete[] buffer;
    window_size = new_size;
    buffer = new float[new_size];
    index = 0;
    filled = false;
    for(int i = 0; i < new_size; i++) buffer[i] = 0;
  }
};

// Filter instances
MovingAverage avg_ch0(5), avg_ch1(5);      // Default 5-point moving average
MedianFilter med_ch0(11), med_ch1(11);     // Default 11-point median filter

// Peak hold variables
float peak_forward = 0.0, peak_reverse = 0.0;
unsigned long peak_forward_time = 0, peak_reverse_time = 0;

// ========== Web Interface (HTML) ==========
// Main page HTML
String getMainPage() {
  return R"=====(
<!DOCTYPE html>
<html>
<head>
<title>SWR Meter Hauptseite</title>
<meta charset='utf-8'>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<style>
body { font-family: Arial; background:#223388; color:#fff; margin:0; padding:20px; }
.container { max-width:600px; margin:auto; }
a { color:#4CAF50; text-decoration:none; font-size:1.2em; display:block; margin:20px 0; }
a:hover { text-decoration:underline; }
</style>
</head>
<body>
<div class='container'>
<h1>SWR Meter</h1>
<a href='/monitor'>Messwerte &amp; Status</a>
<a href='/config'>Konfiguration</a>
<a href='/calibration'>Kalibrierung</a>
<a href='/ota'>OTA Update</a>
</div>
</body>
</html>
)=====";
}

// Monitoring page HTML
String getMonitorPage() {
  return R"=====(
<!DOCTYPE html>
<html>
<head>
<title>SWR Meter Monitor</title>
<meta charset='utf-8'>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<style>
body { font-family: Arial; background:#223388; color:#fff; margin:0; padding:20px; }
.container { max-width:900px; margin:auto; }
.card { background:#1a2a5a; border-radius:8px; margin-bottom:18px; padding:18px; box-shadow:0 2px 8px #0004; }
.value { font-size:2em; font-weight:bold; }
.label { color:#aad; font-size:1em; }
.bargraph { background:#223388; border-radius:8px; margin:10px 0; height:32px; position:relative; }
.bar { position:absolute; bottom:0; width:12px; border-radius:3px 3px 0 0; }
a { color:#4CAF50; text-decoration:none; font-size:1.2em; display:block; margin:20px 0; }
a:hover { text-decoration:underline; }
</style>
</head>
<body>
<div class='container'>
<h1>Messwerte &amp; Status</h1>
<div class='card'>
<div style='display:flex;justify-content:space-between;'>
<div><div class='label'>SWR</div><div class='value' id='swr'>1.00</div></div>
<div><div class='label'>Power, W</div><div class='value' id='power-forward'>0.000</div></div>
</div>
</div>
<div class='card'>
<div class='label'>FWD</div>
<div style='display:flex;align-items:center;'>
<div style='color:#8f8;font-size:1.3em;width:60px;text-align:right;'><span id='fwd-watt'>0</span></div>
<div style='width:8px;'></div>
<div class='bargraph' style='flex:1;position:relative;height:36px;' id='fwd-bargraph'></div>
</div>
<div style='display:flex;justify-content:space-between;font-size:0.95em;'>
<span>0</span><span>10</span><span>20</span><span>30</span><span>40</span><span>50</span><span>60</span><span>70</span><span>80</span><span>90</span><span>100</span>
</div>
</div>
<div class='card'>
<div class='label'>REV</div>
<div style='display:flex;align-items:center;'>
<div style='color:#ff9800;font-size:1.3em;width:60px;text-align:right;'><span id='rev-watt'>0</span></div>
<div style='width:8px;'></div>
<div class='bargraph' style='flex:1;position:relative;height:36px;' id='rev-bargraph'></div>
</div>
<div style='display:flex;justify-content:space-between;font-size:0.95em;'>
<span>0</span><span>10</span><span>20</span><span>30</span><span>40</span><span>50</span><span>60</span><span>70</span><span>80</span><span>90</span><span>100</span>
</div>
</div>
<div class='card'>
<div class='label'>Rohdaten</div>
<div>Kanal 0: <span id='voltage-ch0'>0.0000</span> V</div>
<div>Kanal 1: <span id='voltage-ch1'>0.0000</span> V</div>
<div>FWD dBm: <span id='power-forward-dbm'>0.0</span> dBm</div>
<div>REV dBm: <span id='power-reverse-dbm'>0.0</span> dBm</div>
<div>Rückflussdämpfung: <span id='return-loss'>0.0</span> dB</div>
</div>
<a href='/'>Zurück zur Hauptseite</a>
<a href='/config'>Zur Konfiguration</a>
<a href='/ota'>OTA Update</a>
</div>
<script>
function drawBargraph(id, percent, color) {
  let bar = document.getElementById(id);
  bar.innerHTML = '';
  let bars = 50;
  let on = Math.round(bars * percent / 100);
  for(let i=0;i<bars;i++) {
    let div = document.createElement('div');
    div.className='bar';
    div.style.left = (i*16)+'px';
    div.style.height = '36px';
    div.style.width = '12px';
    div.style.background = (i<on) ? color : '#888';
    bar.appendChild(div);
  }
}
function loadData() {
  fetch('/data').then(r=>r.json()).then(data=>{
    document.getElementById('swr').textContent = data.swr.toFixed(2);
    document.getElementById('power-forward').textContent = data.power_forward_w.toFixed(4);
    document.getElementById('fwd-watt').textContent = data.power_forward_w.toFixed(0);
    document.getElementById('rev-watt').textContent = data.power_reverse_w.toFixed(0);
    document.getElementById('voltage-ch0').textContent = data.voltage_ch0.toFixed(4);
    document.getElementById('voltage-ch1').textContent = data.voltage_ch1.toFixed(4);
    document.getElementById('power-forward-dbm').textContent = data.power_forward_dbm.toFixed(1);
    document.getElementById('power-reverse-dbm').textContent = data.power_reverse_dbm.toFixed(1);
    document.getElementById('return-loss').textContent = data.return_loss_db.toFixed(1);
    let fwdp = Math.min(100, Math.max(0, (data.power_forward_w / data.power_reference * 100)));
    let revp = Math.min(100, Math.max(0, (data.power_reverse_w / data.power_reference * 100)));
    drawBargraph('fwd-bargraph', fwdp, '#8f8');
    drawBargraph('rev-bargraph', revp, '#ff9800');
  });
}
setInterval(loadData, 1000);
loadData();
</script>
</body>
</html>
)=====";
}

// Configuration page HTML
String getConfigPage() {
  String html = R"=====(
<!DOCTYPE html>
<html>
<head>
<title>SWR Meter Konfiguration</title>
<meta charset='utf-8'>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<style>
body { font-family: Arial; background:#223388; color:#fff; margin:0; padding:20px; }
.container { max-width:600px; margin:auto; }
.card { background:#1a2a5a; border-radius:8px; margin-bottom:18px; padding:18px; box-shadow:0 2px 8px #0004; }
label { display:block; margin-top:10px; }
input,select { width:100%; padding:6px; border-radius:4px; border:1px solid #444; margin-top:4px; }
button { background:#4CAF50; color:white; padding:10px 20px; border:none; border-radius:4px; cursor:pointer; margin-top:15px; }
button:hover { background:#45a049; }
a { color:#4CAF50; text-decoration:none; display:block; margin-top:20px; }
a:hover { text-decoration:underline; }
</style>
</head>
<body>
<div class='container'>
<h1>Konfiguration</h1>
<div class='card'>
<label>Leistungsreferenz (100%):</label>
<select id='power-reference'>
)=====";

  // Dynamische Auswahl für Leistungsreferenz
  const float refValues[] = {2000, 1000, 500, 200, 100, 50, 10, 1, 0.1, 0.01, 0.001, 0.0001};
  for (float value : refValues) {
    html += "<option value='" + String(value) + "'";
    if (fabs(value - config.power_reference) < 0.001) {
      html += " selected";
    }
    html += ">" + String(value) + "W</option>";
  }
  
  html += R"=====(
</select>
<label>Messrate (Hz):</label>
<input type='number' min='1' max='3300' id='sample-rate' value=')=====";
  html += String(config.sample_rate);
  html += R"=====('>
<label>Mittelwert Kanal 0:</label>
<input type='number' min='1' max='100' id='avg-window-ch0' value=')=====";
  html += String(config.avg_window_ch0);
  html += R"=====('>
<label>Mittelwert Kanal 1:</label>
<input type='number' min='1' max='100' id='avg-window-ch1' value=')=====";
  html += String(config.avg_window_ch1);
  html += R"=====('>
<label>Median Kanal 0:</label>
<input type='number' min='3' max='101' step='2' id='median-window-ch0' value=')=====";
  html += String(config.median_window_ch0);
  html += R"=====('>
<label>Median Kanal 1:</label>
<input type='number' min='3' max='101' step='2' id='median-window-ch1' value=')=====";
  html += String(config.median_window_ch1);
  html += R"=====('>
<label>Peak Hold Zeit (s):</label>
<input type='number' step='0.01' min='0.1' max='10' id='peak-hold-time' value=')=====";
  html += String(config.peak_hold_time);
  html += R"=====('>
<label>Update delay SWR/Power:</label>
<input type='number' step='1' id='max-counter' value=')=====";
  html += String(config.max_counter_i);
  html += R"=====('>
<button onclick='saveConfig()'>Konfiguration speichern</button>
</div>
<div class='card'>
<h3>WLAN-Client Zugang</h3>
<label>SSID:</label>
<input type='text' id='wifi-ssid' value=')=====";
  html += wifi_ssid;
  html += R"=====('>
<label>Passwort:</label>
<input type='password' id='wifi-password' value=')=====";
  html += wifi_password;
  html += R"=====('>
  <!-- WiFi-Sendeleistung -->
    <label>WiFi Sendeleistung (dBm):</label>
    <input type='number' step='0.25' min='0' max='20.5' id='wifi-tx-power' value=')=====";
    html += String(config.wifi_tx_power, 2);  // Wert mit 2 Dezimalstellen
    
    html += R"=====('>
<button onclick='saveWiFi()'>WLAN speichern &amp; Neustart</button>
</div>
<a href='/'>Zurück zur Hauptseite</a>
<a href='/calibration'>Zur Kalibrierung</a>
<a href='/ota'>OTA Update</a>
</div>
<script>
function saveConfig() {
  let d = {
    power_reference: parseFloat(document.getElementById('power-reference').value),
    sample_rate: parseInt(document.getElementById('sample-rate').value),
    avg_window_ch0: parseInt(document.getElementById('avg-window-ch0').value),
    avg_window_ch1: parseInt(document.getElementById('avg-window-ch1').value),
    median_window_ch0: parseInt(document.getElementById('median-window-ch0').value),
    median_window_ch1: parseInt(document.getElementById('median-window-ch1').value),
    peak_hold_time: parseFloat(document.getElementById('peak-hold-time').value),
    max_counter_i: parseInt(document.getElementById('max-counter').value)
  };
  fetch('/config', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify(d)
  }).then(r=>r.text()).then(r=>{ alert('Konfiguration gespeichert!'); });
}
function saveWiFi() {
  const wifiData = {
    ssid: document.getElementById('wifi-ssid').value,
    password: document.getElementById('wifi-password').value,
    tx_power: parseFloat(document.getElementById('wifi-tx-power').value)
  };
  fetch('/wifi', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(wifiData)
  }).then(r=>r.text()).then(r=>{ alert('WLAN gespeichert! Gerät startet neu...'); });
}
</script>
</body>
</html>
)=====";
  return html;
}

// Kalibrierungsseite HTML mit Rohdaten-Anzeige und Übernahme-Buttons
String getCalibrationPage() {
  String html = R"=====(
<!DOCTYPE html>
<html>
<head>
<title>Kalibrierung</title>
<meta charset='utf-8'>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<style>
body { font-family: Arial; background:#223388; color:#fff; margin:0; padding:20px; }
.container { max-width:100%; margin:auto; overflow-x: auto; }
.card { background:#1a2a5a; border-radius:8px; margin-bottom:18px; padding:18px; box-shadow:0 2px 8px #0004; }
label { display:block; margin-top:10px; }
button { background:#4CAF50; color:white; padding:10px 20px; border:none; border-radius:4px; cursor:pointer; margin-top:15px; }
button:hover { background:#45a049; }
a { color:#4CAF50; text-decoration:none; display:block; margin-top:20px; }
a:hover { text-decoration:underline; }
table { width:100%; border-collapse:collapse; margin:10px 0; font-size: 0.9em; }
th, td { border:1px solid #444; padding:6px; text-align:center; }
input { width: 80px; padding: 4px; font-size: 0.9em; text-align: center; }
.scroll-container { overflow-x: auto; max-width: 100%; }
.use-btn { 
  background: #4CAF50; 
  color: white; 
  border: none; 
  border-radius: 3px; 
  padding: 3px 6px; 
  cursor: pointer; 
  margin-top: 4px;
}
.use-btn:hover { background: #45a049; }
.voltage-display { font-weight: bold; color: #4CAF50; }
</style>
</head>
<body>
<div class='container'>
<h1>Kalibrierung</h1>
<div class='card'>
  <h3>Aktuelle Rohdaten</h3>
  <div>Kanal 0 (A0): <span class="voltage-display" id="currentVoltage0">0.00000</span> V</div>
  <div>Kanal 1 (A1): <span class="voltage-display" id="currentVoltage1">0.00000</span> V</div>
</div>

<div class='card'>
<h3>Kanal 0 Kalibrierpunkte (V)</h3>
<div class="scroll-container">
<table>
<tr>
  <th>dBm</th><th>-40</th><th>-30</th><th>-20</th><th>-10</th>
  <th>0</th><th>10</th><th>20</th><th>30</th>
  <th>40</th><th>50</th><th>60</th>
</tr>
<tr>
  <td>Spannung</td>
)=====";

  // Tabelle für Kanal 0 mit Übernahme-Buttons
  for (int i = 0; i < 11; i++) {
    html += "<td>";
    html += "<input type='number' step='0.00001' min='0' max='3' value='";
    html += String(config.cal_points_ch0[i], 5);
    html += "' id='cal0_";
    html += String(i);
    html += "'>";
    html += "<button class='use-btn' onclick='useCurrentValue(0, ";
    html += String(i);
    html += ")'>Übernehmen</button>";
    html += "</td>";
  }
  html += "</tr></table></div>";

  html += R"=====(
<h3>Kanal 1 Kalibrierpunkte (V)</h3>
<div class="scroll-container">
<table>
<tr>
  <th>dBm</th><th>-40</th><th>-30</th><th>-20</th><th>-10</th>
  <th>0</th><th>10</th><th>20</th><th>30</th>
  <th>40</th><th>50</th><th>60</th>
</tr>
<tr>
  <td>Spannung</td>
)=====";

  // Tabelle für Kanal 1 mit Übernahme-Buttons
  for (int i = 0; i < 11; i++) {
    html += "<td>";
    html += "<input type='number' step='0.00001' min='0' max='3' value='";
    html += String(config.cal_points_ch1[i], 5);
    html += "' id='cal1_";
    html += String(i);
    html += "'>";
    html += "<button class='use-btn' onclick='useCurrentValue(1, ";
    html += String(i);
    html += ")'>Übernehmen</button>";
    html += "</td>";
  }
  html += "</tr></table></div>";

  html += R"=====(
<button onclick='saveCalibration()'>Kalibrierung speichern</button>
</div>
<a href='/config'>Zurück zur Konfiguration</a>
</div>
<script>
// Aktuelle Rohdaten abrufen und anzeigen
function updateRawData() {
  fetch('/rawdata').then(r=>r.json()).then(data=>{
    document.getElementById('currentVoltage0').textContent = data.voltage_ch0.toFixed(5);
    document.getElementById('currentVoltage1').textContent = data.voltage_ch1.toFixed(5);
  });
}

// Aktuellen Wert in das entsprechende Feld übernehmen
function useCurrentValue(channel, index) {
  const voltageElement = document.getElementById('currentVoltage' + channel);
  const inputElement = document.getElementById('cal' + channel + '_' + index);
  
  if (voltageElement && inputElement) {
    inputElement.value = voltageElement.textContent;
  }
}

// Kalibrierdaten speichern
function saveCalibration() {
  const calData = {
    ch0: [],
    ch1: []
  };
  
  // Kanal 0 Werte sammeln
  for(let i=0; i<11; i++) {
    calData.ch0.push(parseFloat(document.getElementById('cal0_'+i).value));
  }
  
  // Kanal 1 Werte sammeln
  for(let i=0; i<11; i++) {
    calData.ch1.push(parseFloat(document.getElementById('cal1_'+i).value));
  }
  
  fetch('/calibration', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify(calData)
  }).then(r=>r.text()).then(r=>{ 
    alert('Kalibrierung gespeichert!'); 
  });
}

// Rohdaten regelmäßig aktualisieren
setInterval(updateRawData, 500);
updateRawData(); // Initial aufrufen
</script>
</body>
</html>
)=====";
  return html;
}

// OTA Update page HTML
String getOTAPage() {
  return R"=====(
<!DOCTYPE html>
<html>
<head>
<title>OTA Update</title>
<meta charset='utf-8'>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<style>
body { font-family: Arial; background:#223388; color:#fff; margin:0; padding:20px; }
.container { max-width:600px; margin:auto; }
.card { background:#1a2a5a; border-radius:8px; margin-bottom:18px; padding:18px; box-shadow:0 2px 8px #0004; }
input[type=file] { width:100%; padding:6px; margin-top:10px; margin-bottom:10px; }
button { background:#4CAF50; color:white; padding:10px 20px; border:none; border-radius:4px; cursor:pointer; }
button:hover { background:#45a049; }
a { color:#4CAF50; text-decoration:none; display:block; margin-top:20px; }
a:hover { text-decoration:underline; }
</style>
</head>
<body>
<div class='container'>
<h1>OTA Firmware Update</h1>
<div class='card'>
<form method='POST' action='/update' enctype='multipart/form-data'>
<input type='file' name='update'>
<button type='submit'>Update starten</button>
</form>
</div>
<a href='/'>Zurück zur Hauptseite</a>
<a href='/monitor'>Messwerte</a>
<a href='/config'>Konfiguration</a>
</div>
</body>
</html>
)=====";
}

// ========== Setup & Main Loop ==========
void setup() {
  Serial.begin(115200);
  preferences.begin("swr_meter", false);  // Initialize NVS
  loadConfiguration();                    // Load saved config
  
  // Initialize display
  tft.init();
  tft.setRotation(1);                     // Landscape orientation
  tft.fillScreen(TFT_BG);
  tft.setTextColor(TFT_FRAME);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.print("SWR Meter Initialisierung...");
  
  // Initialize ADC
  if (!ads.begin()) {
    tft.setCursor(10, 40);
    tft.setTextColor(TFT_RED);
    tft.print("ADS1115 Fehler!");
    while (1) delay(1000);  // Halt on error
  }
  ads.setGain(GAIN_TWOTHIRDS);            // ±6.144V range
  ads.setDataRate(RATE_ADS1115_860SPS);   // 860 samples/second
  
  // Resize filters based on config
  avg_ch0.resize(config.avg_window_ch0);
  avg_ch1.resize(config.avg_window_ch1);
  med_ch0.resize(config.median_window_ch0);
  med_ch1.resize(config.median_window_ch1);
  
  setupWiFi();        // Configure WiFi
  setupWebServer();   // Start web server
  drawDisplayLayout();// Draw static UI elements
}

void loop() {
  server.handleClient();     // Handle web requests
  performMeasurements();     // Take new measurements
  updateDisplay();           // Update TFT display
  managePeakHold();          // Update peak hold values
  //delay(10);                 // Main loop delay
}

// ========== Configuration Handling ==========
// Load configuration from NVS
void loadConfiguration() {
  config.power_reference = preferences.getFloat("power_ref", 2000.0);
  config.avg_window_ch0 = preferences.getInt("avg_ch0", 5);
  config.avg_window_ch1 = preferences.getInt("avg_ch1", 5);
  config.median_window_ch0 = preferences.getInt("med_ch0", 11);
  config.median_window_ch1 = preferences.getInt("med_ch1", 11);
  config.peak_hold_time = preferences.getFloat("peak_hold", 1.0);
  config.sample_rate = preferences.getInt("sample_rate", 860);
  config.max_counter_i = preferences.getInt("max_counter", 10);
  config.wifi_tx_power = preferences.getFloat("wifi_tx_power", 20.5); // Neu
  wifi_ssid = preferences.getString("wifi_ssid", "");
  wifi_password = preferences.getString("wifi_pass", "");
  
  // Lade Kalibrierpunkte
  for (int i = 0; i < 11; i++) {
    config.cal_points_ch0[i] = preferences.getFloat(("cal0_"+String(i)).c_str(), 0.0);
    config.cal_points_ch1[i] = preferences.getFloat(("cal1_"+String(i)).c_str(), 0.0);
  }
}

// Save configuration to NVS
void saveConfiguration() {
  preferences.putFloat("power_ref", config.power_reference);
  preferences.putInt("avg_ch0", config.avg_window_ch0);
  preferences.putInt("avg_window_ch1", config.avg_window_ch1);
  preferences.putInt("med_ch0", config.median_window_ch0);
  preferences.putInt("med_ch1", config.median_window_ch1);
  preferences.putFloat("peak_hold", config.peak_hold_time);
  preferences.putInt("sample_rate", config.sample_rate);
  preferences.putInt("max_counter", config.max_counter_i);
    
  drawDisplayLayout();  // Redraw UI after config change
}

// ========== WiFi & Webserver ==========
void setupWiFi() {
  // Set WiFi TX power
  int tx_power = (int)(config.wifi_tx_power * 4); // Convert to 0.25 dBm units
   WiFi.setTxPower(static_cast<wifi_power_t>(config.wifi_tx_power * 4));
  // Try to connect to saved WiFi
  if (wifi_ssid.length() > 0) {
    WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str());
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) { 
      delay(500); 
      attempts++; 
    }
    if (WiFi.status() == WL_CONNECTED) wifi_connected = true;
  }
  
  // Fallback to AP mode if connection fails
  if (!wifi_connected) {
    WiFi.softAP(ap_ssid, ap_password);
  }
  
  MDNS.begin("swr-meter");  // Start mDNS responder
}

void setupWebServer() {
  // Setup web server routes
  server.on("/", HTTP_GET, []() { server.send(200, "text/html", getMainPage()); });
  server.on("/monitor", HTTP_GET, []() { server.send(200, "text/html", getMonitorPage()); });
  server.on("/config", HTTP_GET, []() { server.send(200, "text/html", getConfigPage()); });
  server.on("/calibration", HTTP_GET, []() { server.send(200, "text/html", getCalibrationPage()); });
  server.on("/ota", HTTP_GET, []() { server.send(200, "text/html", getOTAPage()); });
  
  // Data endpoint
  server.on("/data", HTTP_GET, []() {
    DynamicJsonDocument doc(1024);
    doc["power_forward_w"] = measurement.power_forward_w;
    doc["power_reverse_w"] = measurement.power_reverse_w;
    doc["power_forward_dbm"] = measurement.power_forward_dbm;
    doc["power_reverse_dbm"] = measurement.power_reverse_dbm;
    doc["swr"] = measurement.swr;
    doc["return_loss_db"] = measurement.return_loss_db;
    doc["voltage_ch0"] = measurement.voltage_ch0;
    doc["voltage_ch1"] = measurement.voltage_ch1;
    doc["power_reference"] = config.power_reference;
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });
  
  // Neues Endpoint für Rohdaten (für Kalibrierung)
  server.on("/rawdata", HTTP_GET, []() {
    DynamicJsonDocument doc(200);
    doc["voltage_ch0"] = measurement.voltage_ch0;
    doc["voltage_ch1"] = measurement.voltage_ch1;
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });
  
  // Config GET endpoint
  server.on("/config", HTTP_GET, []() {
    DynamicJsonDocument doc(1024);
    doc["power_reference"] = config.power_reference;
    doc["avg_window_ch0"] = config.avg_window_ch0;
    doc["avg_window_ch1"] = config.avg_window_ch1;
    doc["median_window_ch0"] = config.median_window_ch0;
    doc["median_window_ch1"] = config.median_window_ch1;
    doc["peak_hold_time"] = config.peak_hold_time;
    doc["sample_rate"] = config.sample_rate;
    doc["max_counter_i"] = config.max_counter_i;

    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });
  
  // Config POST endpoint
  server.on("/config", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, server.arg("plain"));
      if (!error) {
        config.power_reference = doc["power_reference"];
        config.avg_window_ch0 = doc["avg_window_ch0"];
        config.avg_window_ch1 = doc["avg_window_ch1"];
        config.median_window_ch0 = doc["median_window_ch0"];
        config.median_window_ch1 = doc["median_window_ch1"];
        config.peak_hold_time = doc["peak_hold_time"];
        config.sample_rate = doc["sample_rate"];
        config.max_counter_i = doc["max_counter_i"];
        
        // Resize filters with new settings
        avg_ch0.resize(config.avg_window_ch0);
        avg_ch1.resize(config.avg_window_ch1);
        med_ch0.resize(config.median_window_ch0);
        med_ch1.resize(config.median_window_ch1);
        
        saveConfiguration();
        server.send(200, "text/plain", "OK");
      } else {
        server.send(400, "text/plain", "JSON Error");
      }
    } else {
      server.send(400, "text/plain", "No Data");
    }
  });
  
  // WiFi config GET endpoint
  server.on("/wifi", HTTP_GET, []() {
    DynamicJsonDocument doc(256);
    doc["ssid"] = wifi_ssid;
    doc["password"] = wifi_password;
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });
  
  // WiFi config POST endpoint
  server.on("/wifi", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      DynamicJsonDocument doc(512);
      DeserializationError error = deserializeJson(doc, server.arg("plain"));
      if (!error) {
        wifi_ssid = doc["ssid"].as<String>();
        wifi_password = doc["password"].as<String>();
        config.wifi_tx_power = doc["tx_power"];
        
        preferences.putString("wifi_ssid", wifi_ssid);
        preferences.putString("wifi_pass", wifi_password);
        preferences.putFloat("wifi_tx_power", config.wifi_tx_power);
        
        server.send(200, "text/plain", "OK");
        delay(1000);
        ESP.restart();  // Reboot to apply new WiFi settings
      } else {
        server.send(400, "text/plain", "JSON Error");
      }
    } else {
      server.send(400, "text/plain", "No Data");
    }
  });
  
  // Kalibrierung POST endpoint
  server.on("/calibration", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, server.arg("plain"));
      if (!error) {
        // Kanal 0 Kalibrierpunkte
        JsonArray ch0 = doc["ch0"];
        for (int i = 0; i < 11 && i < ch0.size(); i++) {
          config.cal_points_ch0[i] = ch0[i];
          preferences.putFloat(("cal0_"+String(i)).c_str(), config.cal_points_ch0[i]);
        }
        
        // Kanal 1 Kalibrierpunkte
        JsonArray ch1 = doc["ch1"];
        for (int i = 0; i < 11 && i < ch1.size(); i++) {
          config.cal_points_ch1[i] = ch1[i];
          preferences.putFloat(("cal1_"+String(i)).c_str(), config.cal_points_ch1[i]);
        }
        
        server.send(200, "text/plain", "OK");
      } else {
        server.send(400, "text/plain", "JSON Error");
      }
    } else {
      server.send(400, "text/plain", "No Data");
    }
  });
  
  // OTA update handler
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        Serial.printf("Update Success: %u bytes\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });
  
  server.begin();  // Start web server
}

// ========== Neue Funktion für Kalibrierung ==========
/**
 * Konvertiert Spannung zu dBm mittels linearer Interpolation zwischen Kalibrierpunkten
 * @param voltage Gemessene Spannung (V)
 * @param cal_points Kalibrierpunkte-Array (11 Werte für -40,-30,...,60 dBm)
 * @return dBm-Wert (extrapoliert bei Werten außerhalb des Bereichs)
 */
float interpolateDbm(float voltage, const float cal_points[]) {
  // dBm-Referenzwerte für die Kalibrierpunkte
  const float dbm_ref[11] = {-40, -30, -20, -10, 0, 10, 20, 30, 40, 50, 60};
  
  // Unterhalb des minimalen Kalibrierpunkts extrapolieren
  if (voltage <= cal_points[0]) {
    return dbm_ref[0] + ((voltage - cal_points[0]) * 10.0 / (cal_points[1] - cal_points[0]));
  }
  
  // Oberhalb des maximalen Kalibrierpunkts extrapolieren
  if (voltage >= cal_points[10]) {
    return dbm_ref[10] + ((voltage - cal_points[10]) * 10.0 / (cal_points[10] - cal_points[9]));
  }
  
  // Interpolation zwischen den Punkten
  for (int i = 0; i < 10; i++) {
    if (voltage >= cal_points[i] && voltage <= cal_points[i+1]) {
      float ratio = (voltage - cal_points[i]) / (cal_points[i+1] - cal_points[i]);
      return dbm_ref[i] + ratio * (dbm_ref[i+1] - dbm_ref[i]);
    }
  }
  
  return 0.0; // Sollte nicht erreicht werden
}

// ========== Measurement & Calculation ==========
void performMeasurements() {
  // Read ADC values
  int16_t adc0 = ads.readADC_SingleEnded(0);
  int16_t adc1 = ads.readADC_SingleEnded(1);
  
  // Convert to voltages
  float voltage0 = ads.computeVolts(adc0);
  float voltage1 = ads.computeVolts(adc1);
  
  // Apply filters
  float voltage0_med = med_ch0.update(voltage0);
  float voltage1_med = med_ch1.update(voltage1);
  measurement.voltage_ch0 = avg_ch0.update(voltage0_med);
  measurement.voltage_ch1 = avg_ch1.update(voltage1_med);
  
  // Calculate power in dBm mit Kalibrierpunkten
  measurement.power_forward_dbm = interpolateDbm(
    measurement.voltage_ch0, 
    config.cal_points_ch0
  );
  
  measurement.power_reverse_dbm = interpolateDbm(
    measurement.voltage_ch1, 
    config.cal_points_ch1
  );
  
  // Convert to Watts
  measurement.power_forward_w = dbmToWatt(measurement.power_forward_dbm);
  measurement.power_reverse_w = dbmToWatt(measurement.power_reverse_dbm);
  
  // Calculate SWR
  if (measurement.power_reverse_w >= measurement.power_forward_w || 
      measurement.power_forward_w <= 0) {
    measurement.swr = 99.0;  // Error case
  } else {
    float r = sqrt(measurement.power_reverse_w / measurement.power_forward_w);
    measurement.swr = (1.0 + r) / (1.0 - r);
    // Clamp SWR values
    if (measurement.swr > 99.0) measurement.swr = 99.0;
    if (measurement.swr < 1.0) measurement.swr = 1.0;
  }
  
  // Calculate return loss
  if (measurement.power_reverse_w > 0 && measurement.power_forward_w > 0) {
    measurement.return_loss_db = 10.0 * log10(
      measurement.power_forward_w / measurement.power_reverse_w
    );
  } else {
    measurement.return_loss_db = 99.9;  // Error case
  }
}

// Convert dBm to Watts
float dbmToWatt(float dbm) {
  return pow(10.0, (dbm - 30.0) / 10.0);
}

// ========== TFT Display Functions ==========
// Draw static UI elements
void drawDisplayLayout() {
  tft.fillScreen(TFT_BG);
  
  // Draw outer frames
  tft.drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, TFT_FRAME);
  tft.drawRect(1, 1, SCREEN_WIDTH-2, SCREEN_HEIGHT-2, TFT_FRAME);
  
  // SWR display area
  tft.fillRect(10, 10, 230, 110, TFT_BG);
  tft.drawRect(10, 10, 230, 110, TFT_FRAME);
  tft.setTextColor(TFT_FRAME); 
  tft.setTextSize(3); 
  tft.setCursor(60, 20); 
  tft.print("SWR");
  
  // Power display area
  tft.fillRect(240, 10, 230, 110, TFT_BG);
  tft.drawRect(240, 10, 230, 110, TFT_FRAME);
  tft.setTextColor(TFT_FRAME); 
  tft.setTextSize(3); 
  tft.setCursor(290, 20); 
  tft.print("Power, W");
  
  // Forward power area
  tft.fillRect(10, 120, 460, 80, TFT_BG);
  tft.drawRect(10, 120, 460, 80, TFT_FRAME);
  tft.setTextColor(TFT_FRAME); 
  tft.setTextSize(2); 
  tft.setCursor(18, 128); 
  tft.print("FWD");
  
  // Reverse power area
  tft.fillRect(10, 200, 460, 80, TFT_BG);
  tft.drawRect(10, 200, 460, 80, TFT_FRAME);
  tft.setTextColor(TFT_FRAME); 
  tft.setTextSize(2); 
  tft.setCursor(18, 208); 
  tft.print("REV");
  
  // Footer
  tft.setTextColor(TFT_LIGHTGREY); 
  tft.setTextSize(2); 
  tft.setCursor(30, 300); 
  tft.print("    SWR/Power Meter DARC OV R14");
  
  // Scale markers
  for(int i=0; i<=10; i++) {
    int scale_value = i * config.power_reference / 10;
    int x = 85 + i * (BAR_WIDTH + BAR_SPACING) * 2.5;
    tft.setTextColor(TFT_FRAME); 
    tft.setTextSize(1);
    tft.setCursor(x-6, 130); 
    tft.printf("%d", scale_value);
    tft.setCursor(x-6, 210); 
    tft.printf("%d", scale_value);
  }
}

// Update dynamic display elements
void updateDisplay() {
  // Update SWR and power values periodically
  if (counter_i <= 0) {
    // Clear SWR area
    tft.fillRect(30, 55, 210, 50, TFT_BG);
    tft.setTextColor(TFT_YELLOW); 
    tft.setTextSize(5); 
    tft.setCursor(55, 55);
    tft.printf("%.2f", measurement.swr);
    
    // Clear power area
    tft.fillRect(270, 55, 180, 50, TFT_BG);
    tft.setTextColor(TFT_CYAN); 
    tft.setTextSize(5); 
    tft.setCursor(290, 55);
    // Format based on power magnitude
    if (measurement.power_forward_w < 10) {
      tft.printf("%.3f", measurement.power_forward_w);
    } else if (measurement.power_forward_w < 200) {
      tft.printf("%.1f", measurement.power_forward_w);
    } else {
      tft.printf("%.0f", measurement.power_forward_w);
    }
    
    counter_i = config.max_counter_i;
  }
  //Serial.println(config.max_counter_i);
  counter_i--;
  
  // Update forward power values
  tft.fillRect(18, 145, 60, 20, TFT_BG);
  tft.setTextColor(TFT_FWD_TEXT); 
  tft.setTextSize(2); 
  tft.setCursor(18, 145);
  if (measurement.power_forward_w < 9900) {
    tft.printf("%.0f", measurement.power_forward_w);
  }
  tft.setCursor(73, 145);
  tft.print("W");
  
  tft.fillRect(18, 178, 60, 20, TFT_BG);
  tft.setTextColor(TFT_FWD_TEXT); 
  tft.setCursor(18, 178);
  tft.printf("%.1f", measurement.power_forward_dbm);
  tft.setCursor(85, 178);
  tft.print("dBm");
  
  // Update reverse power values
  tft.fillRect(18, 225, 60, 20, TFT_BG);
  tft.setTextColor(TFT_REV_TEXT); 
  tft.setCursor(18, 225);
  if (measurement.power_reverse_w < 9900) {
    tft.printf("%.0f", measurement.power_reverse_w);
  }
  tft.setCursor(73, 225);
  tft.print("W");
  
  tft.fillRect(18, 258, 60, 20, TFT_BG);
  tft.setTextColor(TFT_REV_TEXT); 
  tft.setCursor(18, 258);
  tft.printf("%.1f", measurement.power_reverse_dbm);
  tft.setCursor(85, 258); 
  tft.print("dBm");
  
  // Draw bar graphs
  drawBargraph(85, 140, measurement.power_forward_w, peak_forward, 
               config.power_reference, TFT_FWD_BAR, TFT_YELLOW);
  drawBargraph(85, 220, measurement.power_reverse_w, peak_reverse, 
               config.power_reference, TFT_REV_BAR, TFT_ORANGE);
}

// Draw bar graph visualization
void drawBargraph(int x, int y, float current_power, float peak_power, 
                 float max_power, uint16_t normal_color, uint16_t peak_color) {
  // Calculate number of bars to display
  int current_bars = (int)((current_power / max_power) * BAR_BARS + 0.5);
  int peak_bars = (int)((peak_power / max_power) * BAR_BARS + 0.5);
  
  // Clamp values to valid range
  if (current_bars > BAR_BARS) current_bars = BAR_BARS;
  if (peak_bars > BAR_BARS) peak_bars = BAR_BARS;
  
  // Draw main bars
  for (int i = 0; i < BAR_BARS; i++) {
    int bar_x = x + i * (BAR_WIDTH + BAR_SPACING);
    uint16_t color = (i < current_bars) ? normal_color : TFT_BAR_BG;
    tft.fillRect(bar_x, y, BAR_WIDTH, BAR_HEIGHT, color);
  }
  
  // Draw peak indicator
  if (peak_bars > current_bars && peak_bars <= BAR_BARS) {
    int bar_x = x + (peak_bars - 1) * (BAR_WIDTH + BAR_SPACING);
    tft.fillRect(bar_x, y, BAR_WIDTH, BAR_HEIGHT, peak_color);
  }
}

// Manage peak hold functionality
void managePeakHold() {
  unsigned long now = millis();
  
  // Update forward peak
  if (measurement.power_forward_w > peak_forward) {
    peak_forward = measurement.power_forward_w;
    peak_forward_time = now;
  } else if (now - peak_forward_time > (unsigned long)(config.peak_hold_time * 1000)) {
    peak_forward = measurement.power_forward_w;
  }
  
  // Update reverse peak
  if (measurement.power_reverse_w > peak_reverse) {
    peak_reverse = measurement.power_reverse_w;
    peak_reverse_time = now;
  } else if (now - peak_reverse_time > (unsigned long)(config.peak_hold_time * 1000)) {
    peak_reverse = measurement.power_reverse_w;
  }
}