#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

#include "config.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"

// ================== CẤU HÌNH WIFI ==================
// 1. Chế độ Station: Kết nối vào WiFi có sẵn
const char *STA_SSID = "M625 Mechatronic Lab";
const char *STA_PASS = "C7@M.625";

// 2. Chế độ Access Point: ESP32 tự phát sóng để điện thoại kết nối trực tiếp khi test sàn
const char *AP_SSID  = "Linorobot2_PID_Tune";
const char *AP_PASS  = "12345678"; // Tối thiểu 8 ký tự

WebServer        server(80);
WebSocketsServer wsServer(81);

// ================== PHẦN CỨNG (2 ĐỘNG CƠ) ==================
Encoder left_encoder(
    MOTOR1_ENCODER_A,
    MOTOR1_ENCODER_B,
    COUNTS_PER_REV1,
    MOTOR1_ENCODER_INV
);

Encoder right_encoder(
    MOTOR2_ENCODER_A,
    MOTOR2_ENCODER_B,
    COUNTS_PER_REV2,
    MOTOR2_ENCODER_INV
);

Motor left_motor(
    PWM_FREQUENCY,
    PWM_BITS,
    MOTOR1_INV,
    MOTOR1_PWM,
    MOTOR1_IN_A,
    MOTOR1_IN_B
);

Motor right_motor(
    PWM_FREQUENCY,
    PWM_BITS,
    MOTOR2_INV,
    MOTOR2_PWM,
    MOTOR2_IN_A,
    MOTOR2_IN_B
);

// Khởi tạo PID ban đầu
float current_kp = K_P_1;
float current_ki = K_I_1;
float current_kd = K_D_1;

PID left_pid(PWM_MIN, PWM_MAX, current_kp, current_ki, current_kd);
PID right_pid(PWM_MIN, PWM_MAX, current_kp, current_ki, current_kd);

// ================== BIẾN ĐIỀU KHIỂN ==================
float target_rpm       = 80.0;   // RPM mong muốn
String current_dir     = "stop"; // "stop", "fwd", "rev", "left", "right"
float left_target_rpm  = 0.0;
float right_target_rpm = 0.0;

float left_rpm      = 0.0;
float right_rpm     = 0.0;
int   left_pwm      = 0;
int   right_pwm     = 0;

// Tính năng Step Test (Chạy thử t giây rồi tự dừng để bảo vệ an toàn trên sàn)
unsigned long run_until_ms = 0;
bool auto_stop_active      = false;

const uint32_t PID_DT_MS = 20; // 50 Hz - Đồng bộ chuẩn xác với firmware micro-ROS

// ================== GIAO DIỆN WEB HTML/JS ==================
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>Linorobot2 - Tune PID 2 Bánh Sàn</title>
<style>
  body{font-family:system-ui,-apple-system,sans-serif;background:#0f172a;color:#e2e8f0;margin:0;padding:10px;text-align:center}
  h1{font-size:18px;margin:2px 0 4px;color:#38bdf8}
  .card{max-width:760px;margin:8px auto;background:#1e293b;border-radius:14px;padding:12px;box-shadow:0 6px 18px rgba(0,0,0,.35)}
  .row{display:flex;gap:8px;justify-content:center;margin:6px 0}
  button{flex:1;border:0;border-radius:10px;padding:12px 6px;font-size:15px;font-weight:700;color:#fff;cursor:pointer;transition:.12s;user-select:none}
  button:active{transform:scale(.96)}
  .fwd{background:#16a34a;padding:14px}
  .rev{background:#2563eb;padding:14px}
  .turn{background:#0284c7;font-size:15px}
  .stop{background:#dc2626;font-size:17px;font-weight:800;flex:1.2}
  .step{background:#d97706;font-size:13px;padding:10px 4px}
  .apply{background:#7c3aed;padding:13px}
  input[type=range]{width:100%;height:32px;accent-color:#38bdf8}
  .big-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin:4px 0 8px}
  .metric-box{background:#0b1220;border-radius:10px;padding:8px}
  .metric-val{font-size:24px;font-weight:800;line-height:1.2}
  .label{font-size:12px;color:#94a3b8}
  .grid-pid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin:6px 0}
  .grid-pid input{width:100%;box-sizing:border-box;padding:8px;border-radius:8px;border:1px solid #334155;background:#0f172a;color:#f8fafc;text-align:center;font-size:16px;font-weight:600}
  canvas{width:100%;height:270px;background:#070d18;border-radius:10px;display:block}
  .leg{display:flex;gap:12px;justify-content:center;font-size:12px;margin-top:6px;flex-wrap:wrap}
  .dot{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:4px;vertical-align:middle}
  #wsst{font-size:13px;margin-left:4px}
</style>
</head>
<body>
  <h1>TUNE PID 2 BÁNH CÓ TẢI SÀN <span id="wsst">●</span></h1>
  <div class="label">Linorobot2 2WD &middot; D-Pad Điều Hướng &middot; 50Hz Loop</div>

  <!-- BẢNG ĐO THỰC TẾ 2 BÁNH -->
  <div class="card">
    <div class="big-grid">
      <div class="metric-box">
        <div class="label" style="color:#10b981">BÁNH TRÁI (L)</div>
        <div class="metric-val" style="color:#10b981"><span id="lrpm">0.0</span> <small style="font-size:12px">RPM</small></div>
        <div class="label">Đặt: <b id="set-l">0</b> | PWM: <b id="lpwm">0</b></div>
      </div>
      <div class="metric-box">
        <div class="label" style="color:#06b6d4">BÁNH PHẢI (R)</div>
        <div class="metric-val" style="color:#06b6d4"><span id="rrpm">0.0</span> <small style="font-size:12px">RPM</small></div>
        <div class="label">Đặt: <b id="set-r">0</b> | PWM: <b id="rpwm">0</b></div>
      </div>
    </div>
    <canvas id="chart" width="720" height="270"></canvas>
    <div class="leg">
      <span><span class="dot" style="background:#f59e0b"></span>Đặt Trái (Set L)</span>
      <span><span class="dot" style="background:#fbbf24"></span>Đặt Phải (Set R)</span>
      <span><span class="dot" style="background:#10b981"></span>Thực Trái (L)</span>
      <span><span class="dot" style="background:#06b6d4"></span>Thực Phải (R)</span>
    </div>
  </div>

  <!-- BÀN PHÍM ĐIỀU HƯỚNG D-PAD (TIẾN / LÙI / RẼ TRÁI / RẼ PHẢI / DỪNG) -->
  <div class="card">
    <div class="label">Tốc độ đặt: <b id="tgt-val" style="color:#f59e0b;font-size:16px">80</b> RPM &nbsp;|&nbsp; Hướng: <b id="dir-val" style="color:#38bdf8">DỪNG</b></div>
    <input type="range" min="20" max="220" value="80" id="slider" oninput="setTarget(this.value)">
    
    <!-- 1. HÀNG TIẾN -->
    <div class="row">
      <button class="fwd" onclick="setDir('fwd')">&#9650; TIẾN THẲNG</button>
    </div>

    <!-- 2. HÀNG RẼ TRÁI - DỪNG - RẼ PHẢI -->
    <div class="row">
      <button class="turn" onclick="setDir('left')">&#9664; RẼ TRÁI</button>
      <button class="stop" onclick="setDir('stop')">&#9632; DỪNG</button>
      <button class="turn" onclick="setDir('right')">RẼ PHẢI &#9654;</button>
    </div>

    <!-- 3. HÀNG LÙI -->
    <div class="row">
      <button class="rev" onclick="setDir('rev')">&#9660; LÙI THẲNG</button>
    </div>

    <!-- 4. CHẠY TEST BƯỚC AN TOÀN TRÊN SÀN (TỰ DỪNG SAU 2 GIÂY) -->
    <div class="label" style="margin-top:8px">CHẠY THỬ AN TOÀN TRÊN SÀN (TỰ DỪNG SAU 2 GIÂY):</div>
    <div class="row">
      <button class="step" onclick="runStep(2, 'fwd')">&#9201; Test Tiến 2s</button>
      <button class="step" onclick="runStep(2, 'left')">&#9201; Test Trái 2s</button>
      <button class="step" onclick="runStep(2, 'right')">&#9201; Test Phải 2s</button>
      <button class="step" onclick="runStep(2, 'rev')">&#9201; Test Lùi 2s</button>
    </div>
  </div>

  <!-- CHỈNH HỆ SỐ PID THỜI GIAN THỰC -->
  <div class="card">
    <div class="label" style="font-weight:700">TINH CHỈNH PID (ÁP DỤNG NGAY LẬP TỨC)</div>
    <div class="grid-pid">
      <div><div class="label">Kp</div><input id="kp" type="number" step="0.05"></div>
      <div><div class="label">Ki</div><input id="ki" type="number" step="0.1"></div>
      <div><div class="label">Kd</div><input id="kd" type="number" step="0.005"></div>
    </div>
    <div class="row">
      <button class="apply" onclick="applyPID()">LƯU & ÁP DỤNG PID VÀO MOTOR</button>
    </div>
  </div>

<script>
function send(u){ return fetch(u).catch(e=>console.log(e)); }
function setTarget(v){ document.getElementById('tgt-val').innerText=v; send('/rpm?v='+v); }
function setDir(d){ send('/dir?d='+d); }
function runStep(s, d){ send('/step?t='+s+'&d='+d); }
function applyPID(){
  send('/pid?kp='+kp.value+'&ki='+ki.value+'&kd='+kd.value);
}
var kp=document.getElementById('kp'), ki=document.getElementById('ki'), kd=document.getElementById('kd');

// Canvas Chart
var cv=document.getElementById('chart'), cx=cv.getContext('2d');
var W=cv.width, H=cv.height, MID=H/2, PAD=8;
var YR=220, N=180;
var sDataL=[], sDataR=[], lData=[], rData=[];
function pushData(sl,sr,l,r){
  sDataL.push(sl); sDataR.push(sr); lData.push(l); rData.push(r);
  if(lData.length>N){sDataL.shift(); sDataR.shift(); lData.shift(); rData.shift();}
}
function ypix(v){ return MID - (v/YR)*(MID-PAD); }
function drawLine(arr,color,lw,dashed){
  cx.strokeStyle=color; cx.lineWidth=lw;
  if(dashed) cx.setLineDash([4,4]); else cx.setLineDash([]);
  cx.beginPath();
  for(var i=0;i<arr.length;i++){
    var x=arr.length<2?0:i/(N-1)*W;
    var y=ypix(arr[i]);
    if(i===0) cx.moveTo(x,y); else cx.lineTo(x,y);
  }
  cx.stroke();
  cx.setLineDash([]);
}
function draw(){
  cx.clearRect(0,0,W,H);
  // Grid lines
  cx.strokeStyle='#1e293b'; cx.lineWidth=1;
  for(var g=-YR; g<=YR; g+=50){
    var y=ypix(g); cx.beginPath(); cx.moveTo(0,y); cx.lineTo(W,y); cx.stroke();
  }
  // Zero line
  cx.strokeStyle='#334155'; cx.beginPath();
  cx.moveTo(0,MID); cx.lineTo(W,MID); cx.stroke();

  drawLine(sDataL,'#f59e0b',2.0,true);  // Setpoint Left (Nét đứt vàng cam)
  drawLine(sDataR,'#fbbf24',2.0,true);  // Setpoint Right (Nét đứt vàng sáng)
  drawLine(lData,'#10b981',2.5,false);  // Left Actual (Xanh lá)
  drawLine(rData,'#06b6d4',2.5,false);  // Right Actual (Xanh lơ)
}

function dirText(d){
  if(d==='fwd') return 'TIẾN';
  if(d==='rev') return 'LÙI';
  if(d==='left') return 'RẼ TRÁI';
  if(d==='right') return 'RẼ PHẢI';
  return 'DỪNG';
}

// WebSocket connection
var pidLoaded=false, st=document.getElementById('wsst');
function connect(){
  var ws=new WebSocket('ws://'+location.hostname+':81/');
  ws.onopen  =()=>{ st.style.color='#22c55e'; };
  ws.onclose =()=>{ st.style.color='#dc2626'; setTimeout(connect,1000); };
  ws.onmessage=(e)=>{
    var d=JSON.parse(e.data);
    document.getElementById('lrpm').innerText=d.l_rpm.toFixed(1);
    document.getElementById('rrpm').innerText=d.r_rpm.toFixed(1);
    document.getElementById('set-l').innerText=d.set_l.toFixed(0);
    document.getElementById('set-r').innerText=d.set_r.toFixed(0);
    document.getElementById('lpwm').innerText=d.l_pwm;
    document.getElementById('rpwm').innerText=d.r_pwm;
    document.getElementById('dir-val').innerText=dirText(d.dir);
    if(!pidLoaded){ kp.value=d.kp; ki.value=d.ki; kd.value=d.kd; pidLoaded=true; }
    pushData(d.set_l, d.set_r, d.l_rpm, d.r_rpm);
    draw();
  };
}
connect();
</script>
</body>
</html>
)HTML";

// ================== CÁC HÀM XỬ LÝ HTTP ==================
void updateTargetsForDir(String d) {
  current_dir = d;
  if (d == "fwd") {
    left_target_rpm  = target_rpm;
    right_target_rpm = target_rpm;
  } else if (d == "rev") {
    left_target_rpm  = -target_rpm;
    right_target_rpm = -target_rpm;
  } else if (d == "left") {
    // Rẽ trái (quay tại chỗ): Bánh trái lùi, bánh phải tiến
    left_target_rpm  = -target_rpm;
    right_target_rpm = target_rpm;
  } else if (d == "right") {
    // Rẽ phải (quay tại chỗ): Bánh trái tiến, bánh phải lùi
    left_target_rpm  = target_rpm;
    right_target_rpm = -target_rpm;
  } else {
    left_target_rpm  = 0;
    right_target_rpm = 0;
    current_dir      = "stop";
  }
}

void handleRoot() {
  server.send(200, "text/html", INDEX_HTML);
}

void handleRpm() {
  if (server.hasArg("v")) {
    target_rpm = server.arg("v").toFloat();
    if (current_dir != "stop") {
      updateTargetsForDir(current_dir);
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleDir() {
  if (server.hasArg("d")) {
    updateTargetsForDir(server.arg("d"));
    auto_stop_active = false;
  }
  server.send(200, "text/plain", "OK");
}

void handleStep() {
  float sec = 2.0;
  if (server.hasArg("t")) sec = server.arg("t").toFloat();
  if (sec <= 0) sec = 2.0;

  String d = "fwd";
  if (server.hasArg("d")) d = server.arg("d");

  updateTargetsForDir(d);
  run_until_ms     = millis() + (unsigned long)(sec * 1000);
  auto_stop_active = true;

  server.send(200, "text/plain", "OK");
}

void handlePid() {
  if (server.hasArg("kp")) current_kp = server.arg("kp").toFloat();
  if (server.hasArg("ki")) current_ki = server.arg("ki").toFloat();
  if (server.hasArg("kd")) current_kd = server.arg("kd").toFloat();

  left_pid.updateConstants(current_kp, current_ki, current_kd);
  right_pid.updateConstants(current_kp, current_ki, current_kd);

  Serial.printf("[PID CẬP NHẬT] Kp=%.3f | Ki=%.3f | Kd=%.3f\n", current_kp, current_ki, current_kd);
  server.send(200, "text/plain", "OK");
}

void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t len) {
  (void)num; (void)type; (void)payload; (void)len;
}

void broadcastStatus() {
  char buf[250];
  snprintf(buf, sizeof(buf),
    "{\"tgt\":%.1f,\"set_l\":%.1f,\"set_r\":%.1f,\"l_rpm\":%.1f,\"r_rpm\":%.1f,\"l_pwm\":%d,\"r_pwm\":%d,\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,\"dir\":\"%s\"}",
    target_rpm, left_target_rpm, right_target_rpm, left_rpm, right_rpm, left_pwm, right_pwm, current_kp, current_ki, current_kd, current_dir.c_str());
  wsServer.broadcastTXT(buf);
}

// ================== SETUP & LOOP ==================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n==============================================");
  Serial.println(" LINOROBOT2 - BỘ ĐIỀU CHỈNH PID 2 BÁNH TRÊN SÀN");
  Serial.println(" (ĐÃ BỔ SUNG: RẼ TRÁI / RẼ PHẢI TẠI CHỖ)");
  Serial.println("==============================================");

  // Khởi tạo mạng: Bật cả AP và Station
  WiFi.mode(WIFI_AP_STA);
  
  // 1. Phát Access Point để điện thoại kết nối mọi nơi
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("[AP] Tên WiFi: "); Serial.println(AP_SSID);
  Serial.print("[AP] Mật khẩu: "); Serial.println(AP_PASS);
  Serial.print("[AP] Truy cập: http://"); Serial.println(WiFi.softAPIP());

  // 2. Thử kết nối vào WiFi lab
  WiFi.begin(STA_SSID, STA_PASS);
  Serial.print("[STA] Đang kết nối WiFi: "); Serial.println(STA_SSID);
  uint32_t tStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - tStart < 6000) {
    delay(250);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[STA] Kết nối thành công!");
    Serial.print("[STA] Truy cập từ máy tính lab: http://"); Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n[STA] Không tìm thấy WiFi lab -> Bạn có thể kết nối điện thoại vào WiFi AP của robot!");
  }

  // Khởi động Web Server
  server.on("/",     handleRoot);
  server.on("/rpm",  handleRpm);
  server.on("/dir",  handleDir);
  server.on("/step", handleStep);
  server.on("/pid",  handlePid);
  server.begin();

  wsServer.begin();
  wsServer.onEvent(onWsEvent);

  left_pid.reset();
  right_pid.reset();
}

void loop() {
  server.handleClient();
  wsServer.loop();

  // Kiểm tra tự ngắt cho Step Test
  if (auto_stop_active && millis() >= run_until_ms) {
    updateTargetsForDir("stop");
    auto_stop_active = false;
  }

  // VÒNG ĐIỀU KHIỂN PID 50 HZ (20ms)
  static uint32_t tPid = 0;
  if (millis() - tPid >= PID_DT_MS) {
    tPid = millis();

    left_rpm  = left_encoder.getRPM();
    right_rpm = right_encoder.getRPM();

    if (current_dir == "stop" || (left_target_rpm == 0 && right_target_rpm == 0)) {
      left_pid.reset();
      right_pid.reset();
      left_motor.spin(0);
      right_motor.spin(0);
      left_pwm = 0;
      right_pwm = 0;
    } else {
      left_pwm  = left_pid.compute(left_target_rpm, left_rpm);
      right_pwm = right_pid.compute(right_target_rpm, right_rpm);

      left_motor.spin(left_pwm);
      right_motor.spin(right_pwm);
    }
  }

  // BẮN DỮ LIỆU REALTIME LÊN ĐỒ THỊ 10 HZ (100ms)
  static uint32_t tWs = 0;
  if (millis() - tWs >= 100) {
    tWs = millis();
    broadcastStatus();
  }
}
