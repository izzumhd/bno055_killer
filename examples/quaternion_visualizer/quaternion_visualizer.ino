/**
 * ============================================================
 *  BNO055 Killer — Quaternion Visualizer
 *  LSM6DS3 + QMC5883L
 *  by izzumhdh
 * ============================================================
 *  Visualisasi orientasi 3D dari quaternion output Madgwick AHRS.
 *
 *  Mode output:
 *    1 — ASCII Cube (cube 3D diputar di terminal)
 *    2 — Attitude Indicator (horizon, compass, bar graph)
 *    3 — Processing protocol (untuk companion sketch 3D)
 *    4 — JSON (untuk web visualizer)
 *
 *  ────────────────────────────────────────────────────────
 *  MODE 3: Companion Processing Sketch
 *  Paste kode ini di Processing IDE (processing.org):
 * ────────────────────────────────────────────────────────
 *
 *  import processing.serial.*;
 *  Serial port;
 *  float roll, pitch, yaw;
 *
 *  void setup() {
 *    size(600, 600, P3D);
 *    port = new Serial(this, Serial.list()[0], 115200);
 *    port.bufferUntil('\n');
 *  }
 *
 *  void serialEvent(Serial p) {
 *    String s = p.readStringUntil('\n').trim();
 *    if (s.startsWith("!RPY:")) {
 *      String[] v = s.substring(5).split(",");
 *      roll  = Float.parseFloat(v[0]);
 *      pitch = Float.parseFloat(v[1]);
 *      yaw   = Float.parseFloat(v[2]);
 *    }
 *  }
 *
 *  void draw() {
 *    background(30); lights();
 *    translate(width/2, height/2, 0);
 *    rotateZ(radians(roll));
 *    rotateX(radians(-pitch));
 *    rotateY(radians(yaw));
 *    fill(0, 150, 255); stroke(255);
 *    box(150, 40, 250);  // Badan pesawat
 *    fill(255, 100, 0);
 *    pushMatrix(); translate(0,0,-125); box(80,10,40); popMatrix();
 *    fill(200, 200, 0);
 *    pushMatrix(); translate(-120,0,0); box(240,5,50); popMatrix();
 *  }
 *
 * ────────────────────────────────────────────────────────
 *  MODE 4: Web Visualizer
 *  Buka file HTML di browser, hubungkan ke Web Serial API,
 *  parsing JSON: {"w":..,"x":..,"y":..,"z":..}
 * ────────────────────────────────────────────────────────
 */

#include <Wire.h>
#include <bno055_killer.h>

// ============================================================
//  Konfigurasi
// ============================================================
#define VIZ_BAUD         115200
#define VIZ_INTERVAL_MS  50      // 20 Hz refresh (cukup untuk visual)

IMUHandler imu;

// Mode visualisasi — gunakan uint8_t, bukan enum dengan nama umum
uint8_t vizMode = 1;   // 1=ASCII Cube, 2=Attitude, 3=Processing, 4=JSON

// ============================================================
//  ASCII Cube — definisi
// ============================================================
// 8 vertex kubus satuan: [x, y, z]
static const float CUBE_V[8][3] = {
    {-1,-1,-1}, { 1,-1,-1}, { 1, 1,-1}, {-1, 1,-1},  // Belakang
    {-1,-1, 1}, { 1,-1, 1}, { 1, 1, 1}, {-1, 1, 1},  // Depan
};
// 12 edge (pasangan index vertex)
static const uint8_t CUBE_E[12][2] = {
    {0,1},{1,2},{2,3},{3,0},   // Belakang
    {4,5},{5,6},{6,7},{7,4},   // Depan
    {0,4},{1,5},{2,6},{3,7},   // Samping
};

// Buffer karakter layar ASCII (40 kolom x 20 baris)
#define SCR_W 40
#define SCR_H 20
char scr[SCR_H][SCR_W + 1];

// ============================================================
//  Fungsi rotasi quaternion → vertex
// ============================================================
void rotateVec(float w, float x, float y, float z,
               float vx, float vy, float vz,
               float& ox, float& oy, float& oz) {
    // Rotasi vektor v dengan quaternion q: v' = q * [0,v] * q_conj
    float tx = 2.0f * (y*vz - z*vy);
    float ty = 2.0f * (z*vx - x*vz);
    float tz = 2.0f * (x*vy - y*vx);
    ox = vx + w*tx + y*tz - z*ty;
    oy = vy + w*ty + z*tx - x*tz;
    oz = vz + w*tz + x*ty - y*tx;
}

// ============================================================
//  Proyeksi perspektif 3D → 2D pixel
// ============================================================
void project(float x, float y, float z,
             int& px, int& py) {
    float d = 4.0f;    // Jarak kamera
    float scale = 8.0f;
    float pz = z + d;
    if (pz < 0.01f) pz = 0.01f;
    px = (int)(SCR_W/2 + (x / pz) * scale * 1.8f);  // *1.8 koreksi rasio font
    py = (int)(SCR_H/2 - (y / pz) * scale);
}

// ============================================================
//  Bresenham line drawing ke scr buffer
// ============================================================
void drawLine(int x0, int y0, int x1, int y1, char c) {
    int dx = abs(x1-x0), dy = abs(y1-y0);
    int sx = x0<x1 ? 1:-1, sy = y0<y1 ? 1:-1;
    int err = dx - dy;
    while (true) {
        if (x0 >= 0 && x0 < SCR_W && y0 >= 0 && y0 < SCR_H)
            scr[y0][x0] = c;
        if (x0==x1 && y0==y1) break;
        int e2 = 2*err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

// ============================================================
//  Render ASCII Cube
// ============================================================
void renderCube(const Quaternion& q, const EulerAngles& e) {
    // Bersihkan buffer
    for (int r = 0; r < SCR_H; r++) {
        memset(scr[r], ' ', SCR_W);
        scr[r][SCR_W] = '\0';
    }

    // Rotasi semua vertex dengan quaternion
    float rv[8][3];
    for (int i = 0; i < 8; i++) {
        rotateVec(q.w, q.x, q.y, q.z,
                  CUBE_V[i][0], CUBE_V[i][1], CUBE_V[i][2],
                  rv[i][0], rv[i][1], rv[i][2]);
    }

    // Gambar semua edge
    for (int e2 = 0; e2 < 12; e2++) {
        int a = CUBE_E[e2][0], b = CUBE_E[e2][1];
        int x0, y0, x1, y1;
        project(rv[a][0], rv[a][1], rv[a][2], x0, y0);
        project(rv[b][0], rv[b][1], rv[b][2], x1, y1);
        // Edge belakang lebih redup (.)  edge depan lebih terang (+)
        char edgeChar = (e2 < 4) ? '.' : (e2 < 8) ? '+' : '#';
        drawLine(x0, y0, x1, y1, edgeChar);
    }

    // Cetak frame ke Serial (clear screen dengan \033[H = cursor home)
    Serial.print(F("\033[H"));    // ANSI: pindah cursor ke baris 1 kolom 1
    Serial.println(F("  BNO055 Killer — Quaternion Cube"));
    Serial.println(F("  +---------- Edge: . back + front #side ----+"));
    for (int r = 0; r < SCR_H; r++) {
        Serial.print(F("  |"));
        Serial.print(scr[r]);
        Serial.println(F("|"));
    }
    Serial.println(F("  +------------------------------------------+"));
    Serial.printf("  Roll:%7.2f  Pitch:%7.2f  Yaw:%7.2f\r\n",
                  e.roll, e.pitch, e.yaw);
    Serial.printf("  Q: W=%.4f X=%.4f Y=%.4f Z=%.4f\r\n",
                  q.w, q.x, q.y, q.z);
    Serial.println(F("\n  [1]Cube [2]Attitude [3]Processing [4]JSON"));
}

// ============================================================
//  Attitude Indicator — horizon + compass + bar graph
// ============================================================
void printBarH(const char* label, float val, float minV, float maxV, int width) {
    float pct = (val - minV) / (maxV - minV);
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 1.0f) pct = 1.0f;
    int filled = (int)(pct * width);

    Serial.print(label);
    Serial.print(F(" ["));
    for (int i = 0; i < width; i++) {
        Serial.print(i < filled ? '#' : '-');
    }
    Serial.print(F("] "));
    Serial.printf("%7.2f\r\n", val);
}

void renderAttitude(const EulerAngles& e, float heading) {
    Serial.print(F("\033[H"));
    Serial.println(F("  BNO055 Killer — Attitude Indicator"));
    Serial.println(F("  ─────────────────────────────────────────"));

    // Roll bar: -180 .. +180
    printBarH("  Roll  ", e.roll,  -180.0f, 180.0f, 30);
    // Pitch bar: -90 .. +90
    printBarH("  Pitch ", e.pitch,  -90.0f,  90.0f, 30);
    // Yaw bar: -180 .. +180
    printBarH("  Yaw   ", e.yaw,  -180.0f, 180.0f, 30);

    Serial.println(F("  ─────────────────────────────────────────"));

    // Compass rose (24 karakter = 360°, setiap char = 15°)
    int headIdx = (int)(heading / 15.0f) % 24;
    Serial.print(F("  Compass: N"));
    const char* COMPASS = "---E---S---W---N---E---S";
    for (int i = 0; i < 24; i++) {
        Serial.print(i == headIdx ? '^' : COMPASS[i]);
    }
    Serial.printf("N  %.1f deg\r\n", heading);

    Serial.println(F("  ─────────────────────────────────────────"));
    Serial.println(F("  [1]Cube [2]Attitude [3]Processing [4]JSON"));
}

// ============================================================
//  setup()
// ============================================================
void setup() {
    Serial.begin(VIZ_BAUD);
    while (!Serial && millis() < 3000) delay(10);

    // ANSI: clear screen
    Serial.print(F("\033[2J\033[H"));
    Serial.println(F("=== BNO055 Killer — Quaternion Visualizer ==="));
    Serial.println(F("by izzumhdh\n"));

    Wire.begin();
    Wire.setClock(400000UL);

    if (!imu.begin()) {
        Serial.println(F("FATAL: IMU gagal!"));
        while (true) delay(1000);
    }

    Serial.println(F("Komando:"));
    Serial.println(F("  1 — ASCII 3D Cube (perlu terminal ANSI)"));
    Serial.println(F("  2 — Attitude Indicator + Compass"));
    Serial.println(F("  3 — Protocol untuk Processing sketch"));
    Serial.println(F("  4 — JSON untuk web visualizer"));
    Serial.println(F("  g — Kalibrasi gyro\n"));
    delay(2000);
    Serial.print(F("\033[2J"));   // Clear screen sebelum mulai
}

// ============================================================
//  loop()
// ============================================================
uint32_t lastViz = 0;

void loop() {
    imu.update();

    if (Serial.available()) {
        char c = (char)Serial.read();
        if (c >= '1' && c <= '4') {
            vizMode = c - '0';
            Serial.print(F("\033[2J"));   // Clear screen saat ganti mode
        } else if (c == 'g') {
            imu.calibrateGyro();
            imu.saveCalibration();
        }
    }

    if (millis() - lastViz < VIZ_INTERVAL_MS) return;
    lastViz = millis();

    const FusionOutput& out = imu.getOutput();
    if (!out.valid) return;

    switch (vizMode) {
        case 1:
            renderCube(out.quaternion, out.euler);
            break;

        case 2:
            renderAttitude(out.euler, out.heading);
            break;

        case 3:
            // Protocol untuk Processing sketch (lihat komentar di atas)
            Serial.printf("!RPY:%.4f,%.4f,%.4f\r\n",
                          out.euler.roll, out.euler.pitch, out.euler.yaw);
            break;

        case 4:
            // JSON untuk web-based 3D viewer (Three.js / Babylon.js)
            Serial.printf("{\"w\":%.5f,\"x\":%.5f,\"y\":%.5f,\"z\":%.5f,"
                          "\"r\":%.2f,\"p\":%.2f,\"y\":%.2f,\"h\":%.1f}\r\n",
                          out.quaternion.w, out.quaternion.x,
                          out.quaternion.y, out.quaternion.z,
                          out.euler.roll, out.euler.pitch, out.euler.yaw,
                          out.heading);
            break;
    }
}
