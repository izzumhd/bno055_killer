#include <Wire.h>
#include <bno055_killer.h>

// * ============================================================
// *  BNO055 Killer — read yaw, pitch, roll 
// *  using LSM6DS3 + QMC5883L 
// *  by izzumhdh
// * ============================================================

IMUHandler imu;

void setup() {
    Serial.begin(115200);
    
    while (!Serial && millis() < 3000) delay(10);

    Serial.println("Memulai inisialisasi sensor...");

    Wire.begin();
    Wire.setClock(400000UL);
    if (!imu.begin()) {
        Serial.println("Gagal menginisialisasi LSM6DS3. Cek jalur I2C (SDA/SCL)!");
        while (true) {
            delay(10);
        }
    }
    
    Serial.println("\n[PENTING] JANGAN SENTUH ROBOT SELAMA 2 DETIK! Sedang kalibrasi Gyro otomatis...");
    imu.calibrateGyro();
    
    Serial.println("\nSensor berhasil diinisialisasi. Membaca Yaw, Pitch, dan Roll...");
}

void loop() {
    // 1. Panggil update() setiap kali loop berjalan
    // Ini akan membaca data sensor dan memproses filter fusion
    imu.update();

    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 20) {
        lastPrint = millis();

        float yaw   = imu.getYaw();
        float pitch = imu.getPitch();
        float roll  = imu.getRoll();

        Serial.print("Yaw:");
        Serial.print(yaw);
        Serial.print(", Pitch:");
        Serial.print(pitch);
        Serial.print(", Roll:");
        Serial.println(roll);
    }
}
