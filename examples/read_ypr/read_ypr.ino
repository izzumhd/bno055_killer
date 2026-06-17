#include <Wire.h>
#include <bno055_killer.h>

// Buat instance dari IMUHandler
IMUHandler imu;

void setup() {
    Serial.begin(115200);
    
    // Tunggu koneksi serial (untuk board berbasis USB CDC seperti STM32/Leonardo)
    while (!Serial && millis() < 3000) delay(10);

    Serial.println("Memulai inisialisasi sensor...");

    // Mulai I2C bus dengan kecepatan 400 kHz
    Wire.begin();
    Wire.setClock(400000UL);

    // Inisialisasi IMU (sudah mencakup load kalibrasi dari EEPROM jika ada)
    if (!imu.begin()) {
        Serial.println("Gagal menginisialisasi LSM6DS3. Cek jalur I2C (SDA/SCL)!");
        while (true) {
            delay(10); // Berhenti di sini jika error
        }
    }
    
    Serial.println("Sensor berhasil diinisialisasi. Membaca Yaw, Pitch, dan Roll...");
}

void loop() {
    // 1. Panggil update() setiap kali loop berjalan
    // Ini akan membaca data sensor dan memproses filter fusion
    imu.update();

    // 2. Tampilkan data setiap 20 ms (50 Hz)
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 20) {
        lastPrint = millis();

        // Mengambil data Yaw, Pitch, Roll secara individual
        float yaw   = imu.getYaw();
        float pitch = imu.getPitch();
        float roll  = imu.getRoll();

        // Format untuk Serial Plotter (Yaw:100.00, Pitch:20.00, Roll:5.00)
        Serial.print("Yaw:");
        Serial.print(yaw);
        Serial.print(", Pitch:");
        Serial.print(pitch);
        Serial.print(", Roll:");
        Serial.println(roll);
    }
}
