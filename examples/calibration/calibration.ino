/*
 * ============================================================
 *  BNO055 Killer — Semi-Auto Calibration Example
 *  by izzumhdh
 * ============================================================
 * 
 * This program simplifies the calibration process.
 * It will guide you step-by-step and wait for your input 
 * (sending any character in the Serial Monitor) before 
 * proceeding to the next calibration phase.
 * Finally, it saves the data to EEPROM automatically.
 */

#include <Wire.h>
#include <bno055_killer.h>

IMUHandler imu;

// Helper function to block execution until user sends a character
void waitForUserInput() {
    // Clear any pending characters in the buffer
    while (Serial.available()) {
        Serial.read();
    }
    
    // Wait until at least one new character is received
    while (!Serial.available()) {
        delay(10);
    }
    
    // Clear the buffer again
    while (Serial.available()) {
        Serial.read();
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) delay(10);

    Wire.begin();
    Wire.setClock(400000UL);

    if (!imu.begin()) {
        Serial.println("Failed to initialize LSM6DS3. Check wiring!");
        while (1) delay(10);
    }

    Serial.println("\n=========================================");
    Serial.println("     SEMI-AUTO CALIBRATION UTILITY");
    Serial.println("=========================================\n");

    // --------------------------------------------------------
    // PHASE 1: Gyroscope Calibration
    // --------------------------------------------------------
    Serial.println("[STEP 1/2] Gyroscope Calibration (Zero-rate bias)");
    Serial.println("-> Place the sensor/robot PERFECTLY STILL on a flat surface.");
    Serial.println("-> Send ANY character in the Serial Monitor to start...");
    
    Serial.println("\n[PENTING] Tekan 'R' lalu Enter untuk RESET TOTAL EEPROM (Jika Pitch/Roll error).");
    Serial.println("Atau tekan sembarang huruf lain untuk melanjutkan Kalibrasi normal...");
    
    while (!Serial.available()) { delay(10); }
    char cmd = Serial.read();
    if (cmd == 'R' || cmd == 'r') {
        imu.getCalibration().reset();
        imu.saveCalibration();
        Serial.println("=> EEPROM TELAH DIRESET TOTAL KE NOL!");
    }
    
    while (Serial.available()) Serial.read(); // flush

    Serial.println("\nCALIBRATING GYRO... (Do not touch!)");
    
    // This function will block execution for ~1.2 seconds
    imu.calibrateGyro(); 
    // Wait for user before phase 2
    Serial.println("\nGyro calibration complete.");
    imu.saveCalibration();
    Serial.println("Data Gyro Bias berhasil disimpan sementara ke EEPROM!");
    delay(1000); // --------------------------------------------------------
    // PHASE 2: Magnetometer Calibration
    // --------------------------------------------------------
    Serial.println("[STEP 2/2] Magnetometer Calibration (Hard/Soft-Iron)");
    Serial.println("-> Get ready to pick up the sensor/robot and rotate it in ALL DIRECTIONS (Figure-8/Sphere).");
    Serial.println("-> Send ANY character in the Serial Monitor to start a 15-second capture...");
    
    waitForUserInput();

    Serial.println("\nSTART ROTATING NOW! You have 15 seconds...");

    imu.startMagCalibration();
    
    // Loop for exactly 15 seconds to collect compass data
    uint32_t tStart = millis();
    uint32_t lastDot = millis();
    
    while (millis() - tStart < 15000) {
        // MUST be called continuously to fetch samples!
        imu.update(); 
        
        // Print a dot every 1 second as a progress indicator
        if (millis() - lastDot >= 1000) {
            lastDot = millis();
            Serial.print(".");
        }
    }
    
    // Finish matrix computation
    imu.finishMagCalibration();
    Serial.println("\n-> Magnetometer Calibration COMPLETE!\n");

    // --------------------------------------------------------
    // PHASE 3: Save to EEPROM
    // --------------------------------------------------------
    imu.saveCalibration();
    Serial.println("=========================================");
    Serial.println(" SUCCESS! Data saved to EEPROM.");
    Serial.println(" You can now upload your main program.");
    Serial.println("=========================================\n");
}

void loop() {
    // After calibration finishes in setup(), we print the result continuously
    imu.update();

    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 500) {
        lastPrint = millis();
        Serial.print("Current Heading/Yaw: ");
        Serial.print(imu.getYaw());
        Serial.println(" deg");
    }
}
