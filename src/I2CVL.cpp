#include "I2CVL.hpp"

#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 1
#define I2C_FREQUENCY 400000 
#define VL53L0X_ADDRESS 0x29 // 自定义地址，避免冲突

// 建议直接使用默认的 Wire 实例，或者这样定义：

TwoWire I2C_Bus = TwoWire(0); 
VL53L0X lox;

void beginI2C() {
    // 在 ESP32 中，begin 直接传入引脚和频率是最稳妥的
    if (!I2C_Bus.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY)) {
        Serial.println("Failed to initialize I2C bus!");
        while (1);
    }
    Serial.println("I2C initialized successfully!");

    // I2C_Bus.setClock(400000);

    Serial.println("Scanning I2C...");
    for (byte i = 1; i < 127; i++) {
        I2C_Bus.beginTransmission(i);
        if (I2C_Bus.endTransmission() == 0) {
            Serial.printf("Found device at 0x%02X\n", i);
        }
    }
}
void beginVL53L0X() {
    // 1. 注意：wirePort.begin() 在 ESP32 上如果不传参数，会使用默认引脚
    // 如果你在别处已经调用过带引脚参数的 begin，这里可以省略，或者保持一致

    delay(100);

    Serial.println("1");
    
    lox.setBus(&I2C_Bus); // 必须在 init 之前
    
    if (!lox.init()) {
        Serial.println("Failed to detect and initialize VL53L0X!");
        while (1);
    }

    // 2. 关键点：修改地址必须在 init 之后，startContinuous 之前
    Serial.println("2");
    lox.setAddress(VL53L0X_ADDRESS);

    // 3. 核心补充：必须设置计时预算，否则默认是 33ms，达不到你要求的 20ms 频率
    Serial.println("3");
    lox.setMeasurementTimingBudget(20000); 

    // 4. 启动连续测量
    Serial.println("4");
    lox.startContinuous();
}

int IRAM_ATTR VL53L0X_readDistance() {
    // 读取距离值，单位为毫米
    return static_cast<int>(lox.readRangeContinuousMillimeters());
}