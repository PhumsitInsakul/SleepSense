#include "Arduino.h"
#include "Wire.h"
#include <driver/i2s.h>
#define I2S_WS 15
#define I2S_SD 32
#define I2S_SCK 14
#define I2S_PORT I2S_NUM_0
#define bufferLen 64

int16_t sBuffer[bufferLen];
int counter = 0; // Counter to keep track of mean > 100 occurrences

class BH1750FVI {
public:
    typedef enum eDeviceAddress {
        k_DevAddress_L = 0x23,
        k_DevAddress_H = 0x5C
    } eDeviceAddress_t;

    typedef enum eDeviceMode {
        k_DevModeContHighRes = 0x10,
        k_DevModeContHighRes2 = 0x11,
        k_DevModeContLowRes = 0x13,
        k_DevModeOneTimeHighRes = 0x20,
        k_DevModeOneTimeHighRes2 = 0x21,
        k_DevModeOneTimeLowRes = 0x23
    } eDeviceMode_t;

    typedef enum eDeviceState {
        k_DevStatePowerDown = 0x00,
        k_DevStatePowerUp = 0x01,
        k_DevStateReset = 0x07
    } eDeviceState_t;

    BH1750FVI(eDeviceMode_t DeviceMode);
    BH1750FVI(uint8_t AddressPin, eDeviceAddress_t DeviceAddress, eDeviceMode_t DeviceMode);
    void begin(void);
    uint16_t GetLightIntensity(void);
    void SetMode(eDeviceMode_t DeviceMode);
    void Sleep(void);
    void Reset(void);

private:
    void SetAddress(eDeviceAddress_t DeviceAddress);
    void I2CWrite(uint8_t Data);

    uint8_t m_AddressPin;
    eDeviceAddress_t m_DeviceAddress;
    eDeviceMode_t m_DeviceMode;
    bool m_AddressPinUsed;
};

BH1750FVI::BH1750FVI(eDeviceMode_t DeviceMode) : m_DeviceMode(DeviceMode) {
    m_AddressPinUsed = false;
    m_DeviceAddress = k_DevAddress_L;
}

BH1750FVI::BH1750FVI(uint8_t AddressPin, eDeviceAddress_t DeviceAddress, eDeviceMode_t DeviceMode) : m_AddressPin(AddressPin), m_DeviceAddress(DeviceAddress), m_DeviceMode(DeviceMode) {
    m_AddressPinUsed = true;
}

void BH1750FVI::begin(void) {
    Wire.begin();
    I2CWrite(k_DevStatePowerUp); // Turn it On
    if (m_AddressPinUsed) {
        pinMode(m_AddressPin, OUTPUT); // Set the correct pinmode
        digitalWrite(m_AddressPin, HIGH); // Address to high
        SetAddress(m_DeviceAddress); // Set the address
    }
    SetMode(m_DeviceMode); // Set the mode
}

void BH1750FVI::Sleep(void) {
    I2CWrite(k_DevStatePowerDown); // Turn it off
}

void BH1750FVI::Reset(void) {
    I2CWrite(k_DevStatePowerUp); // Turn it on before we can reset it
    I2CWrite(k_DevStateReset); // Reset
}

void BH1750FVI::SetAddress(eDeviceAddress_t DeviceAddress) {
    if (m_AddressPinUsed) {
        m_DeviceAddress = DeviceAddress;
        switch (m_DeviceAddress) {
        case k_DevAddress_L:
            digitalWrite(m_AddressPin, LOW);
            break;
        case k_DevAddress_H:
            digitalWrite(m_AddressPin, HIGH);
            break;
        default:
            digitalWrite(m_AddressPin, HIGH);
            break;
        }
    }
}

void BH1750FVI::SetMode(eDeviceMode_t DeviceMode) {
    m_DeviceMode = DeviceMode;
    delay(10);
    I2CWrite(m_DeviceMode);
}

uint16_t BH1750FVI::GetLightIntensity(void) {
    uint16_t Value = 0;

    Wire.requestFrom(m_DeviceAddress, 2);
    Value = Wire.read();
    Value <<= 8;
    Value |= Wire.read();

    return Value / 1.2;
}

void BH1750FVI::I2CWrite(uint8_t Data) {
    Wire.beginTransmission(m_DeviceAddress);
    Wire.write(Data);
    Wire.endTransmission();
}

BH1750FVI LightSensor(BH1750FVI::k_DevModeContLowRes);
int ledPin = 26;

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
  LightSensor.begin();

  Serial.println("Setup I2S ...");
  delay(1000);
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);
  delay(500);
    
}

void loop() {
  uint16_t lux = LightSensor.GetLightIntensity();
  size_t bytesIn = 0;
  esp_err_t result = i2s_read(I2S_PORT, &sBuffer, bufferLen, &bytesIn, portMAX_DELAY);

  if (lux <= 20) { // 0-20 lux is considered good quality sleep
      digitalWrite(ledPin, LOW); // Turn on the LED
      Serial.printf("Lux: %d\n", lux);
  } else { // More than 20 lux is considered bad quality sleep
      digitalWrite(ledPin, HIGH); // Turn off the LED
      Serial.printf("Lux: %d - Sleep quality is bad\n", lux);
  }
  //delay(250);

  if (result == ESP_OK)
  {
    int samples_read = bytesIn / 8;
    if (samples_read > 0) {
      float mean = 0; //unsigned long ค่าไม่ติดลบแต่ค่าเป็นล้าน
      for (int i = 0; i < samples_read; ++i) {
        mean += (sBuffer[i]);
      }
      mean /= samples_read;

      //Serial.printf("dBFS: %d\n", static_cast<int>(mean));
      Serial.print(static_cast<int>(mean));
      Serial.print('\n');
      if (mean > 100) {
        counter++; //counter = counter + 1; รันได้ผลดี (counter++ ดูสวยดี)
        Serial.printf("dBFS เกินที่กำหนดจำนวน: %d ครั้ง\n", counter); 
      }
    }
    //delay(100); 
  }
  delay(500); //delay มีผลอย่างมากต่อการรับค่าของเสียง (175 โอเค, >250 พัง)
}

void i2s_install(){
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 44100,
    .bits_per_sample = i2s_bits_per_sample_t(16),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = 0, // default interrupt priority
    .dma_buf_count = 8,
    .dma_buf_len = bufferLen,
    .use_apll = false
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

void i2s_setpin(){
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_set_pin(I2S_PORT, &pin_config);
}

