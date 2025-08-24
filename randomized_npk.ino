#include <SoftwareSerial.h>

// Keep RS485 defs so you don’t need to rewire later (not used in this stub)
#define RS485_RX A2
#define RS485_TX A3
SoftwareSerial mod(RS485_RX, RS485_TX);
#define RS485_DE_RE 4

// Generate one JSON line in the same format as the real sensor
void emitNpkJsonOnce() {
  static uint32_t seed = 12345;
  auto rnd01 = []() -> float {
    // Simple LCG for demo purposes
    static uint32_t s = 2463534242u;
    s = 1664525u * s + 1013904223u;
    return (s >> 8) / 16777216.0f; // ~0..1
  };

  float moisture = 25.0 + rnd01() * 40.0;      // 25–65 %
  float temperature = 18.0 + rnd01() * 12.0;   // 18–30 °C
  uint16_t ec = 300 + (uint16_t)(rnd01() * 900.0); // 300–1200 µS/cm
  float ph = 5.5 + rnd01() * 2.0;              // 5.5–7.5
  uint16_t N = 10 + (uint16_t)(rnd01() * 70.0);
  uint16_t P = 5  + (uint16_t)(rnd01() * 35.0);
  uint16_t K = 10 + (uint16_t)(rnd01() * 50.0);

  Serial.print("{\"moisture\":"); Serial.print(moisture, 1);
  Serial.print(",\"temperature\":"); Serial.print(temperature, 1);
  Serial.print(",\"ec\":"); Serial.print(ec);
  Serial.print(",\"ph\":"); Serial.print(ph, 2);
  Serial.print(",\"N\":"); Serial.print(N);
  Serial.print(",\"P\":"); Serial.print(P);
  Serial.print(",\"K\":"); Serial.print(K);
  Serial.println("}");
}


void setup() {
  // USB serial to Raspberry Pi
  Serial.begin(9600);

  // Not strictly needed for the stub, but leave configured for future
  pinMode(RS485_DE_RE, OUTPUT);
  digitalWrite(RS485_DE_RE, LOW); // listen by default
  mod.begin(4800);

  Serial.println("NPK-only STUB ready (hardcoded JSON).");
  
  emitNpkJsonOnce();
}

void loop() {
}
