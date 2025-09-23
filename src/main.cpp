#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// Pines TFT
constexpr uint8_t PIN_TFT_CS  = 15;
constexpr uint8_t PIN_TFT_DC  = 2;
constexpr uint8_t PIN_TFT_RST = 4;

// Pin CS SD (solo para inicialización)
constexpr uint8_t PIN_SD_CS = 13;

// Inicializar TFT
Adafruit_ILI9341 tft(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);

// Configuración de “frames” simulados
constexpr uint8_t NUM_FRAMES_IDLE   = 8;
constexpr uint8_t NUM_FRAMES_LOAD   = 10;
constexpr uint8_t NUM_FRAMES_VORTEX = 12;

// Estados del sistema
enum State {IDLE, LOAD, VORTEX};
State estado = IDLE;

// Variables de simulación del sensor MAP (0.0 a 1.0)
float sensorMAP = 0.0;
float sensorStep = 0.02; // velocidad simulada

// Frame actual para loop de IDLE y VORTEX
uint8_t frameLoop = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("=== Simulación Paso 1: GIFs Dinámicos ===");

  // Inicializar TFT
  tft.begin();
  tft.setRotation(1); // horizontal
  tft.fillScreen(ILI9341_BLACK);

  // Inicializar SD (solo para probar conexión)
  if(!SD.begin(PIN_SD_CS)) {
    Serial.println("Error inicializando SD");
  } else {
    Serial.println("SD inicializada correctamente");
  }
}

void loop() {
  switch(estado){
    case IDLE:
      playGIFLoop(NUM_FRAMES_IDLE, ILI9341_RED);
      sensorMAP += sensorStep/4; // lento para IDLE
      if(sensorMAP > 0.05) estado = LOAD;
      break;

    case LOAD:
      playGIFLoad(NUM_FRAMES_LOAD, sensorMAP);
      sensorMAP += sensorStep; // progresivo
      if(sensorMAP >= 0.8) estado = VORTEX;
      break;

    case VORTEX:
      playGIFLoop(NUM_FRAMES_VORTEX, ILI9341_BLUE);
      sensorMAP = 0.0; // reinicia simulación
      estado = IDLE;
      break;
  }

  delay(100); // pequeño delay para notar cambio de frame
}

// Función para loop constante de frames (IDLE y VORTEX)
void playGIFLoop(uint8_t numFrames, uint16_t baseColor){
  // Cambia ligeramente el color para simular frame distinto
  tft.fillScreen(baseColor + frameLoop*50);
  frameLoop = (frameLoop + 1) % numFrames;
}

// Función LOAD: frame proporcional al sensor
void playGIFLoad(uint8_t numFrames, float sensorRelative){
  uint8_t frameIndex = round(sensorRelative * (numFrames - 1));
  // Limpiar pantalla y dibujar barra proporcional
  tft.fillScreen(ILI9341_BLACK);
  int width = (frameIndex + 1) * (240 / numFrames); // barra horizontal
  tft.fillRect(0, 140, width, 40, ILI9341_YELLOW);
}
