#include <iostream>
#include <string>
#include "StateMachine.h"
#include "DebugManager.h"

int main() {
  // Inicializar objetos mínimos
  StateMachine fsm;
  DebugManager debugMgr;

  // FSM arranca en modo DEBUG para pruebas manuales
  fsm.begin(true, nullptr, nullptr);
  fsm.debugForceState(SystemState::DEBUG);

  std::string line;
  std::cout << "\n🌪️  Turbo-Acoustic Simulator [Native Mode]\n";
  std::cout << "Escribe líneas tipo: tps:2.13,map:3.12,turbo:1,iny:0.75\n";
  std::cout << "---------------------------------------------------------\n";

  while (std::getline(std::cin, line)) {
    // Simula entrada serial desde script Python
    String input(line.c_str());  // Arduino String desde std::string
    debugMgr.updateFromLine(input);  // ✅ Compatible en entorno nativo

    // Simular sensores
    float tps = debugMgr.hasOverride(DebugTarget::TPS) ? debugMgr.getValue(DebugTarget::TPS) : 0.0f;
    float map = debugMgr.hasOverride(DebugTarget::MAP) ? debugMgr.getValue(DebugTarget::MAP) : 0.0f;

    // Actualizar FSM
    fsm.update(map, tps, false, false, debugMgr);

    // Mostrar estado resultante
    SystemState estado = fsm.getState();
    std::cout << "🧠 Estado FSM → ";

    switch (estado) {
      case SystemState::OFF:                std::cout << "OFF"; break;
      case SystemState::SIN_CALIBRAR:      std::cout << "SIN_CALIBRAR"; break;
      case SystemState::CALIBRATION:       std::cout << "CALIBRATION"; break;
      case SystemState::IDLE:              std::cout << "IDLE"; break;
      case SystemState::INYECCION_ACUSTICA:std::cout << "INYECCION_ACUSTICA"; break;
      case SystemState::TURBO:             std::cout << "TURBO"; break;
      case SystemState::DESCAYENDO:        std::cout << "DESCAYENDO"; break;
      case SystemState::DEBUG:             std::cout << "DEBUG"; break;
      default:                             std::cout << "DESCONOCIDO"; break;
    }

    std::cout << "\nTPS: " << tps
              << " | MAP: " << map
              << " | INY: " << debugMgr.getLevel()
              << "\n---------------------------------------------------------\n";
  }

  return 0;
}
