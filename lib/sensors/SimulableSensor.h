#ifndef SIMULABLE_SENSOR_H
#define SIMULABLE_SENSOR_H

class SimulableSensor {
protected:
  bool modoSimulacion = false;
  uint16_t rawSimulado = 0;

public:
  void setSimulatedRaw(uint16_t r) {
    rawSimulado = r;
  }

  void disableSimulation() {
    modoSimulacion = false;
  }

  uint16_t getSimulatedRaw() const {
    return rawSimulado;
  }

  bool isSimulationActive() const {
    return modoSimulacion;
  }
};


#endif // SIMULABLE_SENSOR_H
