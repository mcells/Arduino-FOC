#include "FOCMotor.h"
#include "../../communication/SimpleFOCDebug.h"

/**
 * Default constructor - setting all variabels to default values
 */
FOCMotor::FOCMotor()
{
  // maximum angular velocity to be used for positioning 
  velocity_limit = DEF_VEL_LIM;
  // maximum voltage to be set to the motor
  voltage_limit = DEF_POWER_SUPPLY;
  // not set on the begining
  current_limit = DEF_CURRENT_LIM;

  // index search velocity
  velocity_index_search = DEF_INDEX_SEARCH_TARGET_VELOCITY;
  // sensor and motor align voltage
  voltage_sensor_align = DEF_VOLTAGE_SENSOR_ALIGN;

  // default modulation is SinePWM
  foc_modulation = FOCModulationType::SinePWM;

  // default target value
  target = 0;
  voltage.d = 0;
  voltage.q = 0;
  // current target values
  current_sp = 0;
  current.q = 0;
  current.d = 0;

  // voltage bemf 
  voltage_bemf = 0;

  // Initialize phase voltages U alpha and U beta used for inverse Park and Clarke transform
  Ualpha = 0;
  Ubeta = 0;
  
  //monitor_port 
  monitor_port = nullptr;
  //sensor 
  sensor_offset = 0.0f;
  sensor = nullptr;
  //current sensor 
  current_sense = nullptr;
}


/**
	Sensor linking method
*/
void FOCMotor::linkSensor(Sensor* _sensor) {
  sensor = _sensor;
}

/**
	CurrentSense linking method
*/
void FOCMotor::linkCurrentSense(CurrentSense* _current_sense) {
  current_sense = _current_sense;
}

// shaft angle calculation
float FOCMotor::shaftAngle() {
  // if no sensor linked return previous value ( for open loop )
  if(!sensor) return shaft_angle;
  return sensor_direction*LPF_angle(sensor->getAngle()) - sensor_offset;
}
// shaft velocity calculation
float FOCMotor::shaftVelocity() {
  // if no sensor linked return previous value ( for open loop )
  if(!sensor) return shaft_velocity;
  return sensor_direction*LPF_velocity(sensor->getVelocity());
}

float FOCMotor::electricalAngle(){
  // if no sensor linked return previous value ( for open loop )
  if(!sensor) return electrical_angle;
  return  _normalizeAngle( (float)(sensor_direction * pole_pairs) * sensor->getMechanicalAngle()  - zero_electric_angle );
}

// Measure resistance and inductance of a motor
int FOCMotor::characteriseMotor(float voltage){
    if (!this->current_sense || !this->current_sense->initialized)
    {
      SIMPLEFOC_DEBUG("ERR: MOT: Cannot characterise motor: CS unconfigured or not initialized");
      return 1;
    }

    if (voltage <= 0.0f){
      SIMPLEFOC_DEBUG("ERR: MOT: Cannot characterise motor: Voltage is negative or less than zero");
      return 2;
    }
    voltage = _constrain(voltage, 0.0f, voltage_limit);
    
    float current_electric_angle = electricalAngle();

    float correction_factor = 1.5f; // 1.5 for 3 phase motors, because we measure over a series-parallel connection. TODO: what about 2 phase motors?
    
    SIMPLEFOC_DEBUG("MOT: Measuring phase to phase resistance, keep motor still...");

    // Apply zero volts
    setPhaseVoltage(0, 0, current_electric_angle);
    _delay(500);
    
    PhaseCurrent_s zerocurrent_raw = current_sense->readAverageCurrents();
    DQCurrent_s zerocurrent = current_sense->getDQCurrents(current_sense->getABCurrents(zerocurrent_raw), current_electric_angle);


    // Ramp and hold the voltage to measure resistance
    // 300 ms of ramping
    current_electric_angle = electricalAngle();
    for(int i=0; i < 100; i++){
        setPhaseVoltage(0, voltage/100.0*((float)i), current_electric_angle);
        _delay(3);
    }
    _delay(10);
    PhaseCurrent_s r_currents_raw = current_sense->readAverageCurrents();
    DQCurrent_s r_currents = current_sense->getDQCurrents(current_sense->getABCurrents(r_currents_raw), current_electric_angle);
    
    setPhaseVoltage(0, 0, current_electric_angle);
    
    float resistance = voltage / (correction_factor * (r_currents.d - zerocurrent.d));
    SIMPLEFOC_DEBUG("MOT: Estimated phase to phase resistance: ", 2.0f * resistance);
    _delay(100);

    //start inductance measurement
    SIMPLEFOC_DEBUG("MOT: Measuring inductance, keep motor still...");

    unsigned long t0 = 0;
    unsigned long t1a = 0;
    unsigned long t1b = 0;
    float Ltemp = 0;
    float Ld = 0;
    float Lq = 0;

    uint iterations = 40;
    uint cycles = 3;
    uint risetime_us = 200; // short for worst case scenario with low inductance
    uint settle_us = 100000; // long for worst case scenario with high inductance

    for (size_t axis = 0; axis < 2; axis++)
    {
      for (size_t i = 0; i < iterations; i++)
      {
        // current_electric_angle = i * _2PI / iterations;
        float inductanceq = 0.0f;
        float inductanced = 0.0f;
        
        float qcurrent = 0.0f;
        float dcurrent = 0.0f;
        for (size_t j = 0; j < cycles; j++)
        {
          // current_electric_angle = electricalAngle();
          // read zero current
          zerocurrent_raw = current_sense->readAverageCurrents(10);
          zerocurrent = current_sense->getDQCurrents(current_sense->getABCurrents(zerocurrent_raw), current_electric_angle);
          
          // step the voltage
          setPhaseVoltage(0, voltage, current_electric_angle);
          t0 = micros();
          delayMicroseconds(risetime_us);

          t1a = micros();
          PhaseCurrent_s l_currents_raw = current_sense->getPhaseCurrents();
          t1b = micros();
          setPhaseVoltage(0, 0, current_electric_angle);

          DQCurrent_s l_currents = current_sense->getDQCurrents(current_sense->getABCurrents(l_currents_raw), current_electric_angle);
          delayMicroseconds(settle_us); // wait a bit for the currents to go to 0 again

          if (t0 > t1a || t0 > t1b) continue; // safety first

          // calculate the inductance
          float dt = 0.5f*(t1a + t1b - 2*t0)/1000000.0f;
          inductanced += fabs(- (resistance * dt) / log((voltage - resistance * fabs(l_currents.d - zerocurrent.d)) / voltage))/correction_factor;
          qcurrent+= l_currents.q - zerocurrent.q;
          dcurrent+= l_currents.d - zerocurrent.d;
        }
        qcurrent /= cycles;
        dcurrent /= cycles;
        float delta = qcurrent / (fabsf(dcurrent) + fabsf(qcurrent));


        inductanced /= cycles;
        float lastD = Ltemp;
        Ltemp = i == 0 ? inductanced : Ltemp * 0.8 + inductanced * 0.2;
        
        // SIMPLEFOC_DEBUG("MOT: Estimated D-inductance in mH: ", L * 1000.0f);
        
        float timeconstant = fabs(Ltemp / resistance); // Timeconstant of an RL circuit (L/R) 
        risetime_us = _constrain(1000000 * timeconstant, 100, 10000); // Wait as long as possible (due to limited timing accuracy & sample rate), but as short as needed (while the current still changes)
        settle_us = 10 * risetime_us;
        SIMPLEFOC_DEBUG("MOT: Estimated time constant in us: ", timeconstant * 1000000.0f);

        Serial.printf(">inductance:%f:%f|xy\n", current_electric_angle, Ltemp * 1000.0f);

        if (axis)
        {
          qcurrent *= -1.0f;
        }
        
        if (qcurrent < 0)
        {
          current_electric_angle+=fabsf(delta);
        } else
        {
          current_electric_angle-=fabsf(delta);
        }
        current_electric_angle = fmodf(current_electric_angle, _2PI);
        if(current_electric_angle < 0.0f) current_electric_angle += _2PI;
       
      }

      current_electric_angle += 0.5f * _PI;
      current_electric_angle = fmodf(current_electric_angle, _2PI);
      if(current_electric_angle < 0.0f) current_electric_angle += _2PI;
      iterations /= 2;

      if (axis == 0)
      {
          Lq = Ltemp;
      }else
      {
        Ld = Ltemp;
      }
      
    }
    
    

    SIMPLEFOC_DEBUG("MOT: Inductance measurement complete!");
    SIMPLEFOC_DEBUG("MOT: Measured D-inductance in mH: ", Ld * 1000.0f);
    SIMPLEFOC_DEBUG("MOT: Measured Q-inductance in mH: ", Lq * 1000.0f);

    return 0;
    
}

/**
 *  Monitoring functions
 */
// function implementing the monitor_port setter
void FOCMotor::useMonitoring(Print &print){
  monitor_port = &print; //operate on the address of print
  #ifndef SIMPLEFOC_DISABLE_DEBUG
  SimpleFOCDebug::enable(&print);
  SIMPLEFOC_DEBUG("MOT: Monitor enabled!");
  #endif
}

// utility function intended to be used with serial plotter to monitor motor variables
// significantly slowing the execution down!!!!
void FOCMotor::monitor() {
  if( !monitor_downsample || monitor_cnt++ < (monitor_downsample-1) ) return;
  monitor_cnt = 0;
  if(!monitor_port) return;
  bool printed = 0;

  if(monitor_variables & _MON_TARGET){
    if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
    monitor_port->print(target,monitor_decimals);    
    printed= true;
  }
  if(monitor_variables & _MON_VOLT_Q) {
    if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
    else if(printed) monitor_port->print(monitor_separator);
    monitor_port->print(voltage.q,monitor_decimals);
    printed= true;
  }
  if(monitor_variables & _MON_VOLT_D) {
    if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
    else if(printed) monitor_port->print(monitor_separator);
    monitor_port->print(voltage.d,monitor_decimals);
    printed= true;
  }
  // read currents if possible - even in voltage mode (if current_sense available)
  if(monitor_variables & _MON_CURR_Q || monitor_variables & _MON_CURR_D) {
    DQCurrent_s c = current;
    if( current_sense && torque_controller != TorqueControlType::foc_current ){
      c = current_sense->getFOCCurrents(electrical_angle);
      c.q = LPF_current_q(c.q);
      c.d = LPF_current_d(c.d);
    }
    if(monitor_variables & _MON_CURR_Q) {
      if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
      else if(printed) monitor_port->print(monitor_separator);
      monitor_port->print(c.q*1000, monitor_decimals); // mAmps
      printed= true;
    }
    if(monitor_variables & _MON_CURR_D) {
      if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
      else if(printed) monitor_port->print(monitor_separator);
      monitor_port->print(c.d*1000, monitor_decimals); // mAmps
      printed= true;
    }
  }
 
  if(monitor_variables & _MON_VEL) {
    if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
    else if(printed) monitor_port->print(monitor_separator);
    monitor_port->print(shaft_velocity,monitor_decimals);
    printed= true;
  }
  if(monitor_variables & _MON_ANGLE) {
    if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
    else if(printed) monitor_port->print(monitor_separator);
    monitor_port->print(shaft_angle,monitor_decimals);
    printed= true;
  }
  if(printed){
    if(monitor_end_char) monitor_port->println(monitor_end_char);
    else monitor_port->println("");
  }
}   

