// Artemis Global Tracker: User Functions

bool heating_state[2] = {false, false};
bool cutter_fired[2] = {false, false};
int32_t feather_temperature;
bool feather_heating_state = false;
uint32_t feather_pressure;
uint32_t message_counter = 0;


/**
 * Read external LMT85 temperature sensor.
 *
 * @param num which sensor to read
 *
 * @return measured temperature in °C
 */
float extTemp(const HeatingNum num)
{
  int pin = 0;
  if (num == HEATING_DOGTRACKER)
    pin = dogTempPin;
  else
    pin = battTempPin;
  digitalWrite(tempMeasEN, HIGH); // Enable the temperature measurement
  analogReadResolution(14); //Set resolution to 14 bit
  delay(10); // Let the voltage settle
  uint16_t voltage = ((uint16_t)analogRead(pin)) * 1100 * 2 / 16384 * 3/2; // Convert to mV, taking into acount the 2/3 voltage divider
  digitalWrite(tempMeasEN, LOW); // Disable the temperature measurement
  // Transfer function of LMT85 according to datasheet
  float temp = ( 8.194 - sqrt(67.1416 + 4*0.00262*(1324 - (float)voltage)) ) / ( -0.00262*2 ) + 30.;
  return temp;
}

/**
 * Operate heating.
 *
 * @param num which heating to operate
 * @param state new state of heating (true = on, false = off)
 *
 * @return new state
 */
bool operateHeating(const HeatingNum num, const bool state)
{
  int pin = (num == HEATING_DOGTRACKER) ? dogHeatingPin : heatingPin;
  if (state)
    digitalWrite(pin, HIGH);
  else
    digitalWrite(pin, LOW);
  heating_state[num] = state;
  return state;
}

/**
 * Change heating state depending on temperature.
 * 
 * @param temp temperature in 10^-2 °C
 * @param num heating number
 */
void updateHeating(const int16_t temp, const HeatingNum num)
{
  if (temp > myTrackerSettings.HITEMP.the_data) {
    operateHeating(num, false);
  }
  else if (temp  < myTrackerSettings.LOTEMP.the_data) {
    operateHeating(num, true);
  }
}

/**
 * Parse a LoRa message.
 *
 * @param msg message text
 *
 * @return true on success, false on failure
 */
bool parseMessage(const String msg)
{
  if ( (!msg.startsWith(F("#"))) || (!msg.endsWith(F("$"))) || (msg.length() < 2) )
    return false;
  bool is_ack = false;
  int ind_start = 1;
  int len;
  while ( (len = msg.substring(ind_start).indexOf("*")) != -1 ) {
    String item = msg.substring(ind_start, ind_start+len);
    int ind_sep = item.indexOf(F(":"));
    String key = item.substring(0, ind_sep);
    String value = String("");
    if (ind_sep != -1)
      value = item.substring(ind_sep+1, item.length());
    Serial.print(key);
    Serial.print(F(" = "));
    Serial.println(value);
    if (key.equals(F("ACK"))) {
      is_ack = true;
      message_counter = value.toInt();
    }
    else if (key.equals(F("T"))) {
      feather_temperature = value.toInt();
    }
    else if (key.equals(F("HEAT"))) {
      feather_heating_state = value.toInt();
    }
    else if ( (key.startsWith(F("C"))) && (key.length() == 2) ) {
      uint8_t num = key.substring(1).toInt();
      if ( (num >= 1) && (num <= 2) ) {
        cutter_fired[num-1] = value.toInt();
      } else {
        Serial.print(F("parseMessage: invalid cutter number "));
        Serial.println(num);
      }
    }
    else if (key.equals(F("PRES"))) {
      feather_pressure = value.toInt();
      //myTrackerSettings.USERVAL3.the_data = feather_pressure; // write Feather pressure to USERVAL3
    }
    else {
      Serial.print(F("parseMessage: invalid key "));
      Serial.println(key);
    }
    ind_start += len+1;
  }
  return true;
}

/**
 * Generate a LoRa message text.
 *
 * @param cutters whether to include cutter fire command(s)
 *
 * @return string with message text
 */
String createMessage(const bool cutters[2])
{
  String msg = String(F("#"));
  for (auto num=0; num < 2; num++) {
    if (cutters[num]) {
      msg += String(F("C")) + String(num+1) + String(F(":fire*"));
    }
  }
  if (msg.length() == 1)
    msg += String(F("PING*"));
  msg += String("$");
  return msg;
}

/**
 * Send message to Feather via LoRa radio and receive answer.
 *
 * @param msg message text
 *
 * @return true on success, false on failure
 */
bool sendLoraMessage(String msg)
{
  if (lora_initialized) {
    int tries = 0;
    bool success = false;
    do {
      Serial.print(F("RFM96 transmitting packet ... "));
      int state = radio.transmit(msg);
      if (state == RADIOLIB_ERR_NONE) {
        // the packet was successfully transmitted
        Serial.println(F("success!"));
        success = true;
      } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
        Serial.println(F("too long!"));
      } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
      }
    } while ( (!success) && (++tries < 3) );
    delay(1000); // Wait some time for Feather to respond.
    Serial.print(F("RFM96 waiting for incoming transmission ... "));
    String str;
    int state = radio.receive(str);
    if (state == RADIOLIB_ERR_NONE) {
      // packet was successfully received
      Serial.println(F("success!"));
      // print the data of the packet
      Serial.print(F("RFM96 received data:\t"));
      Serial.println(str);
      // print RSSI (Received Signal Strength Indicator)
      // of the last received packet
      Serial.print(F("RFM96 RSSI:\t"));
      Serial.print(radio.getRSSI());
      Serial.println(F(" dBm"));
      // Parse received string.
      parseMessage(str);
    } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
      // timeout occurred while waiting for a packet
      Serial.println(F("timeout!"));
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      Serial.println(F("CRC error!"));
    } else {
      // some other error occurred
      Serial.print(F("failed, code "));
      Serial.println(state);
    }
  }
  return true;
}

/**
 * Operate cutter through LoRa link.
 *
 * @param num cutter number
 *
 * @return true on success, false on failure
 */
bool operateCutter(const uint8_t num)
{
  if (num > 2) {
    Serial.print(F("operateCutter: invalid cutter number "));
    Serial.print(num);
    return false;
  }
  bool cutters[2] = {false, false};
  cutters[num] = true;
  bool success;
  int tries = 0;
  do {
    tries++;
    success = sendLoraMessage(createMessage(cutters));
    success &= (!cutter_fired[num]);
    if (!success) {
      Serial.print("cutting instruction not successful after try ");
      Serial.println(tries);
    }
  } while ( (!success) && (tries < 3) );
  return success;
}

/**
 * Inquire status of Feather via LoRa link.
 *
 * @return true on success, false on failure
 */
bool inquireFeatherStatus()
{
  bool cutters[2] = {false, false};
  return sendLoraMessage(createMessage(cutters));
}


/**
 * Function executed when the Tracker receives a "USERFUNC1" (0x58) message field.
 * Triggers cutter 1 via LoRa.
 */
void USER_FUNC_1()
{
  debugPrintln(F("Executing USERFUNC1..."));
  operateCutter(0);
}

/**
 * Function executed when the Tracker receives a "USERFUNC2" (0x59) message field.
 * Triggers cutter 2 via LoRa.
 */
void USER_FUNC_2()
{
  debugPrintln(F("Executing USERFUNC2..."));
  operateCutter(1);
}

/**
 * Function executed when the Tracker receives a "USERFUNC3" (0x5a) message field.
 * Currently unused.
 */
void USER_FUNC_3()
{
  debugPrintln(F("Executing USERFUNC3..."));
  // Add your own code here - it will be executed when the Tracker receives a "USERFUNC3" (0x5a) message field
}

/**
 * Function executed when the Tracker receives a "USERFUNC4" (0x5b) message field.
 * Currently unused.
 */
void USER_FUNC_4()
{
  debugPrintln(F("Executing USERFUNC4..."));
  // Add your own code here - it will be executed when the Tracker receives a "USERFUNC4" (0x5b) message field
}

/**
 * Function executed when the Tracker receives a "USERFUNC5" (0x5c) message field.
 * Currently unused.
 */
void USER_FUNC_5(uint16_t myVar)
{
  debugPrintln(F("Executing USERFUNC5..."));
  // Add your own code here - it will be executed when the Tracker receives a "USERFUNC5" (0x5c) message field
}

/**
 * Function executed when the Tracker receives a "USERFUNC6" (0x5d) message field.
 * Currently unused.
 */
void USER_FUNC_6(uint16_t myVar)
{
  debugPrintln(F("Executing USERFUNC6..."));
  // Add your own code here - it will be executed when the Tracker receives a "USERFUNC6" (0x5d) message field
}

/**
 * Function executed when the Tracker receives a "USERFUNC7" (0x5e) message field.
 * Currently unused.
 */
void USER_FUNC_7(uint32_t myVar)
{
  debugPrintln(F("Executing USERFUNC7..."));
  // Add your own code here - it will be executed when the Tracker receives a "USERFUNC7" (0x5e) message field
}

/**
 * Function executed when the Tracker receives a "USERFUNC8" (0x5f) message field.
 * Currently unused.
 */
void USER_FUNC_8(uint32_t myVar)
{
  debugPrintln(F("Executing USERFUNC8..."));
  // Add your own code here - it will be executed when the Tracker receives a "USERFUNC8" (0x5f) message field
}

/**
 * Determine "USERVAL1" (0x20) message field.
 * Used for status:
 *   bit 0 (0x01): cutter 1 has fired
 *   bit 1 (0x02): cutter 2 has fired
 *   bit 2 (0x04): heating is currently on
 *   bit 3 (0x08): dogtracker heating is currently on
 *   bit 4 (0x10): Feather heating is currently on
 */
byte USER_VAL_1()
{
  byte retVal = 0; // The return value.
  if (cutter_fired[0])
    retVal |= 0x01;
  if (cutter_fired[1])
    retVal |= 0x02;
  if (heating_state[0])
    retVal |= 0x04;
  if (heating_state[1])
    retVal |= 0x08;
  if (feather_heating_state)
    retVal |= 0x10;
  return (retVal);
}

/**
 * Determine "USERVAL2" (0x21) message field.
 * Feather temperature in °C.
 */
byte USER_VAL_2()
{
  int8_t temp = static_cast<int8_t>((feather_temperature+50)/100); // Round to whole degrees and convert from 10^-2 °C.
  return static_cast<byte>(temp);
}

/**
 * Determine "USERVAL3" (0x22) message field.
 * Determine battery temperature by external sensor.
 *
 * @return battery temperature in 10^-2 °C
 */
int16_t USER_VAL_3()
{
  int16_t temp = static_cast<int16_t>(extTemp(HEATING_BATTERY) * 100.0); // Convert to 10^-2 °C in compatibility to MS8607 and alarm temperature values.
  updateHeating(temp, HEATING_BATTERY);
  return temp;
}

/**
 * Determine "USERVAL4" (0x23) message field.
 * Determine dogtracker temperature by external sensor.
 *
 * @return dogtracker temperature in 10^-2 °C
 */
int16_t USER_VAL_4()
{
  int16_t temp = static_cast<int16_t>(extTemp(HEATING_DOGTRACKER) * 100.0); // Convert to 10^-2 °C in compatibility to MS8607 and alarm temperature values.
  updateHeating(temp, HEATING_DOGTRACKER);
  return temp;
}

/**
 * Determine "USERVAL5" (0x24) message field
 * Return pressure measured by Feather.
 *
 * @return pressure measured by Feather in Pa
 */
uint32_t USER_VAL_5()
{
  return feather_pressure;
}

/**
 * Determine "USERVAL6" (0x25) message field.
 * Message counter from Feather.
 *
 * @return LoRa message counter from Feather
 */
uint32_t USER_VAL_6()
{
  return message_counter;
}

/**
 * Determine "USERVAL7" (0x26) message field.
 * Currently unused.
 */
float USER_VAL_7()
{
  // Add your own code here - e.g. read an external sensor. The return value will be sent as a "USERVAL7" (0x26) message field
  float retVal; // The return value.
  retVal = 7.0; // Set retVal to 7.0 for testing - delete this line if you add your own code
  return (retVal);
}

/**
 * Determine "USERVAL8" (0x27) message field.
 * Currently unused.
 */
float USER_VAL_8()
{
  // Add your own code here - e.g. read an external sensor. The return value will be sent as a "USERVAL8" (0x27) message field
  float retVal; // The return value.
  retVal = 8.0E8; // Set retVal to 8.0E8 for testing - delete this line if you add your own code
  return (retVal);
}

/**
 * User alarm function executed when a PHT alarm is set off.
 *
 * @param alarmType the type of the alarm (bit mask with values defined in Tracker_Message_Fields.h)
 *
 * Used to switch heating on ALARM_LOTEMP / ALARM_HITEMP and operate cutter on LOPRESS.
 */
void ALARM_FUNC(uint8_t alarmType)
{
  debugPrintln(F("Executing ALARM_FUNC ..."));

  // LOTEMP / HITEMP alarms
  if ((alarmType & ALARM_LOTEMP) == ALARM_LOTEMP) { // If LOTEMP limit has been exceeded
    Serial.println(F("LOTEMP alarm, activating heating."));
    operateHeating(HEATING_BATTERY, true);
  }
  if ((alarmType & ALARM_HITEMP) == ALARM_HITEMP) { // If HITEMP limit has been exceeded
    Serial.println(F("HITEMP alarm, deactivating heating."));
    operateHeating(HEATING_BATTERY, false);
  }

  // LOPRESS alarm
  if ( ((alarmType & ALARM_LOPRESS) == ALARM_LOPRESS) && ((myTrackerSettings.FLAGS1 & FLAGS1_LOPRESS) == FLAGS1_LOPRESS) ) { // If it's a LOPRESS alarm and it is enabled
    if (!cutter_fired[0]) {
      debugPrintln(F("Activating cutter 1."));
      USER_FUNC_1();
    }
  }
}

/**
 * Trigger reading external sensors.
 */
void triggerUserMeasurements()
{
  USER_VAL_3(); // Measure battery temperature via external sensor and operate its heating.
  USER_VAL_4(); // Measure dog tracker temperature via external sensor and operate its heating.
  inquireFeatherStatus(); // Inquire status of Feather through LoRa link and update variables.
}
