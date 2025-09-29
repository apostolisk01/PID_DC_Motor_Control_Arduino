// --- HARDWARE AND MOTOR DEFINITIONS ---
#define L298N_ENA_PIN 4  // PWM Speed Control
#define L298N_IN1_PIN 9  // Direction
#define L298N_IN2_PIN 6  // Direction
#define ENCODER_PIN_A 2  // Encoder Interrupt
#define ENCODER_PIN_B 3  // Encoder Interrupt
#define POT_PIN A0       // Potentiometer for Setpoint
#define POT_KP_PIN A3    // Potentiometer for Kp
#define POT_KI_PIN A5    // Potentiometer for Ki
#define POT_KD_PIN A7    // Potentiometer for Kd

// --- BUTTON DEFINITIONS ---
#define BUTTON_MODE_PIN 30 // A SINGLE button to switch modes

// --- MOTOR SPECIFICATIONS ---
const float GEAR_RATIO = 43.8;
const int ENCODER_CYCLES_PER_MOTOR_REVOLUTION = 16;
const float PULSES_PER_OUTPUT_REVOLUTION = (float)ENCODER_CYCLES_PER_MOTOR_REVOLUTION * 4.0 * GEAR_RATIO;

// --- CONTROL MODE STATE MACHINE ---
enum ControlMode {
  POSITION_CONTROL,
  SPEED_CONTROL
};
ControlMode currentMode = POSITION_CONTROL; // Start in Position Control mode

// --- DEBOUNCE AND BUTTON STATE VARIABLES ---
int lastButtonState = HIGH;         // Last stable state of the button
unsigned long lastDebounceTime = 0; // Last time the button output changed
const long DEBOUNCE_DELAY_MS = 50;  // Debounce time in milliseconds

// --- SHARED GLOBAL VARIABLES ---
volatile long pulseCount = 0;
volatile byte lastEncoded = 0;
unsigned long lastComputeTime = 0;
const int COMPUTE_INTERVAL_MS = 50; // PID/PI loop interval

// --- PID/PI CONTROLLER VARIABLES (must be reset on mode change) ---
double pidError = 0.0;
double integralSum = 0.0;
double lastPidError = 0.0;
long lastPulseCountForSpeedCalc = 0;

// --- TUNING GAINS FOR POSITION CONTROL ---
// double pos_Kp = 1.5; // Values are now read from pots
// double pos_Ki = 0.1;
// double pos_Kd = 0.2;

// --- TUNING GAINS FOR SPEED CONTROL ---
// double spd_Kp = 1.2; // Values are now read from pots
// double spd_Ki = 0.5;

// --- MODE-SPECIFIC CONSTANTS ---
const double POS_PID_OUTPUT_DEADBAND = 5.0; // For position control jitter
const int SPD_POT_DEAD_ZONE = 15; // For speed control zero point

// =================================================================
// SETUP FUNCTION
// =================================================================
void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 2000);

  // --- Pin Initializations ---
  pinMode(L298N_ENA_PIN, OUTPUT);
  pinMode(L298N_IN1_PIN, OUTPUT);
  pinMode(L298N_IN2_PIN, OUTPUT);
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  pinMode(BUTTON_MODE_PIN, INPUT_PULLUP); // Initialize the single mode button

  // --- Encoder Interrupts ---
  byte initialPinAState = digitalRead(ENCODER_PIN_A);
  byte initialPinBState = digitalRead(ENCODER_PIN_B);
  lastEncoded = (initialPinAState << 1) | initialPinBState;
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), updateEncoder, CHANGE);

  // --- Initial State ---
  lastComputeTime = millis();
  resetControlState(true); // Initial setup message
}

// =================================================================
// MAIN LOOP
// =================================================================
void loop() {
  handleModeSwitching();

  // Run the logic for the current control mode
  if (currentMode == POSITION_CONTROL) {
    runPositionControl();
  } else { // SPEED_CONTROL
    runSpeedControl();
  }
}

// =================================================================
// MODE SWITCHING AND STATE MANAGEMENT
// =================================================================
void handleModeSwitching() {
  int currentButtonState = digitalRead(BUTTON_MODE_PIN);

  // Check for a press (HIGH to LOW transition) and debounce it
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY_MS) {
      // Toggle the control mode
      currentMode = (currentMode == POSITION_CONTROL) ? SPEED_CONTROL : POSITION_CONTROL;
      resetControlState(false); // Reset controller for the new mode
      lastDebounceTime = millis(); // Update the time of the press
    }
  }
  lastButtonState = currentButtonState;
}

void resetControlState(bool isInitialSetup) {
  // Stop the motor safely
  digitalWrite(L298N_IN1_PIN, LOW);
  digitalWrite(L298N_IN2_PIN, LOW);
  analogWrite(L298N_ENA_PIN, 0);

  // Reset controller state variables
  integralSum = 0.0;
  lastPidError = 0.0;
  pidError = 0.0;
  
  // Reset encoder count to establish a new zero reference for position control
  noInterrupts();
  pulseCount = 0;
  lastPulseCountForSpeedCalc = 0;
  interrupts();

  // Give a small delay for buttons to be released and system to settle
  delay(100); 

  // Print status to Serial Monitor
  Serial.println("\n---------------------------------");
  if (currentMode == POSITION_CONTROL) {
    Serial.println("MODE: POSITION CONTROL ACTIVATED");
    Serial.println("SetpointDeg,ActualWrappedDeg,Error,Kp,Ki,Kd,PWM");
  } else {
    Serial.println("MODE: SPEED CONTROL ACTIVATED");
    Serial.println("SetpointRPM,CurrentRPM,Kp,Ki,Error,PWM");
  }
  Serial.println("---------------------------------");
  
  if(isInitialSetup) {
    Serial.println("System Initialized.");
  }
}

// =================================================================
// POSITION CONTROL LOGIC
// =================================================================
void runPositionControl() {
  unsigned long currentTime = millis();
  if (currentTime - lastComputeTime < COMPUTE_INTERVAL_MS) {
    return; // Run at a fixed interval
  }
  lastComputeTime = currentTime;

  // Setpoint from potentiometer (0-359 degrees)
  int potValue = analogRead(POT_PIN);
  double setpointAngle = map(potValue, 0, 1023, 0, 359);

  // --- CORRECTED GAIN MAPPING ---
  int Kp_potValue = analogRead(POT_KP_PIN);
  double pos_Kp = (double)Kp_potValue / 1023.0 * 3.53;

  int Ki_potValue = analogRead(POT_KI_PIN);
  double pos_Ki = (double)Ki_potValue / 1023.0 * 5.10;

  int Kd_potValue = analogRead(POT_KD_PIN);
  double pos_Kd = (double)Kd_potValue / 1023.0 * 0.61;

  // Get current angle
  long currentPulseCountSnapshot;
  noInterrupts();
  currentPulseCountSnapshot = pulseCount;
  interrupts();
  double currentAngle = ((float)currentPulseCountSnapshot / PULSES_PER_OUTPUT_REVOLUTION) * 360.0;

  // PID calculations
  pidError = setpointAngle - currentAngle;
  while (pidError > 180.0)  pidError -= 360.0;
  while (pidError < -180.0) pidError += 360.0;

  float dt_sec = (float)COMPUTE_INTERVAL_MS / 1000.0;
  integralSum += pidError * dt_sec;
  // Simple constrain for integral windup
  integralSum = constrain(integralSum, -200, 200);

  double derivative = (pidError - lastPidError) / dt_sec;
  lastPidError = pidError;

  double rawPidOutput = (pos_Kp * pidError) + (pos_Ki * integralSum) + (pos_Kd * derivative);

  // Control motor direction and speed
  int pwmMagnitude = constrain(abs((int)round(rawPidOutput)), 0, 255);

  if (abs(rawPidOutput) < POS_PID_OUTPUT_DEADBAND) {
    digitalWrite(L298N_IN1_PIN, LOW);
    digitalWrite(L298N_IN2_PIN, LOW);
    pwmMagnitude = 0;
  } else if (rawPidOutput > 0) {
    digitalWrite(L298N_IN1_PIN, HIGH);
    digitalWrite(L298N_IN2_PIN, LOW);
  } else {
    digitalWrite(L298N_IN1_PIN, LOW);
    digitalWrite(L298N_IN2_PIN, HIGH);
  }
  analogWrite(L298N_ENA_PIN, pwmMagnitude);

  // Logging
  Serial.print(setpointAngle, 0); Serial.print(",");
  double wrappedAngle = fmod(currentAngle, 360.0);
  if (wrappedAngle < 0) wrappedAngle += 360.0;
  Serial.print(wrappedAngle, 2); Serial.print(",");
  Serial.print(pidError, 2); Serial.print(",");
  Serial.print(pos_Kp, 2); Serial.print(",");
  Serial.print(pos_Ki, 2); Serial.print(",");
  Serial.print(pos_Kd, 2); Serial.print(",");
  Serial.println(pwmMagnitude);
}

// =================================================================
// SPEED CONTROL LOGIC
// =================================================================
void runSpeedControl() {
  unsigned long currentTime = millis();
  if (currentTime - lastComputeTime < COMPUTE_INTERVAL_MS) {
    return; // Run at a fixed interval
  }
  float dt_sec = (float)(currentTime - lastComputeTime) / 1000.0;
  lastComputeTime = currentTime;
  
  // Setpoint from potentiometer (-251 to 251 RPM)
  int potValue = analogRead(POT_PIN);
  double setpointRPM;
  if (abs(potValue - 512) < SPD_POT_DEAD_ZONE) {
    setpointRPM = 0.0;
  } else {
    setpointRPM = map(potValue, 0, 1023, -251, 251);
  }

  // --- CORRECTED GAIN MAPPING ---
  int Kp_potValue = analogRead(POT_KP_PIN);
  double spd_Kp = (double)Kp_potValue / 1023.0 * 0.61;

  int Ki_potValue = analogRead(POT_KI_PIN);
  double spd_Ki = (double)Ki_potValue / 1023.0 * 1.83;

  // Calculate current RPM
  long pulsesMoved;
  noInterrupts();
  pulsesMoved = pulseCount - lastPulseCountForSpeedCalc;
  lastPulseCountForSpeedCalc = pulseCount;
  interrupts();
  
  float revolutions = (float)pulsesMoved / PULSES_PER_OUTPUT_REVOLUTION;
  double currentRPM = (revolutions / dt_sec) * 60.0;
  
  // PI calculations
  pidError = setpointRPM - currentRPM;
  integralSum += pidError * dt_sec;
  // Simple constrain for integral windup
  integralSum = constrain(integralSum, -500, 500);
  
  double rawPidOutput = (spd_Kp * pidError) + (spd_Ki * integralSum);

  // Control motor direction and speed
  int pwmMagnitude = constrain(abs((int)round(rawPidOutput)), 0, 255);

  if (setpointRPM == 0.0) {
    pwmMagnitude = 0;
    integralSum = 0; // Reset integral at zero to prevent creep
    digitalWrite(L298N_IN1_PIN, LOW);
    digitalWrite(L298N_IN2_PIN, LOW);
  } else if (rawPidOutput > 0) {
    digitalWrite(L298N_IN1_PIN, HIGH);
    digitalWrite(L298N_IN2_PIN, LOW);
  } else {
    digitalWrite(L298N_IN1_PIN, LOW);
    digitalWrite(L298N_IN2_PIN, HIGH);
  }
  analogWrite(L298N_ENA_PIN, pwmMagnitude);

  // Logging
  Serial.print(setpointRPM, 2); Serial.print(",");
  Serial.print(currentRPM, 2); Serial.print(",");
  Serial.print(spd_Kp, 2); Serial.print(",");
  Serial.print(spd_Ki, 2); Serial.print(",");
  Serial.print(pidError, 2); Serial.print(",");
  Serial.println(pwmMagnitude);
}

// =================================================================
// ENCODER INTERRUPT SERVICE ROUTINE (SHARED)
// =================================================================
void updateEncoder() {
  byte MSB = digitalRead(ENCODER_PIN_A);
  byte LSB = digitalRead(ENCODER_PIN_B);
  byte encoded = (MSB << 1) | LSB;
  byte sum = (lastEncoded << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) pulseCount++;
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) pulseCount--;
  lastEncoded = encoded;
}