//LiquidCrystal lcd(2,3,4,5,6,7);
#include <Servo.h>
#include <LiquidCrystal.h>

Servo esc;  // Servo object to control ESC
LiquidCrystal lcd(3, 2, 4, 5, 6, 7); // LCD object (RS, RW, D4-D7)

// Pin definitions
int escPin = 9;         // PWM signal pin for ESC
int potPin = A1;        // Potentiometer connected to analog pin A1
int trigPin = 12;       // Trigger pin for ultrasonic sensor
int echoPin = 11;       // Echo pin for ultrasonic sensor

// PID parameters
float Kp = 0.080;        // Proportional gain (increased)
float Ki = 0.0190;       // Integral gain (adjust as needed)
float Kd = 09.500;       // Derivative gain

// Variables for PID control
float error = 0, previousError = 0;
float integral = 0;
int currentPWM = 900 ;  // Starting PWM value (mid-range)
int minThrottle = 900;    // Minimum PWM value for ESC
int maxThrottle = 2100;   // Maximum PWM value for ESC

// Moving Average Filter Variables
#define NUM_SAMPLES 7  // Number of samples to average
long samples[NUM_SAMPLES];  // Array to store samples
int sampleIndex = 0;        // Index to track current sample position
long sum = 0;               // Sum of samples for averaging

void setup() {
  esc.attach(escPin);  // Attach ESC to pin 9
  lcd.begin(16, 2);     // Initialize the LCD (16x2 display)
  lcd.print("Initializing...");

  // Set up ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Serial monitor for debugging
  Serial.begin(9600);

  // Set initial ESC speed and delay for initialization
  esc.writeMicroseconds(currentPWM); // Set initial speed
  delay(2000);  // Allow the ESC to initialize
  lcd.clear();

  // Initialize sample array to zero
  for (int i = 0; i < NUM_SAMPLES; i++) {
    samples[i] = 0;
  }
}

void loop() {
  // Ultrasonic sensor reading
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echo time and calculate distance (depth)
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;  // Convert time to distance in cm

  // Add the new distance to the sum for moving average
  sum -= samples[sampleIndex];   // Subtract the oldest sample from the sum
  samples[sampleIndex] = distance; // Store the new sample
  sum += samples[sampleIndex];    // Add the new sample to the sum

  // Move to the next sample index (circular buffer)
  sampleIndex = (sampleIndex + 1) % NUM_SAMPLES;

  // Calculate the average distance
  long avgDistance = sum / NUM_SAMPLES;

  // Read potentiometer value and map it to target depth
  int potValue = analogRead(potPin);
  int targetDepth = map(potValue, 0, 1023, 10, 60); // Map to the depth range (10 - 60 cm)

  // Calculate error for PID (desired - current depth)
  error = avgDistance - targetDepth;

  // PID calculations
  integral += error;                        // Integral term
  integral = constrain(integral, -50, 50);  // Limit integral windup
  float derivative = error - previousError; // Derivative term
  previousError = error;                    // Store error for next cycle

  // Calculate PID output
  float pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Update the PWM based on PID output
  currentPWM += pidOutput;

  // Constrain PWM within throttle limits
  currentPWM = constrain(currentPWM, minThrottle, maxThrottle);

  // Send the PWM signal to the ESC
  esc.writeMicroseconds(currentPWM);

  // Display target depth and current depth on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Target: ");
  lcd.print(targetDepth);
  lcd.setCursor(0, 1);
  lcd.print("Depth: ");
  lcd.print(avgDistance); // Displaying filtered distance

  // Print debug information to the serial monitor
  // Print values for serial plotter (each value separated by space)
  Serial.print(60 - targetDepth);      // Target Depth
  Serial.print(" ");
  Serial.print(60 - avgDistance);      // Current Depth (filtered)
  Serial.print(" ");
  Serial.print(-error);                 // Error
  Serial.print(" ");
  Serial.println(currentPWM / 100.0);   // Current PWM (end the line here)

  delay(50);  // Short delay for PID update rate
}
