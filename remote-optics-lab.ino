#define TEMPERATURE 20                  // temperature in celsius
#define HUMIDITY 50                     // humidity in %
#define SOUND_SPEED_0 331.4             // speed of sound at 0 degrees celsius

const int trigPin = 32;                 // trig input of the HC-SR04
const int echoPin = 33;                 // echo output of the HC-SR04

long duration;                          // to store the duration of the echo pulse
float speed;                            // to store the speed of sound
float distanceCm;                       // to store the distance measured in cm


void setup() {
  Serial.begin(9600);                   // Start the serial communication
  pinMode(trigPin, OUTPUT);             // Set the trigPin as an Output
  pinMode(echoPin, INPUT);              // Set the echoPin as an Input
}


void loop() {
  distanceCm = getDistance();           // Get the distance

  Serial.print("Distance (cm): ");      // Print the distance in the Serial Monitor
  Serial.println(distanceCm);

  delay(1000);                            // Wait 10ms between each measurement
}

float getDistance() {
  digitalWrite(trigPin, LOW);           // Clear the trigPin
  delayMicroseconds(2);                 // Wait for 2 micro seconds
  digitalWrite(trigPin, HIGH);          // Set the trigPin on HIGH state for 10 micro seconds
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);           // Set the trigPin on LOW state

  duration = pulseIn(echoPin, HIGH);    // Read the echoPin, returns the sound wave travel time in microseconds
  // speed of sound in m/s with temperature and humidity compensation
  speed = SOUND_SPEED_0 + (0.606 * TEMPERATURE) + (0.0124 * HUMIDITY); 

  return duration * (speed/10000)/2;    // Calculate and return the distance in cm
}
