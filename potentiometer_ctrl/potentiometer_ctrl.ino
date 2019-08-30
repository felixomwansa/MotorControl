const int BRAKE = 0;
const int CW = 1;
const int CCW = 2;
const int CS_THRESHOLD = 15;

const int MOTOR_A1_PIN = 7;   //Motor control input pins
const int MOTOR_B1_PIN = 8;

const int PWM_MOTOR = 5;      // Motor PWM input pin
const int CURRENT_SENSE = A2; // Current sense pin
const int EN_PIN = A0;        // Enable/Diag pin
int motor_Speed = 150;    //Default motor speed
int motor_State = BRAKE;  // Current motor state
int mot_current = 0;      // Motor current

int potPin = A1;    // select the input pin for the potentiometer
int potValue = 0;  // variable to store the value coming from the sensor
//===============================================================================
//  Initialization
//===============================================================================
void setup() {
  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);
  pinMode(PWM_MOTOR, OUTPUT);
      // Uncomment the next 2 lines to use the Enable pins to enable/disable the device.
      // To monitor for fault conditions instead, they would be defined as inputs  
 // pinMode(EN_PIN, OUTPUT);      
 // digitalWrite(EN_PIN, HIGH);  
  
  Serial.begin(9600);           // Initialize serial monitor
  Serial.println("Enter command:");    // Printout commands
  Serial.println("S = STOP");
  Serial.println("F = FORWARD");
  Serial.println("R = REVERSE");
  Serial.println("C = READ MOTOR CURRENT");
  Serial.println("Pxxx = PWM SPEED (P000 - P254)");
  Serial.println("P? = RETURNS CURRENT PWM SPEED");  
}
//===============================================================================
//  Main
//===============================================================================
void loop() {
  //get potentiometer reading and map to pwm pin
    potValue = analogRead(potPin);
    motor_Speed = map(potValue, 0, 1023, 0, 255);
     
// loop while monitoring the serial port and then jump to DoSerial to
// handle incoming characters and act on them

if (Serial.available()) DoSerial();
}
//===============================================================================
//  Subroutine to handle characters typed via Serial Monitor Window
//===============================================================================
void DoSerial()
{
  int index = 0;
  int pwm_Value = 0;
  int mot1_ADC = 0;
  float mot1_voltage = 0.0;
  
  char ch = Serial.read();  // Read the character we know we have
  Serial.println(ch);       // Echo character typed to show we got it

  // Use Switch/Case statement to handle the different commands
  switch (ch) {
  case 'f':   // Motor FORWARD command
  case 'F':   // This fall-through case statement accepts upper and lower case
    motor_State = CW;
    Motor_Cmd(motor_State, motor_Speed);
    Serial.println("Motor Forward");
    break;

  case 'r':   // Motor REVERSE command
  case 'R':
    motor_State = CCW;
    Motor_Cmd(motor_State, motor_Speed);
    Serial.println("Motor Reverse");
    break;

   case 's':   // Motor STOP command
   case 'S':
    motor_State = BRAKE;
    Motor_Cmd(motor_State, 0);
    Serial.println("Motor Stop");
    break;
      
   case 'c':   // Motor Current command
   case 'C':
    mot1_ADC = analogRead(CURRENT_SENSE);
    mot1_voltage = mot1_ADC * (5.0 / 1024);
    Serial.print("Motor 1 Current: ");
    Serial.print (mot1_voltage * 26*100);
    Serial.println (" mA");
    break;

  case 'p':  // Motor SPEED command
  case 'P':
    // This command is a little trickier.  We are looking for a number from 0-255
    // to follow this command so we can set the PWM speed.  If we see a '?'
    // we will report our current speed setting, otherwise we start collecting chars
    // into the readString array.
    delay(2);  // Give time for more characters to arrive.
    for (int i; i<4; i++) readString[i] = ' ';  // Clear string array
    while (Serial.available())  // Read what we get and put into the string array
    {
      char c = Serial.read();
      readString[index] = c;
      index++;
      delay(2);
    }
    readString[3] = ' '; // Append null to end of string array to make it a valid string
    index = 0;            // Reset our index back to the start of the string
    if (readString[index] == '?')   // ? means report our current speed setting and exit.
    {
      Serial.print("Current PWM Setting: ");
      Serial.println(motor_Speed);
      break;
    }
    pwm_Value = atoi(readString);  // Try to convert string into integer
    // We assume a 0 value is because of a non-valid input and ignore the command
    /*if(pwm_Value!=0) {   
      if (pwm_Value > 255) pwm_Value = 255;     // Cap WPM setting at 255
      Serial.println(pwm_Value);        // Echo what we end up with to confirm we got it
      motor_Speed = pwm_Value;
      Motor_Cmd(motor_State, motor_Speed);
    }*/
    break;
  
  default:
  Motor_Cmd(motor_State, motor_Speed);
    break;
  }
}
void Motor_Cmd(int direct, int pwm)     //Function that writes to the motors
{
  {
    if(direct == CW)    {
      digitalWrite(MOTOR_A1_PIN, LOW); 
      digitalWrite(MOTOR_B1_PIN, HIGH);
    }
    else if(direct == CCW)    {
      digitalWrite(MOTOR_A1_PIN, HIGH);
      digitalWrite(MOTOR_B1_PIN, LOW);      
    }
    else    {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, LOW);            
    }
    analogWrite(PWM_MOTOR, pwm); 
  }
}
