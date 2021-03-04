# fastservo
PID Servo library for controlling a motor via PWM and encoder

After instantiating the class with required parameters, this class will monitor an encoder position and generate an appropriate PWM signal to drive a motor to given set point using trapezoidal motion profiles.

FastServo::tick(uint32_t t_now) function must be called from main loop as often as possible.
