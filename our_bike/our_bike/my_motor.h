void DCMotor::setFill(int duty) {
  //setData(SET_PWM_DUTY_CYCLE_DC_MOTOR, instance, duty);
  this->duty = duty;
  if (duty == 0) {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
    return;
  }
  if (duty > 0) {
    analogWrite(in1, 0);
    analogWrite(in2, map(duty, 0, 255, 0, 255));
  } else {
    analogWrite(in2, 0);
    analogWrite(in1, map(-duty, 0, 255, 0, 255));
  }

}