/*******************************************************
 * joystick control loop
 ******************************************************/
bool joystickLoop(gunCoords & coords) {
  coords.horizontal = map(analogRead(XAXIS), 0, 1024, -10, 10);
  coords.vertical = map(analogRead(YAXIS), 0, 1024, -10, 10);
  return !digitalRead(FIRE);
}
