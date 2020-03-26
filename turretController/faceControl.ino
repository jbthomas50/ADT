/***************************************************************
 * Serial input is read in bytes, so we have to deserialize it
 **************************************************************/
template <class t>
t deserialize(uint8_t *serialData)
{
  t data;
  memcpy(&data, (void*)serialData, sizeof(t));
  return data;
}

/*******************************************************************************************************
 * get the start frame. make sure we are syncing up with the sender so we recieve correct information
 * TODO: check for correct data in start frame before starting the rest of the recieve process.
 *******************************************************************************************************/
bool getStartFrame()
{
  Serial3.read();
  Serial3.read();
  return true;
}

/******************************************************************
 * empty the buffer
 * TODO: check for end frameto ensure data is received correctly
 *****************************************************************/
void getEndFrame()
{
  while (Serial3.available())
  {
    Serial3.read();
  }
}

/********************************************************
 * Get the coordinates and the command to fire or not
 *******************************************************/
bool getInput(serializedGunCoords & ser_coords)
{
  bool fire = false;
  while (!Serial3.available()){}
  if(getStartFrame())
  {
  // Horizontal angle
  ser_coords.horizontal[0] = Serial3.read();
  ser_coords.horizontal[1] = Serial3.read();
  ser_coords.horizontal[2] = Serial3.read();
  ser_coords.horizontal[3] = Serial3.read();
  // vertical angle
  ser_coords.vertical[0] = Serial3.read();
  ser_coords.vertical[1] = Serial3.read();
  ser_coords.vertical[2] = Serial3.read();
  ser_coords.vertical[3] = Serial3.read();
  // fire or no fire
  fire = Serial3.read();
  // do we want to send # of darts?
  }
  getEndFrame();
  return fire;
}

/*******************************************************
 * facial recognition loop
 ******************************************************/
bool faceLoop(gunCoords & coords) {
  Serial3.write(READY); // send code to begin aiming
  sleep(COMMUNICATION_INTERRUPT);
  Serial.println("getting serial input");
  serializedGunCoords ser_coords;
  bool fire = getInput(ser_coords);
  
  coords.horizontal = deserialize<float>(ser_coords.horizontal);
  coords.vertical = deserialize<float>(ser_coords.vertical);

  return fire;  
}
