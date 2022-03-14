void setup() {
  // put your setup code here, to run once:
  Wire.begin(); /* join i2c bus (address optional) */
  Serial.begin(9600); /* begin serial for debug */
}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(8); /* begin with device address 8 */
  Wire.write("Hello ESP");   /* sends hello string */
  Wire.endTransmission();    /* stop transmitting */

  Wire.requestFrom(8, 13); /* request & read data of size 13 from slave */
  while (Wire.available()) {
    char c = Wire.read();
    Serial.print(c);
  }
  Serial.println();
  delay(1000);
}
