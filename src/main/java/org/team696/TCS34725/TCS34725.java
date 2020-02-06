//This is stolen from 1736 (https://github.com/RobotCasserole1736/CasseroleLib/blob/master/java/src/org/usfirst/frc/team1736/lib/Sensors/TCS34725ColorSensor.java)
package org.team696.TCS34725;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class TCS34725 {
  private I2C i2c;
  private String name;

  // I2C constants
  @SuppressWarnings("unused")
  public static final class I2C_constants {
    public static final int TCS34725_I2C_ADDR =     (0x29);
    private static final int TCS34725_COMMAND_BIT = (0x80);

    private static final int TCS34725_ENABLE =      (0x00);
    private static final int TCS34725_ENABLE_AIEN = (0x10); /* RGBC Interrupt Enable */
    private static final int TCS34725_ENABLE_WEN =  (0x08); /* Wait enable - Writing 1 activates the wait timer */
    private static final int TCS34725_ENABLE_AEN =  (0x02); /* RGBC Enable - Writing 1 actives the ADC, 0 disables it */
    private static final int TCS34725_ENABLE_PON =  (0x01); /*
                                                            * Power on - Writing 1 activates the internal oscillator, 0
                                                            * disables it
                                                            */
    private static final int TCS34725_CONFIG =      (0x0D);
    private static final int TCS34725_CONFIG_WLONG =(0x02); /*
                                                              * Choose between short and long (12x) wait times via
                                                              * TCS34725_WTIME
                                                              */
    private static final int TCS34725_CONTROL =     (0x0F); /* Set the gain level for the sensor */
    private static final int TCS34725_ID =          (0x12); /* 0x44 = TCS34721/TCS34725, 0x4D = TCS34723/TCS34727 */
    private static final int TCS34725_STATUS =      (0x13);
    private static final int TCS34725_STATUS_AINT = (0x10); /* RGBC Clean channel interrupt */
    private static final int TCS34725_STATUS_AVALID=(0x01); /*
                                                               * Indicates that the RGBC channels have completed an
                                                               * integration cycle
                                                               */
    private static final int TCS34725_CDATAL =      (0x14); /* Clear channel data */
    private static final int TCS34725_CDATAH =      (0x15);
    private static final int TCS34725_RDATAL =      (0x16); /* Red channel data */
    private static final int TCS34725_RDATAH =      (0x17);
    private static final int TCS34725_GDATAL =      (0x18); /* Green channel data */
    private static final int TCS34725_GDATAH =      (0x19);
    private static final int TCS34725_BDATAL =      (0x1A); /* Blue channel data */
    private static final int TCS34725_BDATAH =      (0x1B);
    private static final int TCS34725_ATIME =       (0x01); /* Integration time */

    private static final int TCS34725_INTEGRATIONTIME_2_4MS = (0xFF);/** < 2.4ms - 1 cycle - Max Count: 1024 */
    private static final int TCS34725_INTEGRATIONTIME_24MS =  (0xF6);/** < 24ms - 10 cycles - Max Count: 10240 */
    private static final int TCS34725_INTEGRATIONTIME_50MS =  (0xEB);/** < 50ms - 20 cycles - Max Count: 20480 */
    private static final int TCS34725_INTEGRATIONTIME_101MS = (0xD5);/** < 101ms - 42 cycles - Max Count: 43008 */
    private static final int TCS34725_INTEGRATIONTIME_154MS = (0xC0);/** < 154ms - 64 cycles - Max Count: 65535 */
    private static final int TCS34725_INTEGRATIONTIME_700MS = (0x00);/** < 700ms - 256 cycles - Max Count: 65535 */

    private static final int TCS34725_GAIN_1X =   (0x00);/** < No gain */
    private static final int TCS34725_GAIN_4X =   (0x01);/** < 4x gain */
    private static final int TCS34725_GAIN_16X =  (0x02);/** < 16x gain */
    private static final int TCS34725_GAIN_60X =  (0x03);/** < 60x gain */
  }

  // State Variables
  private boolean sensor_initalized;
  /** True if sensor has been initialized, false if not */
  private boolean good_data_read;
  /** True if the last read from the sensor was good, bad if data was corrupted */
  private int red_val;
  private int green_val;
  private int blue_val;
  private int clear_val;

  public TCS34725(I2C i2c) {
    this.i2c = i2c;
    sensor_initalized = false;
    good_data_read = false;
  }

  public TCS34725(Port port) {
    this(new I2C(port, I2C_constants.TCS34725_I2C_ADDR));
  }

  public byte getAddress() {
    return I2C_constants.TCS34725_I2C_ADDR;
  }

  public void setName(String str){
    name = str;
  }

  public String getName(){
    return name;
  }

  /**
   * Initializes the actual sensor state so colors can be read. By default the
   * sensor powers up to a "disabled" state. This enables it. Additionally, checks
   * the sensor has the proper internal ID and sets hard-coded gains and
   * integrator times.
   * 
   * @return 0 on success, -1 on failure to initialize
   */
  public int init(IntegrationTime intTime, Gain gain) {
    sensor_initalized = false;
    // TODO: logging
    //System.out.print("Initalizing Color Sensor...");

    byte[] whoamiResponse = new byte[1];
    whoamiResponse[0] = 0x00;

    // Check we're actually connected to the sensor
    i2c.read(I2C_constants.TCS34725_ID, 1, whoamiResponse);
    if ((whoamiResponse[0] != 0x44) && (whoamiResponse[0] != 0x10)) {
      // TODO: logging
      // System.out.println("\nError - whoami register mismatch on Color Sensor!
      // Cannot Initalize!");
      return -1;
    }

    // Set the integration time
    setIntegrationTime(intTime);

    // Set the gain
    setGain(gain);

    // Power-on the sensor's internals (it defaults to off)
    i2c.write(I2C_constants.TCS34725_ENABLE, I2C_constants.TCS34725_ENABLE_PON);
    switch (intTime){
      case IT_2_4MS:
        safeSleep(3);
        break;
      case IT_24MS:
        safeSleep(25);
        break;
      case IT_50MS:
        safeSleep(51);
        break;
      case IT_101MS:
        safeSleep(102);
        break;
      case IT_154MS:
        safeSleep(155);
        break;
      case IT_700MS:
        safeSleep(701);
        break;
    }
    i2c.write(I2C_constants.TCS34725_ENABLE, I2C_constants.TCS34725_ENABLE_PON | I2C_constants.TCS34725_ENABLE_AEN);

    //Turn on LED
    setLED(true);

    // TODO: logging
    // System.out.println("done!");
    sensor_initalized = true;
    return 0;

  }

  /**
   * Sets sensor integration time
   * @param intTime Integration time (member of enum, not continuously adjustable)
   */
  public void setIntegrationTime(IntegrationTime intTime){
    switch(intTime){
      case IT_2_4MS:
        i2c.write(I2C_constants.TCS34725_ATIME, I2C_constants.TCS34725_INTEGRATIONTIME_2_4MS);
        break;
      case IT_24MS:
        i2c.write(I2C_constants.TCS34725_ATIME, I2C_constants.TCS34725_INTEGRATIONTIME_24MS);
        break;
      case IT_50MS:
        i2c.write(I2C_constants.TCS34725_ATIME, I2C_constants.TCS34725_INTEGRATIONTIME_50MS);
        break;
      case IT_101MS:
        i2c.write(I2C_constants.TCS34725_ATIME, I2C_constants.TCS34725_INTEGRATIONTIME_101MS);
        break;
      case IT_154MS:
        i2c.write(I2C_constants.TCS34725_ATIME, I2C_constants.TCS34725_INTEGRATIONTIME_154MS);
        break;
        case IT_700MS:
        i2c.write(I2C_constants.TCS34725_ATIME, I2C_constants.TCS34725_INTEGRATIONTIME_700MS);
        break;
    }
  }

  /**
   * Sets sensor analog gain
   * @param gain Gain (member of enum, not continuously adjustable)
   */
  public void setGain(Gain gain){
    switch(gain){
      case X1:
        i2c.write(I2C_constants.TCS34725_CONTROL, I2C_constants.TCS34725_GAIN_1X);
        break;
      case X4:
        i2c.write(I2C_constants.TCS34725_CONTROL, I2C_constants.TCS34725_GAIN_4X);
        break;
      case X16:
        i2c.write(I2C_constants.TCS34725_CONTROL, I2C_constants.TCS34725_GAIN_16X);
        break;
      case X60:
        i2c.write(I2C_constants.TCS34725_CONTROL, I2C_constants.TCS34725_GAIN_60X);
        break;
    }
  }


  /**
   * Turns LED on or off 
   * (LED pin must be wired to INT pin)
   * @param state
   */
  public void setLED(boolean state){
    byte[] r = new byte[1];
    i2c.read(I2C_constants.TCS34725_ENABLE, 1, r);
    if (state) {
      r[0] |= I2C_constants.TCS34725_ENABLE_AIEN;
    }
    else {
      r[0] &= ~I2C_constants.TCS34725_ENABLE_AIEN;
    }
    i2c.write(I2C_constants.TCS34725_ENABLE, r[0]);
  }

  /**
   * Queries the sensor for the red, green, blue, and clear values Qualifies the
   * read to ensure the sensor has not been reset since the last read.
   * 
   * @return 0 on read success, -1 on failure.
   */
  public int readColors() {
    byte[] red_bytes = { 0, 0 };
    byte[] green_bytes = { 0, 0 };
    byte[] blue_bytes = { 0, 0 };
    byte[] clear_bytes = { 0, 0 };

    byte[] enable_test_buf = { 0 };

    // Don't bother doing anything if the sensor isn't initialized
    if (!sensor_initalized) {
      // TODO: logging
      // System.out.println("Error: Attempt to read from color sensor, but it's not
      // initalized!");
      return -1;
    }

    // Call the read bad if the enable register isn't set properly
    // (this gets reset to a different value if the sensor is power-cycled)
    i2c.read(I2C_constants.TCS34725_ENABLE, 1, enable_test_buf);
    if (enable_test_buf[0] != (I2C_constants.TCS34725_ENABLE_PON | I2C_constants.TCS34725_ENABLE_AEN)) {
      // TODO: logging
      // System.out.println(
      // "Error: Attempt to read from color sensor, but the enable register did not
      // read as expected! Sensor has probably been reset.");
      sensor_initalized = false;
      good_data_read = false;
      return -1;
    }

    // Read data off of the sensor
    i2c.read(I2C_constants.TCS34725_RDATAH, 2, red_bytes);
    i2c.read(I2C_constants.TCS34725_GDATAH, 2, green_bytes);
    i2c.read(I2C_constants.TCS34725_BDATAH, 2, blue_bytes);
    i2c.read(I2C_constants.TCS34725_CDATAH, 2, clear_bytes);

    // Perform typecasting and bit-shifting on the recieved data.
    red_val = (int) ((red_bytes[1] << 8) | (red_bytes[0] & 0xFF));
    green_val = (int) ((green_bytes[1] << 8) | (green_bytes[0] & 0xFF));
    blue_val = (int) ((blue_bytes[1] << 8) | (blue_bytes[0] & 0xFF));
    clear_val = (int) ((clear_bytes[1] << 8) | (clear_bytes[0] & 0xFF));

    // Set that we've got good data and return.
    good_data_read = true;
    return 0;
  }

  /**
   * Returns the most recent red intensity read from the sensor
   * 
   * @return most recently read red intensity
   */
  public int getRedVal() {
    return red_val;
  }

  /**
   * Returns the most recent green intensity read from the sensor
   * 
   * @return most recently read green intensity
   */
  public int getGreenVal() {
    return green_val;
  }

  /**
   * Returns the most recent blue intensity read from the sensor
   * 
   * @return most recently read blue intensity
   */
  public int getBlueVal() {
    return blue_val;
  }

  /**
   * Returns the most recent overall intensity read from the sensor
   * 
   * @return most recently read overall intensity
   */
  public int getClearVal() {
    return clear_val;
  }

  /**
   * Wrapped Thread.sleep call to safely delay for a given period of time.
   * Presumes there is nothing to do if sleeping is interrupted.
   * 
   * @param milliseconds Time to delay for in ms.
   */
  private void safeSleep(long milliseconds) {
    try {
      Thread.sleep(milliseconds);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }
}
