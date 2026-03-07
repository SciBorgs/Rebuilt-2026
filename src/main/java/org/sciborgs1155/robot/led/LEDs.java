package org.sciborgs1155.robot.led;

import static org.sciborgs1155.robot.led.LEDConstants.LED_LENGTH;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** Used to tell multiple LEDStrip to do things in a cleaner way. */
public class LEDs implements AutoCloseable {
  public final LEDStrip tempStrip;

  /**
   * Creates a new LEDs controller with the specified LED strips.
   *
   * @param strip a temporary LED strip for testing this TODO: actually get leds and try this out
   */
  public LEDs(LEDStrip strip) {
    this.tempStrip = strip;
  }

  /**
   * Creates a new LEDs controller with the default LED strip configuration.
   *
   * @return A new LEDs instance.
   */
  public static LEDs create() {
    LEDStrip tempLED = new LEDStrip(0, LED_LENGTH - 1, false);
    // LEDStrip middleLED = new LEDStrip(38, 59, true);
    // LEDStrip rightLED = new LEDStrip(60, 97, true);
    return new LEDs(tempLED);
  }

  /** Sets all LEDStrips to a progress gradient. */
  public Command progressGradient(DoubleSupplier percent, BooleanSupplier atGoal) {
    return tempStrip.progressGradient(percent, atGoal);
  }

  /** Blinks all LEDStrips with a given color. */
  public Command blink(Color color) {
    return tempStrip.blink(color);
  }

  /** Sets all LEDStrips to auto. */
  public Command autos() {
    return tempStrip.autos();
  }

  /** Sets all LEDStrips with a given color. */
  public Command solid(Color color) {
    return tempStrip.solid(color);
  }

    /** Sets all LEDStrips to scrolling a given color. */
  public Command scroll(Color color) {
    return tempStrip.scrolling(color);
  }

  /** Sets all the LEDs based on an error. */
  public Command error(DoubleSupplier error, double tolerance) {
    return tempStrip.error(error, tolerance);
  }

  @Override
  public void close() throws Exception {
    tempStrip.close();
  }
}
