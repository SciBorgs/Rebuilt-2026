package org.sciborgs1155.robot.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

// TODO: Update every year. Very important.
/** Used to tell multiple LEDStrip to do things in a cleaner way. */
public class LEDs implements AutoCloseable {
  public final LEDStrip leftStrip;
  public final LEDStrip middleStrip;
  public final LEDStrip rightStrip;

  public LEDs(LEDStrip leftStrip, LEDStrip middleStrip, LEDStrip rightStrip) {
    this.leftStrip = leftStrip;
    this.middleStrip = middleStrip;
    this.rightStrip = rightStrip;
  }

  public static LEDs create() {
    LEDStrip leftLED = new LEDStrip(0, 37, false);
    LEDStrip middleLED = new LEDStrip(38, 59, true);
    LEDStrip rightLED = new LEDStrip(60, 97, true);
    return new LEDs(leftLED, middleLED, rightLED);
  }

  /** Sets all LEDStrips to elevator progress gradient. */
  public Command progressGradient(DoubleSupplier percent, BooleanSupplier atGoal) {
    return leftStrip
        .progressGradient(percent, atGoal)
        .alongWith(rightStrip.progressGradient(percent, atGoal));
  }

  /** Blinks all LEDStrips with a given color. */
  public Command blink(Color color) {
    return leftStrip.blink(color).alongWith(middleStrip.blink(color), rightStrip.blink(color));
  }

  /** Sets all LEDStrips to auto. */
  public Command autos() {
    return leftStrip.autos().alongWith(middleStrip.autos(), rightStrip.autos());
  }

  /** Sets all LEDStrips with a given color. */
  public Command solid(Color color) {
    return leftStrip.solid(color).alongWith(middleStrip.solid(color), rightStrip.solid(color));
  }

  /** Sets all the LEDs based on an error. */
  public Command error(DoubleSupplier error, double tolerance) {
    return leftStrip
        .error(error, tolerance)
        .alongWith(middleStrip.error(error, tolerance), rightStrip.error(error, tolerance));
  }

  @Override
  public void close() throws Exception {
    leftStrip.close();
    middleStrip.close();
    rightStrip.close();
  }
}
