package org.sciborgs1155.lib;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class ShiftTracker {
  private static Timer timer = new Timer();
  private static final String RED = "Red";
  private static final String BLUE = "Blue";

  private ShiftTracker() {}

  /** Runs periodically in teleop to update elastic widgets */
  public static void periodic() {
    String firstActive = "R".equals(DriverStation.getGameSpecificMessage()) ? BLUE : RED;
    String secondActive = RED.equals(firstActive) ? BLUE : RED;
    String curr = "???????";
    int timeLeft = 0;

    int[] shifts = {10, 35, 60, 85, 110, 140};
    String[] phases = {"Both", firstActive, secondActive, firstActive, secondActive, "Both"};

    for (int i = 0; i < shifts.length; i++) {
      int currentTime = (int) timer.get();
      if (currentTime < shifts[i]) {
        curr = phases[i];
        timeLeft = shifts[i] - currentTime;
        break;
      }
    }

    SmartDashboard.putString("Active Hub", getHex(curr));
    SmartDashboard.putString("Current Shift", "Time Left: " + timeLeft);
  }

  private static String getHex(String color) {
    switch (color) {
      case RED:
        return Color.kRed.toHexString();
      case BLUE:
        return Color.kBlue.toHexString();
      case "Both":
        return Color.kPurple.toHexString();
      default:
        return "";
    }
  }

  /** Command to start using periodic to update widget */
  public static Command startTracking() {
    return Commands.runOnce(() -> timer.start()).andThen(Commands.run(() -> periodic()));
  }
}
