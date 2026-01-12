package org.sciborgs1155.robot;

import static java.util.Map.entry;

import java.util.Map;

public final class Ports {
  // TODO: Add and change all ports as needed.

  public static final Map<Integer, String> idToName =
      Map.ofEntries(
          entry(Drive.FRONT_LEFT_DRIVE, "FL drive"),
          entry(Drive.REAR_LEFT_DRIVE, "RL drive"),
          entry(Drive.FRONT_RIGHT_DRIVE, "FR drive"),
          entry(Drive.REAR_RIGHT_DRIVE, "RR drive"),
          entry(Drive.FRONT_LEFT_TURNING, "FL turn"),
          entry(Drive.REAR_LEFT_TURNING, "RL turn"),
          entry(Drive.FRONT_RIGHT_TURNING, "FR turn"),
          entry(Drive.REAR_RIGHT_TURNING, "RR turn"),
          // For Talons
          entry(Drive.FRONT_LEFT_CANCODER, "FL cancoder"),
          entry(Drive.REAR_LEFT_CANCODER, "RL cancoder"),
          entry(Drive.FRONT_RIGHT_CANCODER, "FR cancoder"),
          entry(Drive.REAR_RIGHT_CANCODER, "RR cancoder"));

  public static final class OI {
    public static final int OPERATOR = 0;
    public static final int DRIVER = 1;
  }

  public static final class Drive {
    public static final int CANANDGYRO = 20;
    public static final int FRONT_LEFT_DRIVE = 11;
    public static final int REAR_LEFT_DRIVE = 10;
    public static final int FRONT_RIGHT_DRIVE = 12;
    public static final int REAR_RIGHT_DRIVE = 13;

    public static final int FRONT_LEFT_TURNING = 15;
    public static final int REAR_LEFT_TURNING = 14;
    public static final int FRONT_RIGHT_TURNING = 16;
    public static final int REAR_RIGHT_TURNING = 17;

    // For Talons
    public static final int FRONT_LEFT_CANCODER = 5;
    public static final int REAR_LEFT_CANCODER = 7;
    public static final int FRONT_RIGHT_CANCODER = 6;
    public static final int REAR_RIGHT_CANCODER = 8;
  }

  public static final class LEDs {
    public static final int LED_PORT = 9;
  }
}
