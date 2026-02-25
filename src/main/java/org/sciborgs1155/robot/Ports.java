package org.sciborgs1155.robot;

import static java.util.Map.entry;

import java.util.Map;

public final class Ports {
  // TODO: Add and change all ports as needed.

  public static final Map<Integer, String> ID_TO_NAME =
      Map.ofEntries(
          // Drive
          entry(Drive.FRONT_LEFT_DRIVE, "FL drive"),
          entry(Drive.REAR_LEFT_DRIVE, "RL drive"),
          entry(Drive.FRONT_RIGHT_DRIVE, "FR drive"),
          entry(Drive.REAR_RIGHT_DRIVE, "RR drive"),
          entry(Drive.FRONT_LEFT_TURNING, "FL turn"),
          entry(Drive.REAR_LEFT_TURNING, "RL turn"),
          entry(Drive.FRONT_RIGHT_TURNING, "FR turn"),
          entry(Drive.REAR_RIGHT_TURNING, "RR turn"),
          entry(Drive.FRONT_LEFT_CANCODER, "FL cancoder"),
          entry(Drive.REAR_LEFT_CANCODER, "RL cancoder"),
          entry(Drive.FRONT_RIGHT_CANCODER, "FR cancoder"),
          entry(Drive.REAR_RIGHT_CANCODER, "RR cancoder"),

          // Intaking and Indexing
          entry(Intake.ROLLERS, "intake extension"),
          entry(Slapdown.EXTENSION, "intake extension"),
          entry(Hopper.MOTOR, "hopper"),
          entry(Indexer.MOTOR, "indexer"),

          // Shooting
          entry(Turret.MOTOR, "turret"),
          entry(Turret.ENCODER_A, "turrent cancoder a (front)"),
          entry(Turret.ENCODER_B, "turret cancoder b (middle)"),
          entry(Hood.MOTOR_PORT, "hood"),
          entry(Hood.CANCODER, "hood cancoder"),
          entry(Shooter.LEADER, "shooter leader (top)"),
          entry(Shooter.FOLLOWER, "shooter follower (bottom)"),

          // Climb
          entry(Climb.LEFT_MOTOR_ID, "climb (left)"),
          entry(Climb.RIGHT_MOTOR_ID, "climb (right)"));

  public static final class OI {
    public static final int OPERATOR = 0;
    public static final int DRIVER = 1;
  }

  public static final class Drive {
    public static final int CANANDGYRO = 50;
    public static final int FRONT_LEFT_DRIVE = 11;
    public static final int REAR_LEFT_DRIVE = 12;
    public static final int FRONT_RIGHT_DRIVE = 13;
    public static final int REAR_RIGHT_DRIVE = 10;

    public static final int FRONT_LEFT_TURNING = 15;
    public static final int REAR_LEFT_TURNING = 16;
    public static final int FRONT_RIGHT_TURNING = 17;
    public static final int REAR_RIGHT_TURNING = 14;

    // For Talons
    public static final int FRONT_LEFT_CANCODER = 5;
    public static final int REAR_LEFT_CANCODER = 6;
    public static final int FRONT_RIGHT_CANCODER = 8;
    public static final int REAR_RIGHT_CANCODER = 7;
  }

  public static final class LEDs {
    public static final int LED_PORT = 9;
  }

  public static final class Intake {
    public static final int ROLLERS = 22;
  }

  public static final class Slapdown {
    public static final int EXTENSION = 20;
  }

  public static final class Hopper {
    public static final int MOTOR = 21;
    public static final int BEAMBREAK = 8;
  }

  public static final class Indexer {
    public static final int MOTOR = 23;
    public static final int BEAMBREAK = 7;
  }

  public static final class Turret {
    public static final int MOTOR = 25;
    public static final int ENCODER_A = 26;
    public static final int ENCODER_B = 27;
  }

  public static final class Hood {
    public static final int MOTOR_PORT = 30;
    public static final int CANCODER = 31;
  }

  public static final class Shooter {
    public static final int LEADER = 28;
    public static final int FOLLOWER = 29;
  }

  public static final class Climb {
    public static final int LEFT_MOTOR_ID = 40;
    public static final int RIGHT_MOTOR_ID = 41;
  }
}
