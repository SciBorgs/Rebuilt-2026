package org.sciborgs1155.robot.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class IndexerConstants {

  public static final Current CURRENT_LIMIT = Amps.of(30);
  public static final double INTAKING_VELOCITY = 1;

  public static final double P = 0.0;
  public static final double I = 0.0;
  public static final double D = 0.0;

  public static final double S = 0.0;
  public static final double V = 0.0;
  public static final double A = 0.0;

  public static final double MAX_VOLTAGE = 9;

  public static final Distance STAGE_ONE_RADIUS = Inches.of(0.5);
  public static final Distance STAGE_TWO_RADIUS = Inches.of(0.75);

  public static final LinearVelocity PASSTHROUGH_SPEED = MetersPerSecond.of(1.5);
}
