package org.sciborgs1155.robot.intake;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Current;

public class IntakeConstants {
  /**
   * We still want this for later, just add on more later TODO
   *
   * <p>You probably what the angular to linear scaling factors in here based on the dimensions
   * later
   */
  public static final Current CURRENT_LIMIT = Amps.of(5);

  public static final double INTAKE_POWER = 0.5;
}
