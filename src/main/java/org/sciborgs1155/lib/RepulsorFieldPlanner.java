package org.sciborgs1155.lib;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.lib.LoggingUtils.*;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.FieldConstants;

/**
 * Reuplsor field planner
 *
 * <p>Taken directly from 6995's code. Big thanks! :D
 */
public class RepulsorFieldPlanner {
  abstract static class Obstacle {
    double strength = 1.0;
    boolean positive = true;

    public Obstacle(double strength, boolean positive) {
      this.strength = strength;
      this.positive = positive;
    }

    /**
     * Finds the Force from this obstacle to a position.
     *
     * @param position The current position.
     * @param target The goal position.
     * @return The force at a certain position.
     */
    public abstract Force getForceAtPosition(Translation2d position, Translation2d target);

    /**
     * @param dist The distance from the position to the obstacle.
     * @return The force magnitude from that position
     */
    protected double distToForceMag(double dist) {
      double forceMag = strength / (0.00001 + Math.pow(dist, 2));
      return forceMag * (positive ? 1 : -1);
    }

    /**
     * @param dist The distance from the position to the obstacle
     * @param falloff The falloff.
     * @return The force magnitude from the position, with subtracted falloff.
     */
    protected double distToForceMag(double dist, double falloff) {
      double original = strength / (0.00001 + Math.pow(dist, 2));
      double falloffMag = strength / (0.00001 + Math.pow(falloff, 2));
      return Math.max(original - falloffMag, 0) * (positive ? 1 : -1);
    }
  }

  static class PointObstacle extends Obstacle {
    Translation2d loc;
    double radius = 0.5;

    public PointObstacle(Translation2d loc, double strength, boolean positive) {
      super(strength, positive);
      this.loc = loc;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      // displacement from obstacle
      double dist = loc.getDistance(position);
      if (dist > 4) {
        return new Force();
      }
      // distance from the position to the outer radius of the target.
      double outwardsMag = distToForceMag(loc.getDistance(position) - radius);

      // initial calculated force; vector from the obstacle to the position.
      Force initial = new Force(outwardsMag, position.minus(loc).getAngle());

      // theta = angle between position->target vector and obstacle->position vector
      Rotation2d theta = target.minus(position).getAngle().minus(position.minus(loc).getAngle());

      // divide magnitude by 2 and multiply by the sign of theta
      double mag = outwardsMag * Math.signum(Math.sin(theta.getRadians() / 2)) / 2;

      return initial
          .rotateBy(Rotation2d.kCCW_90deg) // rotate left 90 degrees
          .div(initial.getNorm()) // normalize
          .times(mag) // set magnitude
          .plus(initial); // add initial force
    }
  }

  static class CircleObstacle extends Obstacle {
    Translation2d loc;
    double radius = 0.5;

    public CircleObstacle(Translation2d loc, double strength, double radius, boolean positive) {
      super(strength, positive);
      this.loc = loc;
      this.radius = radius;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      // displacement from obstacle
      Translation2d targetToLoc = loc.minus(target);

      // 1 meter from loc, direction is away from target
      Translation2d sidewaysCircle = new Translation2d(1, targetToLoc.getAngle()).plus(loc);

      // force magnitude from the sidewaysCircle to the position
      double sidewaysMag = distToForceMag(sidewaysCircle.getDistance(position));

      // force magnitude from the outward radius of the obstacle.
      double outwardsMag = distToForceMag(Math.max(0.01, loc.getDistance(position) - radius));

      // initial force from the obstacle.
      Force initial =
          new Force(
              outwardsMag,
              position.minus(loc).getNorm() > 1e-4
                  ? position.minus(loc).getAngle()
                  : Rotation2d.kZero);

      // flip the sidewaysMag based on which side of the goal-sideways circle the robot is on
      Rotation2d sidewaysTheta =
          target.minus(position).getNorm() > 1e-4
              ? target.minus(position).getAngle().minus(position.minus(sidewaysCircle).getAngle())
              : Rotation2d.kZero;

      // sideways force calculations to go AROUND objects. sine sign is used to figure out which way
      // to go around
      double sideways = sidewaysMag * Math.signum(Math.sin(sidewaysTheta.getRadians()));
      Rotation2d sidewaysAngle = targetToLoc.getAngle().rotateBy(Rotation2d.kCCW_90deg);

      // adds sideways to force for resultant force
      return new Force(sideways, sidewaysAngle).plus(initial);
    }
  }

  static class HorizontalObstacle extends Obstacle {
    double y;

    public HorizontalObstacle(double y, double strength, boolean positive) {
      super(strength, positive);
      this.y = y;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      return new Force(0, distToForceMag(y - position.getY(), 1));
    }
  }

  static class VerticalObstacle extends Obstacle {
    double x;

    public VerticalObstacle(double x, double strength, boolean positive) {
      super(strength, positive);
      this.x = x;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      return new Force(distToForceMag(x - position.getX(), 1), 0);
    }
  }

  public static final double GOAL_STRENGTH = 0.65;

  /** TODO: Update this yearly to define the physcial field for pathing! */
  public static final List<Obstacle> FIELD_OBSTACLES =
      List.of(
          new CircleObstacle(
              new Translation2d(4.49, 4), 0.6, Units.inchesToMeters(65.5 / 2.0), true),
          new CircleObstacle(
              new Translation2d(13.08, 4), 0.6, Units.inchesToMeters(65.5 / 2.0), true));

  public static final List<Obstacle> WALLS =
      List.of(
          new HorizontalObstacle(0.0, 0.5, true),
          new HorizontalObstacle(FieldConstants.WIDTH.in(Meters), 0.5, false),
          new VerticalObstacle(0.0, 0.5, true),
          new VerticalObstacle(FieldConstants.LENGTH.in(Meters), 0.5, false));

  private List<Obstacle> fixedObstacles = new ArrayList<>();
  private Optional<Translation2d> goalOpt = Optional.empty();

  @Logged
  public Pose2d goal() {
    return new Pose2d(goalOpt.orElse(Translation2d.kZero), Rotation2d.kZero);
  }

  // A grid of arrows drawn in AScope
  // private static final int ARROWS_X = RobotBase.isSimulation() ? 40 : 0;
  // private static final int ARROWS_Y = RobotBase.isSimulation() ? 20 : 0;
  // private static final int ARROWS_SIZE = (ARROWS_X + 1) * (ARROWS_Y + 1);
  // private ArrayList<Pose2d> arrows = new ArrayList<>(ARROWS_SIZE);

  private SwerveSample prevSample;

  public RepulsorFieldPlanner() {
    fixedObstacles.addAll(FIELD_OBSTACLES);
    fixedObstacles.addAll(WALLS);
    // for (int i = 0; i < ARROWS_SIZE; i++) {
    //   // arrows.add(new Pose2d());
    // }
    this.prevSample = sample(Translation2d.kZero, Rotation2d.kZero, 0, 0, 0);
  }

  @NotLogged private boolean useGoalInArrows = false;
  @NotLogged private boolean useObstaclesInArrows = true;
  @NotLogged private boolean useWallsInArrows = true;

  // private Pose2d arrowBackstage = new Pose2d(-10, -10, Rotation2d.kZero);

  /** Updates the grid of vectors // */
  // void updateArrows() {
  //   for (int x = 0; x <= ARROWS_X; x++) {
  //     for (int y = 0; y <= ARROWS_Y; y++) {
  //       Translation2d position =
  //           new Translation2d(x * FIELD_LENGTH / ARROWS_X, y * FIELD_WIDTH / ARROWS_Y);
  //       Force force = Force.kZero;
  //       if (useObstaclesInArrows)
  //         force = force.plus(getObstacleForce(position, goal().getTranslation()));
  //       if (useWallsInArrows)
  //         force = force.plus(getWallForce(position, goal().getTranslation()));
  //       if (useGoalInArrows) {
  //         force = force.plus(getGoalForce(position, goal().getTranslation()));
  //       }
  //       if (force.getNorm() < 1e-6) {
  //         arrows.set(x * (ARROWS_Y + 1) + y, arrowBackstage);
  //       } else {
  //         var rotation = force.getAngle();

  //         arrows.set(x * (ARROWS_Y + 1) + y, new Pose2d(position, rotation));
  //       }
  //     }
  //   }
  // }

  /**
   * Force towards the goal.
   *
   * @param curLocation Location of the robot.
   * @param goal Position of the goal.
   * @return The force to the goal.
   */
  Force getGoalForce(Translation2d curLocation, Translation2d goal) {
    var displacement = goal.minus(curLocation);
    if (displacement.getNorm() == 0) {
      return new Force();
    }
    var direction = displacement.getAngle();
    var mag =
        GOAL_STRENGTH * (1 + 1.0 / (0.0001 + displacement.getNorm() * displacement.getNorm()));
    return new Force(mag, direction);
  }

  /**
   * Force from the walls.
   *
   * @param curLocation Location of the robot.
   * @param target Position of the goal.
   * @return The force from the walls.
   */
  Force getWallForce(Translation2d curLocation, Translation2d target) {
    var force = Force.kZero;
    for (Obstacle obs : WALLS) {
      force = force.plus(obs.getForceAtPosition(curLocation, target));
    }
    return force;
  }

  /**
   * Force from obstacles.
   *
   * @param curLocation Location of the robot.
   * @param target Position of the goal.
   * @return The force from the obstacles.
   */
  Force getObstacleForce(Translation2d curLocation, Translation2d target) {
    var force = Force.kZero;
    for (Obstacle obs : FIELD_OBSTACLES) {
      force = force.plus(obs.getForceAtPosition(curLocation, target));
    }
    return force;
  }

  /**
   * Complete force from obstacles, goal and walls.
   *
   * @param curLocation Location of the robot.
   * @param target Position of the goal.
   * @return The total resultant force from field elements.
   */
  Force getForce(Translation2d curLocation, Translation2d target) {
    var goalForce =
        getGoalForce(curLocation, target)
            .plus(getObstacleForce(curLocation, target))
            .plus(getWallForce(curLocation, target));
    return goalForce;
  }

  /**
   * Creates a {@link SwerveSample} from relevant values.
   *
   * @param trans Position of the robot.
   * @param rot Heading of the robot.
   * @param vx x-component of the field relative robot velocity.
   * @param vy y-component of the field relative robot velocity.
   * @param omega Rotational velocity of the robot.
   * @return A Choreo SwerveSample.
   */
  public static SwerveSample sample(
      Translation2d trans, Rotation2d rot, double vx, double vy, double omega) {
    return new SwerveSample(
        0,
        trans.getX(),
        trans.getY(),
        rot.getRadians(),
        vx,
        vy,
        omega,
        0,
        0,
        0,
        new double[4],
        new double[4]);
  }

  /**
   * Changes the goal of the robot.
   *
   * @param goal The new goal of the pathfinder.
   */
  public void setGoal(Translation2d goal) {
    this.goalOpt = Optional.of(goal);
    // updateArrows();
  }

  /**
   * @param pose The current pose of the robot.
   * @param currentSpeeds The current {@link ChassisSpeeds} of the robot.
   * @param maxSpeed The desired maximum speed of the robot.
   * @param useGoal Whether or not to use the given goal.
   * @return A {@link SwerveSample} representing the next desired robot swerve state to get to the
   *     goal. Includes obstacle avoidance.
   */
  public SwerveSample getCmd(
      Pose2d pose, ChassisSpeeds currentSpeeds, double maxSpeed, boolean useGoal) {
    return getCmd(pose, currentSpeeds, maxSpeed, useGoal, pose.getRotation());
  }

  public SwerveSample getCmd(
      Pose2d pose,
      ChassisSpeeds currentSpeeds,
      double maxSpeed,
      boolean useGoal,
      Rotation2d goalRotation) {
    // Distance travelled in one period
    double stepSize_m = maxSpeed * Constants.PERIOD.in(Seconds);

    if (goalOpt.isEmpty()) {
      // Tells the robot to stop moving if there is no goal.
      return sample(pose.getTranslation(), pose.getRotation(), 0, 0, 0);
    } else {
      long startTime = System.nanoTime();

      // sets the goal the position and the error
      Translation2d goal = goalOpt.get();
      Translation2d position = pose.getTranslation();
      Translation2d err = position.minus(goal);

      if (useGoal && err.getNorm() < stepSize_m * 1.5) {
        // Tells the robot to stop moving if it's already there.
        return sample(goal, goalRotation, 0, 0, 0);
      } else {
        // Add in all forces, ternary operator for the useGoal -> getGoalForce
        Force netForce =
            getObstacleForce(position, goal)
                .plus(getWallForce(position, goal))
                .plus(useGoal ? getGoalForce(position, goal) : Force.kZero);

        // Change stepSize_m if we are using goal
        stepSize_m =
            useGoal
                ? Math.min(maxSpeed, maxSpeed * Math.min(err.getNorm() / 2, 1)) * 0.02
                : stepSize_m;

        // Next desired displacement from the max speed and angle of the net force
        Translation2d step = new Translation2d(stepSize_m, netForce.getAngle());

        // Next desired position
        var intermediateGoal = position.plus(step);

        var endTime = System.nanoTime();
        log("/lib/repulsorTimeS", (endTime - startTime));

        // set the previous sample as the current sample
        prevSample =
            sample(intermediateGoal, goalRotation, step.getX() / 0.02, step.getY() / 0.02, 0);
        return prevSample;
      }
    }
  }

  public double pathLength = 0;

  public ArrayList<Translation2d> getTrajectory(
      Translation2d current, Translation2d goalTranslation, double stepSize_m) {
    pathLength = 0;
    // goalTranslation = goalOpt.orElse(goalTranslation);
    ArrayList<Translation2d> traj = new ArrayList<>();
    Translation2d robot = current;
    for (int i = 0; i < 400; i++) {
      var err = robot.minus(goalTranslation);
      if (err.getNorm() < stepSize_m * 1.5) {
        traj.add(goalTranslation);
        break;
      } else {
        var netForce = getForce(robot, goalTranslation);
        if (netForce.getNorm() == 0) {
          break;
        }
        var step = new Translation2d(stepSize_m, netForce.getAngle());
        var intermediateGoal = robot.plus(step);
        traj.add(intermediateGoal);
        pathLength += stepSize_m;
        robot = intermediateGoal;
      }
    }
    return traj;
  }
}
