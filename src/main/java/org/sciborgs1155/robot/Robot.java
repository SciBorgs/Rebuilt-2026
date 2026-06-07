package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.disabled;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.teleop;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.test;
import static org.sciborgs1155.lib.LoggingUtils.log;
import static org.sciborgs1155.robot.Constants.*;
import static org.sciborgs1155.robot.drive.DriveConstants.MAX_ANGULAR_ACCEL;
import static org.sciborgs1155.robot.drive.DriveConstants.MAX_SPEED;
import static org.sciborgs1155.robot.drive.DriveConstants.TELEOP_ANGULAR_SPEED;
import static org.sciborgs1155.robot.shooter.ShooterConstants.CENTER_TO_SHOOTER;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.Arrays;
import org.littletonrobotics.urcl.URCL;
import org.sciborgs1155.lib.CommandRobot;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.ShiftTracker;
import org.sciborgs1155.lib.Tracer;
import org.sciborgs1155.robot.Constants.ShootingData;
import org.sciborgs1155.robot.Ports.OI;
import org.sciborgs1155.robot.climb.Climb;
import org.sciborgs1155.robot.commands.Alignment;
import org.sciborgs1155.robot.commands.Autos;
import org.sciborgs1155.robot.commands.Shooting;
import org.sciborgs1155.robot.commands.shooting.FuelVisualizer;
import org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer;
import org.sciborgs1155.robot.commands.shooting.ShootingAlgorithm;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants;
import org.sciborgs1155.robot.hood.Hood;
import org.sciborgs1155.robot.hopper.Hopper;
import org.sciborgs1155.robot.indexer.Indexer;
import org.sciborgs1155.robot.intake.Intake;
import org.sciborgs1155.robot.led.LEDs;
import org.sciborgs1155.robot.shooter.Shooter;
import org.sciborgs1155.robot.shooter.ShooterConstants;
import org.sciborgs1155.robot.slapdown.Slapdown;
import org.sciborgs1155.robot.turret.Turret;
import org.sciborgs1155.robot.vision.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged
public class Robot extends CommandRobot {
  // INPUT DEVICES
  private final CommandXboxController operator = new CommandXboxController(OI.OPERATOR);
  private final CommandXboxController driver = new CommandXboxController(OI.DRIVER);

  @NotLogged private final PowerDistribution pdh = new PowerDistribution();

  // SUBSYSTEMS
  private final Drive drive = Drive.create();
  private final Vision vision = Vision.create();
  private final Intake intake = Intake.create();
  private final Turret turret = Turret.create();
  private final Hood hood = Hood.create();
  private final Shooter shooter = Shooter.create();
  private final Indexer indexer = Indexer.create();
  private final Hopper hopper = Hopper.create();
  private final Slapdown slapdown = Slapdown.create();
  private final Climb climb = Climb.none();
  private final LEDs leds = LEDs.create();

  // COMMANDS
  private final Alignment align = new Alignment(drive);

  @NotLogged
  private final ProjectileVisualizer fuelVisualizer =
      isReal()
          ? null
          : new FuelVisualizer(
                  ShootingAlgorithm.toShotVelocitySupplier(
                      () -> shooter.velocity() * ShooterConstants.RADIUS.in(Meters),
                      () -> Math.PI / 2 - hood.angle(),
                      () -> turret.position(),
                      drive::pose3d),
                  () -> drive.pose3d().plus(CENTER_TO_SHOOTER),
                  drive::fieldRelativeChassisSpeeds)
              .configPhysics(true, true, false, false)
              .configGeneration(.5, 80, 60)
              .config(true, true);

  private final Shooting shooting =
      new Shooting(shooter, turret, hood, drive, hopper, indexer, slapdown, fuelVisualizer);

  @NotLogged
  private final SendableChooser<Command> autos =
      Autos.configureAutos(drive, intake, slapdown, shooting, climb, align);

  @Logged private double speedMultiplier = FULL_SPEED_MULTIPLIER;
  private double deadband = DEADBAND;

  /** The robot contains subsystems, OI devices, and commands. */
  public Robot() {
    super(PERIOD.in(Seconds));
    configureGameBehavior();
    configureBindings();

    // Warms up pathfinding commands, as the first run could have significant delays.
    CommandScheduler.getInstance().schedule(align.warmupCommand());
  }

  @Override
  public void robotPeriodic() {
    Tracer.startTrace("commands");
    CommandScheduler.getInstance().run();
    Tracer.endTrace();
  }

  /** Configures basic behavior for different periods during the game. */
  private void configureGameBehavior() {
    // TODO: Add configs for all additional libraries, components, intersubsystem interaction
    // Configure logging with DataLogManager, Epilogue, and FaultLogger
    DataLogManager.start();
    SignalLogger.enableAutoLogging(true);
    addPeriodic(FaultLogger::update, 2);
    Epilogue.bind(this);

    // FaultLogger.register(pdh);
    SmartDashboard.putData("Auto Chooser", autos);

    if (TUNING) {
      addPeriodic(
          () ->
              log(
                  "/Robot/camera transforms",
                  Arrays.stream(vision.cameraTransforms())
                      .map(
                          t ->
                              new Pose3d(
                                  drive
                                      .pose3d()
                                      .getTranslation()
                                      .plus(
                                          t.getTranslation()
                                              .rotateBy(drive.pose3d().getRotation())),
                                  t.getRotation().plus(drive.pose3d().getRotation())))
                      .toArray(Pose3d[]::new),
                  Pose3d.struct),
          PERIOD.in(Seconds));
    }

    // Configure pose estimation updates every tick
    addPeriodic(
        () ->
            drive.updateEstimates(
                vision.estimatedGlobalPoses(drive.gyroHeading(), disabled().getAsBoolean())),
        PERIOD);

    RobotController.setBrownoutVoltage(6.0);

    if (isReal()) {
      URCL.start();
      pdh.clearStickyFaults();
      pdh.setSwitchableChannel(true);
    } else {
      DriverStation.silenceJoystickConnectionWarning(true);
      addPeriodic(fuelVisualizer::updateLogging, PERIOD);
      addPeriodic(fuelVisualizer::updateLaunchSimulation, ProjectileVisualizer.LAUNCH_PERIOD);
      addPeriodic(
          fuelVisualizer::updateTrajectorySimulation, ProjectileVisualizer.TRAJECTORY_PERIOD);
    }
  }

  /** Configures trigger -> command bindings. */
  private void configureBindings() {
    teleop().onTrue(ShiftTracker.startTracking());

    // x and y are switched: we use joystick Y axis to control field x motion
    InputStream rawX = InputStream.of(driver::getLeftY).log("/Robot/raw x").negate();
    InputStream rawY = InputStream.of(driver::getLeftX).log("/Robot/raw y").negate();

    // Apply speed multiplier, deadband, square inputs, and scale translation to max speed
    InputStream r =
        InputStream.hypot(rawX, rawY)
            .log("/Robot/raw joystick")
            .scale(() -> speedMultiplier)
            .clamp(1.0)
            .deadband(() -> DEADBAND, 1.0)
            .signedPow(2.0)
            .log("/Robot/processed joystick")
            .scale(MAX_SPEED.in(MetersPerSecond))
            .rateLimit(DriveConstants.MAX_ACCEL.in(MetersPerSecondPerSecond));

    InputStream theta = InputStream.atan(rawX, rawY);

    // Split x and y components of translation input
    InputStream x =
        r.scale(theta.map(Math::cos))
            .log("/Robot/final x"); // .rateLimit(MAX_ACCEL.in(MetersPerSecondPerSecond));
    InputStream y =
        r.scale(theta.map(Math::sin))
            .log("/Robot/final y"); // .rateLimit(MAX_ACCEL.in(MetersPerSecondPerSecond));

    // Apply speed multiplier, deadband, square inputs, and scale rotation to max teleop speed
    InputStream omega =
        InputStream.of(driver::getRightX)
            .negate()
            .scale(() -> speedMultiplier)
            .clamp(1.0)
            .deadband(() -> DEADBAND, 1.0)
            .signedPow(2.0)
            .scale(TELEOP_ANGULAR_SPEED.in(RadiansPerSecond))
            .rateLimit(MAX_ANGULAR_ACCEL.in(RadiansPerSecond.per(Second)));

    drive.setDefaultCommand(drive.drive(x, y, omega).withName("joysticks"));

    if (TUNING) {
      SignalLogger.enableAutoLogging(false);

      // manual .start() call is blocking, for up to 100ms
      teleop().onTrue(Commands.runOnce(() -> SignalLogger.start()));
      disabled().onTrue(Commands.runOnce(() -> SignalLogger.stop()));
    }

    autonomous()
        .whileTrue(
            Commands.deferredProxy(autos::getSelected).asProxy()); // .alongWith(leds.autos()));

    test().whileTrue(systemsCheck());

    driver.povUp().whileTrue(drive.zeroHeading());

    operator
        .rightBumper()
        .and(operator.leftBumper())
        .onTrue(
            Commands.runOnce(() -> speedMultiplier = SLOW_SPEED_MULTIPLIER)
                .alongWith(Commands.runOnce(() -> deadband = SLOW_DEADBAND)))
        .onFalse(
            Commands.runOnce(() -> speedMultiplier = FULL_SPEED_MULTIPLIER)
                .alongWith(Commands.runOnce(() -> deadband = DEADBAND)));

    // INTAKE TOGGLE
    driver.leftTrigger().whileTrue(intake.intake());

    driver
        .x()
        .or(driver.povDown())
        // .or(operator.povDown())
        .whileTrue(slapdown.squeeze())
        .onFalse(slapdown.extend()); // jank jank jank

    driver
        .povRight()
        // .or(operator.povRight())
        .whileTrue(slapdown.retract())
        .onFalse(slapdown.extend()); // jank jank jank

    // driver.povLeft().or(operator.povLeft()).whileTrue(slapdown.extend());

    // OUTTAKE THE INTAKE
    driver
        .a()
        .whileTrue(intake.outtake().alongWith(hopper.outtake()).alongWith(indexer.backward()));

    operator
        .a()
        .whileTrue(shooting.shootDriving(Shooting.LEFT_FEED, x, y, omega).withName("left feed"));

    // FEED CONTINUOUS (LEFT SIDE)
    driver
        .leftBumper()
        .whileTrue(shooting.shootDriving(Shooting.LEFT_FEED, x, y, omega).withName("left feed"));

    // FEED CONTINUOUS (RIGHT SIDE)
    driver
        .rightBumper()
        .whileTrue(shooting.shootDriving(Shooting.RIGHT_FEED, x, y, omega).withName("right feed"));

    // SCORE CONTINUOUS
    driver
        .rightTrigger()
        .whileTrue(shooting.shootDriving(Shooting.HUB_TARGET, x, y, omega).withName("HUB"));

    // SCORING FALL BACK (FIXED POSITION)
    driver
        .y()
        .whileTrue(
            hopper
                .intake()
                .alongWith(indexer.forward().alongWith(shooter.runShooter(150)))
                .withName("fallback"));

    // driver.b().whileTrue(slapdown.squeezeVolts()).onFalse(slapdown.extend());

    // CLIMB
    // operator
    //     .y()
    //     .whileTrue(climb.extend())
    //     .onFalse(climb.retract());

    operator.x().whileTrue(shooting.shootWithTestData().withName("test data"));

    operator
        .leftBumper()
        .whileTrue(intake.intake().alongWith(indexer.forward()).alongWith(hopper.intake()));

    operator.y().whileTrue(hopper.intake());

    // operator.a().whileTrue(turret.goTo(() -> 3 * Math.PI / 2));
    operator.b().whileTrue(hopper.outtake());

    operator.povRight().whileTrue(slapdown.homingSequence());

    operator.leftTrigger().whileTrue(turret.goLeft().withName("left"));
    operator.rightTrigger().whileTrue(turret.goRight().withName("right"));

    operator.povUp().onTrue(ShootingData.changeSC(1));
    operator.povDown().onTrue(ShootingData.changeSC(-1));

    // operator.povLeft().whileTrue(slapdown.homingSequence());

    shooting
        .crossingAlliance()
        .whileTrue(
            shooting
                .hideAway()
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("crossing"));

    // DEBUG
    // TODO: various operator debug stuff (turret, hood, shooter)

    // operator
    //     .a()
    //     .whileTrue(turret.goTo(() ->
    // TurretConstants.MIN_ANGLE.plus(Degrees.of(20)).in(Radians)));
    // operator.y().whileTrue(turret.goTo(() -> 0));
    // operator.leftTrigger().whileTrue(hood.goTo(Degrees.of(45)).withName("goto 45"));
    // operator.rightTrigger().whileTrue(hood.goTo(Degrees.of(25)).withName("goto 25"));
    driver.povLeft().or(operator.povLeft()).whileTrue(hood.homingSequence());

    // operator
    //     .leftBumper()
    //     .or(operator.rightBumper())
    //     .whileTrue(turret.manualTurret(InputStream.of(() -> operator.getLeftX())));
  }

  /**
   * Command factory to make both controllers rumble.
   *
   * @param rumbleType The area of the controller to rumble.
   * @param strength The intensity of the rumble.
   * @return The command to rumble both controllers.
   */
  public Command rumble(RumbleType rumbleType, double strength) {
    return Commands.runOnce(
            () -> {
              driver.getHID().setRumble(rumbleType, strength);
              operator.getHID().setRumble(rumbleType, strength);
            })
        .andThen(Commands.waitSeconds(0.3))
        .finallyDo(
            () -> {
              driver.getHID().setRumble(rumbleType, 0);
              operator.getHID().setRumble(rumbleType, 0);
            });
  }

  /**
   * Creates a command that runs a systems check on all mechanisms.
   *
   * @return A command that tests all mechanisms.
   */
  public Command systemsCheck() {
    return Commands.sequence(
            // drive.systemsCheck(),
            turret.systemsCheck().withTimeout(6),
            hood.systemsCheck().withTimeout(6),
            shooter.systemsCheck().withTimeout(6),
            slapdown.systemsCheck().withTimeout(1),
            intake.intake().withTimeout(3))
        .withName("Test Mechansims");
  }

  @Override
  public void close() {
    super.close();
    try {
      drive.close();
    } catch (Exception ignored) {
    }
  }
}
