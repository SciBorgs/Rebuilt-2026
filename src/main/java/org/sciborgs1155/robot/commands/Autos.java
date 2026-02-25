package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.Constants.Robot.MASS;
import static org.sciborgs1155.robot.Constants.Robot.MOI;
import static org.sciborgs1155.robot.Constants.alliance;
import org.sciborgs1155.robot.climb.Climb;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants.ControlMode;
import static org.sciborgs1155.robot.drive.DriveConstants.MAX_SPEED;
import static org.sciborgs1155.robot.drive.DriveConstants.MODULE_OFFSET;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.Rotation;
import org.sciborgs1155.robot.drive.DriveConstants.Translation;
import static org.sciborgs1155.robot.drive.DriveConstants.WHEEL_COF;
import static org.sciborgs1155.robot.drive.DriveConstants.WHEEL_RADIUS;
import org.sciborgs1155.robot.hood.Hood;
import org.sciborgs1155.robot.intake.Intake;
import org.sciborgs1155.robot.shooter.Shooter;
import org.sciborgs1155.robot.turret.Turret;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;

public final class Autos {

  // Prevents instantiation
  private Autos() {}

  /**
   * Configures PathPlanner auto routines and returns a chooser for autonomous commands.
   *
   * @param drive The drive subsystem to use for autonomous.
   * @return A SendableChooser for selecting autonomous commands.
   */
  @NotLogged
  public static SendableChooser<Command> configureAutos(Drive drive, Intake intake, Shooter shooter, Hood hood, Turret turret, Climb climb) {
    AutoBuilder.configure(
        drive::pose,
        drive::resetOdometry,
        drive::robotRelativeChassisSpeeds,
        (s, g) -> drive.setChassisSpeeds(s, ControlMode.CLOSED_LOOP_VELOCITY),
        new PPHolonomicDriveController(
            new PIDConstants(Translation.P, Translation.I, Translation.D),
            new PIDConstants(Rotation.P, Rotation.I, Rotation.D)),
        new RobotConfig(
            MASS,
            MOI,
            new ModuleConfig(
                WHEEL_RADIUS,
                MAX_SPEED,
                WHEEL_COF,
                DCMotor.getKrakenX60(1).withReduction(Driving.GEARING),
                Driving.STATOR_LIMIT,
                1),
            MODULE_OFFSET),
        () -> alliance() == Alliance.Red,
        drive);

    PPHolonomicDriveController.overrideRotationFeedback(() -> drive.heading().getRadians());
    NamedCommands.registerCommand("shoot", new ScheduleCommand(Commands.parallel(hood.goToShootingAngle(Degrees.of(45)), turret.goTo(() -> 1), shooter.runShooter(50))));
    NamedCommands.registerCommand("intake", intake.intake());
    SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
    chooser.addOption("no auto", Commands.none());
    return chooser;
  }
}
