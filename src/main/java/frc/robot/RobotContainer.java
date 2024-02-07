// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Conv.ConvSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;;

public class RobotContainer
{
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final ConvSubsystem m_conv = new ConvSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/neo"));
  CommandJoystick driverController = new CommandJoystick(1);
  XboxController driverXbox = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Climber
    // Put things here when I make it
    // Conv
    NamedCommands.registerCommand("runConv", m_conv.autoRunConv());
    NamedCommands.registerCommand("stopConv", m_conv.stopConv());
    // Intake
    NamedCommands.registerCommand("runIntake", m_intake.autoRunIntake());
    NamedCommands.registerCommand("stopIntake", m_intake.stopIntake());
    // Shooter
    NamedCommands.registerCommand("runShooter", m_shooter.autoShooterRun());
    NamedCommands.registerCommand("stopShooter", m_shooter.stopShooter());

    // Configure the trigger bindings
    configureBindings();

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,() -> MathUtil.applyDeadband(driverXbox.getLeftY(),OperatorConstants.LEFT_Y_DEADBAND),() -> MathUtil.applyDeadband(driverXbox.getLeftX(),OperatorConstants.LEFT_X_DEADBAND),() -> MathUtil.applyDeadband(driverXbox.getRightX(),OperatorConstants.RIGHT_X_DEADBAND),driverXbox::getYButtonPressed,driverXbox::getAButtonPressed,driverXbox::getXButtonPressed,driverXbox::getBButtonPressed);

    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(() -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),() -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),() -> driverXbox.getRawAxis(2));

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(() -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),() -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),() -> driverXbox.getRawAxis(2));

    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX(),
        () -> -driverXbox.getRightY());

    drivebase.setDefaultCommand( !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngle );
  }

  private void configureBindings()
  {
    // Climber, DPad
    Constants.operatorController.povUp().onTrue(null);
    Constants.operatorController.povDown().onTrue(null);
    // Conv, Bumpers and run when inake
    Constants.operatorController.axisGreaterThan(5, 0.1).whileTrue(m_conv.runConvIntake());
    Constants.operatorController.rightBumper().onTrue(m_conv.runConv(.5));
    Constants.operatorController.leftBumper().onTrue(m_conv.runConv(-.5));
    Constants.operatorController.rightBumper().onFalse(m_conv.runConv(0));
    Constants.operatorController.leftBumper().onFalse(m_conv.runConv(0));
    // Intake, left and right
    new Trigger(() -> Math.abs(Constants.operatorController.getRawAxis(1)) > 0.1).whileTrue(m_intake.runLeftIntake(Constants.operatorController::getLeftY));
    new Trigger(() -> Math.abs(Constants.operatorController.getRawAxis(1)) < 0.1).whileTrue(m_intake.runLeftIntake(Constants.operatorController::getLeftY));
    new Trigger(() -> Math.abs(Constants.operatorController.getRawAxis(1)) == 0).whileTrue(m_intake.stopLeftIntake());
    new Trigger(() -> Math.abs(Constants.operatorController.getRawAxis(5)) > 0.1).whileTrue(m_intake.runRightIntake(Constants.operatorController::getRightY));
    new Trigger(() -> Math.abs(Constants.operatorController.getRawAxis(5)) < 0.1).whileTrue(m_intake.runRightIntake(Constants.operatorController::getRightY));
    new Trigger(() -> Math.abs(Constants.operatorController.getRawAxis(5)) == 0).whileTrue(m_intake.stopRightIntake());
    // Shooter
    Constants.operatorController.rightTrigger(.1).whileTrue(m_shooter.runShooter(1));
    Constants.operatorController.leftTrigger(.1).whileTrue(m_shooter.runShooter(-1));
    Constants.operatorController.rightTrigger().whileFalse(m_shooter.runShooter(0));
    Constants.operatorController.leftTrigger().whileFalse(m_shooter.runShooter(0));


    // Default stuff remove eventually
    new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
    new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    new JoystickButton(driverXbox, 2).whileTrue(Commands.deferredProxy(() -> drivebase.driveToPose(new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))));
    //new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Path", true);
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
