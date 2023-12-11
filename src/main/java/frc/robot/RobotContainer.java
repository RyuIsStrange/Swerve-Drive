// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));
  private final SendableChooser<Command> autoChooser;
  CommandJoystick driverController = new CommandJoystick(1);
  XboxController driverXbox = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Add all actions to PathPlanner
    NamedCommands.registerCommand("Action 1", new PrintCommand("Performing Action 1."));
    NamedCommands.registerCommand("Action 2", new PrintCommand("Performing Action 2."));
    NamedCommands.registerCommand("Action 3", new PrintCommand("Performing Action 3."));

    // Make chooser on smart dashboard
    autoChooser = AutoBuilder.buildAutoChooser("Do Nothing");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the trigger bindings
    configureBindings();

    AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
            OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
            OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX(),
        () -> -driverXbox.getRightY());

    AbsoluteFieldDrive closedFieldAbsoluteDrive = new AbsoluteFieldDrive(drivebase,
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
            OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
            OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));
    TeleopDrive simClosedFieldRel = new TeleopDrive(drivebase,
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
            OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
            OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2), () -> true);
    TeleopDrive closedFieldRel = new TeleopDrive(
        drivebase,
        () -> MathUtil.applyDeadband(driverController.getY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverController.getRawAxis(3), () -> true);

    drivebase.setDefaultCommand(!RobotBase.isSimulation() ? closedAbsoluteDrive : closedFieldAbsoluteDrive);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));

    // *******************************
    // Path Plan to pose, then follow path
    // *******************************
    // On the fly path following
    PathPlannerPath path = PathPlannerPath.fromPathFile("Get Cone");

    // Create the constraints to use while pathfinding. The constraints defined in
    // the path will only be used for the path.
    PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
        path,
        constraints,
        3.0 // Rotation delay distance in meters. This is how far the robot should travel
            // before attempting to rotate.
    );
    new JoystickButton(driverXbox, 3).onTrue(pathfindingCommand);

    // *******************************
    // Path Plan to pose
    // *******************************
    // Since we are using a holonomic drivetrain, the rotation component of this
    // pose
    // represents the goal holonomic rotation
    Pose2d targetPose = new Pose2d(2, 2.25, Rotation2d.fromDegrees(180));

    // Create the constraints to use while pathfinding
    PathConstraints scoring_constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command scoring_pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        scoring_constraints,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel
            // before attempting to rotate.
    );
    new JoystickButton(driverXbox, 4).onTrue(scoring_pathfindingCommand);
    // new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new
    // InstantCommand(drivebase::lock, drivebase)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void setDriveMode() {
    // drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
