package frc.robot;

// Un-used but may be needed
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.Commands;
// Main things
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
// Subsystems
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Conv.ConvSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class RobotContainer {
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final ConvSubsystem m_conv = new ConvSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();

  private final SwerveSubsystem m_drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/neo"));
  private final SendableChooser<Command> autoChooser;
  // CommandJoystick driverController = new CommandJoystick(1);
  XboxController driverXbox = new XboxController(0);
  
  UsbCamera camera1;
  public RobotContainer() {
    // Conv
    NamedCommands.registerCommand("runConv", m_conv.autoRunConv());
    NamedCommands.registerCommand("stopConv", m_conv.stopConv());
    // Intake
    NamedCommands.registerCommand("runIntake", m_intake.autoRunIntake());
    NamedCommands.registerCommand("stopIntake", m_intake.stopIntake());
    // Shooter
    NamedCommands.registerCommand("runShooter", m_shooter.autoShooterRun());
    NamedCommands.registerCommand("stopShooter", m_shooter.stopShooter());
    autoChooser = AutoBuilder.buildAutoChooser("Simple Auto");
    // Configure the trigger bindings
    SmartDashboard.putData("Auto Mode", autoChooser);
    //configurePathPlanner();
    
    PortForwarder.add(5800, "photonvision.local", 5800);
    camera1 = CameraServer.startAutomaticCapture(0);
    camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    configureBindings();
  }

  private void configureBindings() {
    // Climber, DPad
    Constants.operatorController.povDown().whileTrue(m_climber.uhOhNoWorky(.75)).whileFalse(m_climber.uhOhNoWorkyStop());
    Constants.operatorController.povUp().whileTrue(m_climber.uhOhNoWorky(-.75)).whileFalse(m_climber.uhOhNoWorkyStop());

    // Conv, Bumpers and run when inake
    Constants.operatorController.rightBumper().onTrue(m_conv.runConv(1))
      .onFalse(m_conv.runConv(0));
    Constants.operatorController.leftBumper().onTrue(m_conv.runConv(-1))
      .onFalse(m_conv.runConv(0));
    
    Constants.operatorController.a().onTrue(m_conv.autoRunConv())
      .onFalse(m_conv.stopConv());
    Constants.operatorController.b().onTrue(m_conv.runConv(-0.6))
      .onFalse(m_conv.stopConv());

    // Intake
    Constants.operatorController.a().whileTrue(m_intake.autoRunIntake())
      .whileFalse(m_intake.stopIntake());
    Constants.operatorController.b().whileTrue(m_intake.autoRunIntakeRevers())
      .whileFalse(m_intake.stopIntake());
    
    // Shooter, LT & RT
    Constants.operatorController.rightTrigger(.1).whileTrue(m_shooter.runShooter(.75))
      .whileFalse(m_shooter.runShooter(0));
    Constants.operatorController.leftTrigger(.1).whileTrue(m_shooter.runShooter(-.75))
      .whileFalse(m_shooter.runShooter(0));


    // Default stuff remove eventually
    new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(m_drivebase::zeroGyro)));
    new JoystickButton(driverXbox, 4).whileTrue(new InstantCommand(m_drivebase::lock, m_drivebase));
    
    // new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(m_drivebase::addFakeVisionReading));
    // new JoystickButton(driverXbox, 2).whileTrue(Commands.deferredProxy(() -> m_drivebase.driveToPose(new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))));
  }
  
  public void configurePathPlanner() {

    m_drivebase.setupPathPlanner();
  }

  public Command getAutonomousCommand() {
    // Gets Selected Auto from Shuffleboard
    return autoChooser.getSelected();
  }

  public void setDriveMode() {
    Command driveFieldOrientedDirectAngle = m_drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX(),
        () -> -driverXbox.getRightY());

    m_drivebase.setDefaultCommand( 
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngle );
  }

  public void setMotorBrake(boolean brake) {
    m_drivebase.setMotorBrake(brake);
  }
}
