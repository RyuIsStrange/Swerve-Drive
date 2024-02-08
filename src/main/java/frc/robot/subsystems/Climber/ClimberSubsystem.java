package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax climberMain;
    private final SparkPIDController PIDController;
    private RelativeEncoder climbEncoder;

    public ClimberSubsystem() {
        climberMain = new CANSparkMax(17, MotorType.kBrushless);

        climberMain.restoreFactoryDefaults();
        climberMain.setIdleMode(IdleMode.kBrake);
        PIDController = climberMain.getPIDController();
        climbEncoder = climberMain.getEncoder();
    }
    // Waiting for them to make it, so I can do the math for Encoder distance
}
