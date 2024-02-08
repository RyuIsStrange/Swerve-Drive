package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;
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

    public void run(double speed) {
        climberMain.set(speed);
    }
    public void stop(){
        climberMain.set(0);
    }
    public double getEncoderPosition() {
        return climbEncoder.getPosition();
    }
    public double getHeight() {
        return climbEncoder.getPosition();
    }


    public enum ClimberState {
        RETRACTED(0), EXTENDED(100);

        public double height;

        private ClimberState(double height) {
            this.height = height;
        }
    }

    public void set(double p, double i, double d, double f, double iz) {
        PIDController.setP(p);
        PIDController.setI(i);
        PIDController.setD(d);
        PIDController.setFF(f);
        PIDController.setIZone(iz);
    }

    public void runPID(double targetPosition) {
        PIDController.setReference(targetPosition, CANSparkBase.ControlType.kPosition);
    }
    public Command setLeftSpeed(double speed) {
        return run(()-> {
            climberMain.set(speed);
        });
    }
    public Command setHeight(double height) {
        return run(() -> {
            runPID(height);
        });
    }
    // Waiting for them to make it, so I can do the math for Encoder distance
}
