package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax shooterMotor;
    public ShooterSubsystem() {
        shooterMotor = new CANSparkMax(16, MotorType.kBrushless);
        shooterMotor.restoreFactoryDefaults();
        shooterMotor.setIdleMode(IdleMode.kBrake);
    }

    public Command runShooter(double speed){
        return run(() ->
            shooterMotor.set(speed)
        );
    }

    // Auto
    public Command stopShooter(){
        return this.run(() -> 
            shooterMotor.set(0)
        );
    }
    public Command autoShooterRun(){
        return this.run(() -> 
            shooterMotor.set(1)
        );
    }
}
