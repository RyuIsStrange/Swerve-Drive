package frc.robot.subsystems.Conv;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

public class ConvSubsystem extends SubsystemBase {
    private final CANSparkMax convMotor;
    
    public ConvSubsystem() {
        convMotor = new CANSparkMax(15, MotorType.kBrushless);
        convMotor.restoreFactoryDefaults();
        convMotor.setIdleMode(IdleMode.kBrake);
    }
    
    public Command runConv(double speed){
        return run(() -> {
            convMotor.set(speed);
        });
    }
    public Command runConvIntake(){
        return run(() -> {
            convMotor.set(.1);
        });
    }

    // Auto
    public Command autoRunConv(){
        return run(() -> {
            convMotor.set(.5);
        });
    }
    public Command stopConv(){
        return run(() -> {
            convMotor.set(0);
        });
    }
}
