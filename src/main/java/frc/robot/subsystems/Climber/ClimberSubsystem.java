package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax climberMain;

    public ClimberSubsystem() {
        climberMain = new CANSparkMax(17, MotorType.kBrushless);

        climberMain.restoreFactoryDefaults();
        climberMain.setIdleMode(IdleMode.kBrake);
    }

    public Command uhOhNoWorky(double speed){
        return run(() -> 
            climberMain.set(speed)
        );
    }

    public Command uhOhNoWorkyStop() {
        return run(() -> 
            climberMain.set(0)
        );
    }
}
