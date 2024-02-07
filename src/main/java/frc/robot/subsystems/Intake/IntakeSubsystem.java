package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

public class IntakeSubsystem extends SubsystemBase {
    /* 
     * 2024s Intake
     * There are just a few  more sparkmaxs in the actual
     * For this there are just 2 for example purpose
     */
    private final CANSparkMax intakefr;
    
    public IntakeSubsystem() {
        intakefr = new CANSparkMax(13, MotorType.kBrushless);

        intakefr.restoreFactoryDefaults();
        intakefr.setInverted(true);
        intakefr.setIdleMode(IdleMode.kCoast);
    }

    public Command runIntake(double speed){
        return run(()-> {
            intakefr.set(speed);
        });
    }

    public Command stopIntake(double speed){
        return run(() -> {
            intakefr.set(0);
        });
    }
    
    public Command autoRunIntake(double speed){
        return run(() -> {
            intakefr.set(.5);
        });
    }

    @Override
    public void periodic(){}
}