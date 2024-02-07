package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

//import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeRight; // Right Joystick
    private final CANSparkMax intakeLeft; // Left Joystick

    public IntakeSubsystem() {
        intakeRight = new CANSparkMax(13, MotorType.kBrushless);
        intakeLeft = new CANSparkMax(14, MotorType.kBrushless);

        intakeRight.restoreFactoryDefaults();
        intakeLeft.restoreFactoryDefaults();
        intakeRight.setIdleMode(IdleMode.kCoast);
        intakeLeft.setIdleMode(IdleMode.kCoast);
    }

    public Command runRightIntake(DoubleSupplier supplier){
        double speed = supplier.getAsDouble();
        return run(()-> {
            intakeRight.set(speed);
        });
    }
    public Command runLeftIntake(DoubleSupplier supplier){
        double speed = supplier.getAsDouble();
        return run(()-> {
            intakeLeft.set(speed);
        });
    }

    public Command stopRightIntake(){
        return run(() -> {
            intakeRight.set(0);
        });
    }

    public Command stopLeftIntake(){
        return run(() -> {
            intakeLeft.set(0);
        });
    }
    // Auto Command
    public Command stopIntake(){
        return run(() -> {
            intakeRight.set(0);
            intakeLeft.set(0);
        });
    }
    
    public Command autoRunIntake(){
        return run(() -> {
            intakeRight.set(0.5);
            intakeLeft.set(-0.5);
        });
    }

    @Override
    public void periodic(){}
}