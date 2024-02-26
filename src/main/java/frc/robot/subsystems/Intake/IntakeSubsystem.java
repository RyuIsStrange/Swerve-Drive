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
    //private final ConvSubsystem m_conv = new ConvSubsystem();

    public IntakeSubsystem() {
        intakeRight = new CANSparkMax(13, MotorType.kBrushless);
        intakeLeft = new CANSparkMax(14, MotorType.kBrushless);

        intakeRight.restoreFactoryDefaults();
        intakeLeft.restoreFactoryDefaults();
        intakeRight.setIdleMode(IdleMode.kCoast);
        intakeLeft.setIdleMode(IdleMode.kCoast);
    }
    /* Intake still not on Joysticks
    // public void changeAngle(double liftPower) {
    //     intakeLeft.set(liftPower);
    // }

    // public Command runManual(DoubleSupplier supplier){
    //     double power = supplier.getAsDouble();
    //     return run(() -> {
    //         changeAngle(power);
    //     });
    // }
    // public void setLeftMan(double liftPower) {
    //     intakeLeft.set(liftPower);
    // }
     */

    public Command runRightIntake(Double supplier){
        //double speed = supplier.getAsDouble();
        return run(()-> {
            intakeRight.set(supplier);
        });
    }
    public Command runLeftIntake(DoubleSupplier supplier){
        double power = supplier.getAsDouble();
        return run(() -> {
            intakeLeft.set(power);
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
            intakeRight.set(.75);
            intakeLeft.set(.75);
            //m_conv.runConvIntake();
        });
    }

    public Command autoRunIntakeRevers(){
        return run(() -> {
            intakeRight.set(-.85);
            intakeLeft.set(-.85);
            //m_conv.runConvIntake();
        });
    }
    @Override
    public void periodic(){}
}