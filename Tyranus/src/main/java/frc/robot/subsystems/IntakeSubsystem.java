package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {
    private WPI_TalonSRX mIntake = new WPI_TalonSRX(Constants.Intake.MOTOR);
    private Solenoid intakeSolenoid = new Solenoid(0);

    public IntakeSubsystem() {
        super();
       
       //mIntake.setIdleMode(IdleMode.kCoast);
    }

    public void extend() {
        intakeSolenoid.set(true);
    }

    public void retract() {
        intakeSolenoid.set(false);
    }

    public void rollWheels(double speed) {
        mIntake.set(speed);
    }

    public void stop() {
        rollWheels(0);
    }

}