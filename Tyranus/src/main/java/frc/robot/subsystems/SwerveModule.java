/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class SwerveModule {
  private final WPI_TalonSRX driveMotor;
  private final WPI_TalonSRX turningMotor;
  private double distZero;
  private double kDriveEncoderDistancePerPulse;

  private final AbsoluteEncoder driveEncoder;
  private PIDController drivePidController;
  private PIDController steerPid;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  public double dkP, dkI, dkD, dkIz, dkFF, dkMaxOutput, dkMinOutput;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel, int driveEncoderPort,
      double kDriveEncoderDistancePerPulse, boolean driveEncoderReversed, AbsoluteEncoder encoder) {

    this.kDriveEncoderDistancePerPulse = kDriveEncoderDistancePerPulse;
    driveMotor = new WPI_TalonSRX(driveMotorChannel);
    turningMotor = new WPI_TalonSRX(turningMotorChannel);

    // PID coefficients
    kP = 1;
    kI = 0;
    kD = 0;
    dkP = .5;
    dkI = 0;
    dkD = 0;

    drivePidController = new PIDController(dkP, dkI, dkD);
    steerPid = new PIDController(kP, kI, kD);

    this.driveEncoder = encoder;

    // turningEncoder = turningMotor.getAnalog(CANAnalog.AnalogMode.kAbsolute);

    // Set the distance per pulse for the drive AbsoluteEncoder. We can simply use
    // the
    // distance traveled for one rotation of the wheel divided by the
    // AbsoluteEncoder
    // resolution.
    // driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderDistancePerPulse);
    // driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderDistancePerPulse);

    // Set whether drive AbsoluteEncoder should be reversed or not
    // driveEncoder.set

    // Set the distance (in this case, angle) per pulse for the turning
    // AbsoluteEncoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the AbsoluteEncoder resolution.
    // driveEncoder.set(ModuleConstants.kTurningEncoderDistancePerPulse);
    // turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderDistancePerPulse);

    // Set whether turning AbsoluteEncoder should be reversed or not
    encoder.setReverseDirection(driveEncoderReversed);
    // driveEncoder.setDistancePerPulse(kDriveEncoderDistancePerPulse);

    // steerPid.setInputRange(0, 2 * Math.PI);
    // steerPid.setOutputRange(-Constants.DriveConstants.SWERVE_STEER_CAP,
    // Constants.DriveConstants.SWERVE_STEER_CAP);
    // steerPid.setContinuous();
    // steerPid.disable();
    // we want to log out the encoder values so that we can get additional data for
    // calibration
    SmartDashboard.putNumber("Initial Steer Encoder Value", encoder.getAngle());

    turningMotor.setNeutralMode(NeutralMode.Brake);
    // driveController.setIdleMode(IdleMode.kBrake);
    // driveController.setSmartCurrentLimit(30);
    // driveController.setOpenLoopRampRate(0.25);
    // driveController.burnFlash();
    // encoder = driveController.getEncoder();
    resetEncoders();
  }

  public double getDistance() {
    double distance = driveMotor.getSelectedSensorPosition() * kDriveEncoderDistancePerPulse - distZero;
    SmartDashboard.putNumber("distance", distance);
    return distance;
  }

  public double getAngle() {
    return driveEncoder.getAngle();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveMotor.getSelectedSensorVelocity(), new Rotation2d(driveEncoder.getAngle()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((p != kP)) {
      drivePidController.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      drivePidController.setI(i);
      kI = i;
    }

    SmartDashboard.putNumber("P", p);
    SmartDashboard.putNumber("I", i);
    SmartDashboard.putNumber("D", d);
    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);

    double desiredDrive = state.speedMetersPerSecond;
    double desiredSteering = state.angle.getRadians();
    double currentSteering = driveEncoder.getAngle();

    // calculate shortest path to angle with forward drive (error -pi to pi)
    double steeringError = Math.IEEEremainder(desiredSteering - currentSteering, 2 * Math.PI);

    // reverse drive if error is larger than 90 degrees
    if (steeringError > Math.PI / 2) {
      steeringError -= Math.PI;
      desiredDrive *= -1;
    } else if (steeringError < -Math.PI / 2) {
      steeringError += Math.PI;
      desiredDrive *= -1;
    }

    
    SmartDashboard.putNumber("desiredDrive", desiredDrive);
    SmartDashboard.putNumber("desiredSteering", desiredSteering);
    SmartDashboard.putNumber("currentSteering", currentSteering);

    double steeringSetpoint = currentSteering + steeringError;

    // sketchy code to show people it moves
    driveMotor.set(desiredDrive);

    // This works nicely, except:
    // >The robot doesn't take the quickest path and then reverse the drive motor
    // >When it hits 360 degrees, it rotates all the way around to 0 (see above)
    steerPid.setSetpoint(steeringSetpoint);

    SmartDashboard.putNumber("SetPoint", steeringSetpoint);
    SmartDashboard.putNumber("ProcessVariable", turningMotor.get());
  }

  /**
   * Zeros all the SwerveModule encoders.
   */

  public void resetEncoders() {
    //TODO: Fix and uncomment
    //driveEncoder.resetAccumulator();
    distZero += getDistance();
  }
}
