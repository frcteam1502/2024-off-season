package frc.robot.Swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

final class ModuleConstants {
 
  // kinematics
  public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
  public static final double DRIVE_GEAR_RATIO = 1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
  public static final double STEER_GEAR_RATIO = 1 / ((14.0 / 50.0) * (10.0 / 60.0));
  public static final double DRIVE_METERS_PER_ENCODER_REV = (WHEEL_DIAMETER_METERS * Math.PI) / DRIVE_GEAR_RATIO;
  public static final double DRIVE_ENCODER_MPS_PER_REV = DRIVE_METERS_PER_ENCODER_REV / 60; 
  
  // max turn speed = (5400/ 21.43) revs per min 240 revs per min 4250 deg per min
  public static final double MODULE_TURN_PID_CONTROLLER_P = 3.4;
  public static final double MODULE_TURN_PID_CONTROLLER_I = 0;
  public static final double MODULE_TURN_PID_CONTROLLER_D = 0;
  
  // public static final double MODULE_TURN_PID_CONTROLLER_F = 0;
  public static final double MODULE_DRIVE_PID_CONTROLLER_P = .08;
  public static final double MODULE_DRIVE_PID_CONTROLLER_I = 0;
  public static final double MODULE_DRIVE_PID_CONTROLLER_D = 0;
  public static final double MODULE_DRIVE_PID_CONTROLLER_F = 1.0;
  
  public static final double CLOSED_LOOP_RAMP_RATE = .25;
  public static final int SMART_CURRENT_LIMIT = 80;

  /*
  public static final double MAX_METERS_PER_SECOND = 4.4; //5600 * DRIVE_ENCODER_MPS_PER_REV;
  public static final double TURNING_DEGREES_PER_ENCODER_REV = 360 / STEER_GEAR_RATIO;
  public static final double RADIANS_PER_ENCODER_REV = TURNING_DEGREES_PER_ENCODER_REV * (Math.PI/180);
  public static final double MAX_MODULE_ROTATION_RADIANS_PER_SECOND = Math.PI/2;
  public static final double MAX_MODULE_ROTATION_RADIANS_PER_SECOND_PER_SECOND = Math.PI;
  */
}


public class SwerveModule {
  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final RelativeEncoder driveEncoder;

  private final CANCoder absEncoder;

  private final SparkMaxPIDController drivePIDController;
  private final PIDController turningPIDController = new PIDController(ModuleConstants.MODULE_TURN_PID_CONTROLLER_P, ModuleConstants.MODULE_TURN_PID_CONTROLLER_I, ModuleConstants.MODULE_TURN_PID_CONTROLLER_D);

  private double commandedSpeed;
  private double commandedAngle;

  public SwerveModule(CANSparkMax driveMotor, CANSparkMax turnMotor, CANCoder absEncoder, double absOffset, boolean CANCoderDirection) {
    this.driveMotor = driveMotor;
    this.turningMotor = turnMotor;
    this.absEncoder = absEncoder;

    driveMotor.setClosedLoopRampRate(ModuleConstants.CLOSED_LOOP_RAMP_RATE);
    driveMotor.setSmartCurrentLimit(ModuleConstants.SMART_CURRENT_LIMIT);

    driveEncoder = driveMotor.getEncoder();

    // Set the distance per pulse for the drive encoder. 
    driveEncoder.setPositionConversionFactor(ModuleConstants.DRIVE_METERS_PER_ENCODER_REV);

    // Set the velocity per pulse for the drive encoder
    driveEncoder.setVelocityConversionFactor(ModuleConstants.DRIVE_ENCODER_MPS_PER_REV);

    // Set the angle in radians per pulse for the turning encoder.
    //turningEncoder.setPositionConversionFactor(Constants.ModuleConstants.RADIANS_PER_ENCODER_REV);
    this.absEncoder.configSensorDirection(CANCoderDirection);
    this.absEncoder.configMagnetOffset(-absOffset);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    this.turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    this.drivePIDController = this.driveMotor.getPIDController();
    this.drivePIDController.setP(ModuleConstants.MODULE_DRIVE_PID_CONTROLLER_P);
    this.drivePIDController.setI(ModuleConstants.MODULE_DRIVE_PID_CONTROLLER_I);
    this.drivePIDController.setD(ModuleConstants.MODULE_DRIVE_PID_CONTROLLER_D);
    this.drivePIDController.setFF(ModuleConstants.MODULE_DRIVE_PID_CONTROLLER_F);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(getAbsPositionZeroed(true)));
  }

  public double getVelocity() {
    return driveEncoder.getVelocity();
  }

  public Rotation2d geRotation2d() {
    return new Rotation2d(Units.degreesToRadians(absEncoder.getAbsolutePosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(getAbsPositionZeroed(true)));
  }

  public void zeroModule() {
    driveEncoder.setPosition(0);
  }

  public double getAbsPositionZeroed(boolean inRadians) {
    return Units.degreesToRadians(absEncoder.getAbsolutePosition());
  }

  public double getCommandedSpeed(){
    return commandedSpeed;
  }

  public double getModuleVelocity(){
    return driveEncoder.getVelocity();
  }

  public double getCommandedAngle(){
    return commandedAngle;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getAbsPositionZeroed(true)));

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = turningPIDController.calculate(getAbsPositionZeroed(true), state.angle.getRadians());

    //Set SmartDashboard variables
    commandedSpeed = state.speedMetersPerSecond;
    commandedAngle = turnOutput;

    drivePIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
    turningMotor.setVoltage(turnOutput);
  }
}