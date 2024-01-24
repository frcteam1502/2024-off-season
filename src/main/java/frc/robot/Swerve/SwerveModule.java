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

public class SwerveModule {
  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;
  private final RelativeEncoder driveEncoder;
  private final CANCoder absEncoder;
  private final SparkMaxPIDController drivePIDController;
  private final PIDController turningPIDController;

  public SwerveModule(team1502.configuration.Builders.SwerveModule config) {
    this.driveMotor = buildMotor(config.DrivingMotor());
    this.turningMotor = buildMotor(config.TurningMotor());
    this.absEncoder = buildEncoder(config.Encoder());

    driveMotor.setClosedLoopRampRate(config.getDouble("closedLoopRampRate"));
    driveMotor.setSmartCurrentLimit(config.getInt("smartCurrentLimit"));

    driveEncoder = driveMotor.getEncoder();

    // Set the distance per pulse for the drive encoder. 
    driveEncoder.setPositionConversionFactor(config.getPositionConversionFactor());

    // Set the velocity per pulse for the drive encoder
    driveEncoder.setVelocityConversionFactor(config.getVelocityConversionFactor());

    // Set the angle in radians per pulse for the turning encoder.
    this.absEncoder.configSensorDirection(config.Encoder().Direction());
    this.absEncoder.configMagnetOffset(-config.Encoder().MagneticOffset());

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    this.turningPIDController = new PIDController(
      config.TurningMotor().PID().P(),
      config.TurningMotor().PID().I(),
      config.TurningMotor().PID().D());
    this.turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    this.drivePIDController = this.driveMotor.getPIDController();
    this.drivePIDController.setP(config.DrivingMotor().PID().P());
    this.drivePIDController.setI(config.DrivingMotor().PID().I());
    this.drivePIDController.setD(config.DrivingMotor().PID().D());
    this.drivePIDController.setFF(config.DrivingMotor().PID().FF());
  }

  private CANSparkMax buildMotor(team1502.configuration.Builders.Controllers.MotorController config) {
    var motor = new CANSparkMax(config.CanNumber(), config.Motor().MotorType());
    motor.setIdleMode(config.IdleMode());
    motor.setInverted(config.Reversed());
    return motor;
  }

  private CANCoder buildEncoder(team1502.configuration.Builders.Controllers.CANCoder config) {
    var encoder = new CANCoder(config.CanNumber());
    return encoder;
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

    //driveMotor.setVoltage(driveOutput + driveFeedforward);
    drivePIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
    turningMotor.setVoltage(turnOutput);
  }
}