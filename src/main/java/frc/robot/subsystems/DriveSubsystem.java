package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.GameState;
import frc.robot.Swerve.SwerveModules;

import team1502.configuration.RobotConfiguration;
import team1502.configuration.annotations.SubsystemInfo;

@SubsystemInfo(disabled = true)
public class DriveSubsystem extends SubsystemBase{
  
  private boolean isTurning = false;
  private double targetAngle = 0.0;
  private double turnCommand = 0.0;
  private double forwardCommand = 0;
  private double strafeCommand = 0;

  private final Pigeon2 gyro;

  private final SwerveModules swerveModules;
  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator odometry;

  private Pose2d pose = new Pose2d();
  
  private double pitchOffset;
  private final double goStraightGain;
  private final double maxSpeed;
  private final double empiricalSpeed; // for comparison

  public DriveSubsystem(RobotConfiguration config) {
    gyro = new Pigeon2(config.GyroSensor("Pigeon2").CanNumber());
  
    swerveModules = new SwerveModules(config);
    kinematics = config.Eval(e -> e.SwerveDrive(d->d.getKinematics()));
    maxSpeed = swerveModules.maxSpeed;
    empiricalSpeed = swerveModules.empiricalSpeed;
    
    goStraightGain = config.Eval(e -> e.SwerveDrive(d->d.getDouble("goStraightGain")));
    pitchOffset = 0;

    this.odometry = new SwerveDrivePoseEstimator(kinematics, getGyroRotation2d(), getModulePositions(), pose);

    pitchOffset = gyro.getRoll();

    reset();
  }

  private void checkInitialAngle() {
    if (GameState.isTeleop() && GameState.isFirst()) {
      targetAngle = gyro.getYaw();
    }
  }
  
  @Override
  public void periodic() {
    checkInitialAngle();
    updateOdometry();
  }
  
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    checkInitialAngle();
    ChassisSpeeds speedCommands = new ChassisSpeeds(0, 0, 0);

    if (GameState.isTeleop()) {
      if (Math.abs(rot) > 0) {
        isTurning = true;
        targetAngle = gyro.getYaw();
      } 
      else if (rot == 0 && isTurning) {
        isTurning = false;
      }

      if (isTurning) {
        turnCommand = rot;
      }
      else { 
        turnCommand = (targetAngle - gyro.getYaw()) * goStraightGain;
      }

      forwardCommand = xSpeed;
      strafeCommand = ySpeed;
    }

    if(fieldRelative){
      speedCommands = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnCommand, getGyroRotation2d());
    } else {
      speedCommands.omegaRadiansPerSecond = turnCommand;
      speedCommands.vxMetersPerSecond = xSpeed;
      speedCommands.vyMetersPerSecond = ySpeed;
    }

    driveRobotRelative(speedCommands);
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
    //This method is a consumer of ChassisSpeed and sets the corresponding module states.  This is required for PathPlanner 2024
    //Convert from robot frame of reference (ChassisSpeeds) to swerve module frame of reference (SwerveModuleState)
    var swerveModuleStates = kinematics.toSwerveModuleStates(robotRelativeSpeeds);
    //Normalize wheel speed commands to make sure no speed is greater than the maximum achievable wheel speed.
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);
    //Set the speed and angle of each module
    setDesiredState(swerveModuleStates);
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    //This method is a supplier of ChassisSpeeds as determined by the module states.  This is required for PathPlanner 2024
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  
  public SwerveModuleState[] getModuleStates(){ return swerveModules.getModuleStates(); }
  public SwerveModulePosition[] getModulePositions() { return swerveModules.getModulePositions(); }
  public void setDesiredState(SwerveModuleState[] swerveModuleStates) { swerveModules.setDesiredState(swerveModuleStates); }
  public void resetModules() { swerveModules.resetModules(); }

  public void updateOdometry() {
    pose = odometry.update(getGyroRotation2d(), getModulePositions());
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getGyroRotation2d(), getModulePositions(), pose);
  }
  
  public SwerveModuleState[] makeSwerveModuleState(double[] speeds, double[] angles) {
    SwerveModuleState[] moduleStates = new SwerveModuleState[angles.length];
    for(int i = 0; i < angles.length; i++) moduleStates[i] = new SwerveModuleState(speeds[i], new Rotation2d(Units.degreesToRadians(angles[i])));
    return moduleStates;
  }

  public void setToBreak() {
    resetModules();
    double[] speeds = {0, 0, 0, 0};
    double[] angles = {90, 90, 90, 90};
    SwerveModuleState[] moduleStates = makeSwerveModuleState(speeds, angles);
    setDesiredState(moduleStates);
  }

  public Rotation2d getGyroRotation2d() {
    return new Rotation2d(Units.degreesToRadians(gyro.getYaw()));
  }

  public Pose2d getPose2d() {
    return odometry.getEstimatedPosition();
  }

  public double getVelocity() {
    return Math.sqrt(
      Math.pow(ChassisSpeeds.fromFieldRelativeSpeeds(forwardCommand, strafeCommand, turnCommand, getGyroRotation2d()).vxMetersPerSecond, 2) + 
      Math.pow(ChassisSpeeds.fromFieldRelativeSpeeds(forwardCommand, strafeCommand, turnCommand, getGyroRotation2d()).vyMetersPerSecond, 2)
    );
  }

  public Rotation2d getHeading() {
    return new Rotation2d(
      Math.atan2(
        Math.pow(ChassisSpeeds.fromFieldRelativeSpeeds(forwardCommand, strafeCommand, turnCommand, getGyroRotation2d()).vyMetersPerSecond, 2), 
        Math.pow(ChassisSpeeds.fromFieldRelativeSpeeds(forwardCommand, strafeCommand, turnCommand, getGyroRotation2d()).vxMetersPerSecond, 2)
      )
    );
  }

  public double getRoll() {
    return gyro.getRoll() - pitchOffset;
  }

  public void resetGyro() {
    gyro.setYaw(0);
  }

  public void reset() {
    resetGyro();
    resetModules();
    resetOdometry(pose);
  }  
}
