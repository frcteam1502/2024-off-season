package frc.robot.subsystems;


import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;


import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Swerve.SwerveModule;

// -- path planner and limelight --
// import java.util.HashMap;
// import java.util.List;
// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.PathPoint;
// import com.pathplanner.lib.auto.PIDConstants;
// import com.pathplanner.lib.auto.SwerveAutoBuilder;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.libraries.LimelightHelpers;

final class Gyro {
  public static final Pigeon2 gyro = new Pigeon2(14);
}

final class CANCoders {
  //Front Left CANCoder
  public static final CANCoder FRONT_LEFT_CAN_CODER = new CANCoder(12);
  public static final boolean FRONT_LEFT_CAN_CODER_DIRECTION = false;
  public static final double FRONT_LEFT_CAN_CODER_OFFSET = 111.4;

  //Front Right CANCoder
  public static final CANCoder FRONT_RIGHT_CAN_CODER = new CANCoder(13);
  public static final boolean FRONT_RIGHT_CAN_CODER_DIRECTION = false;
  public static final double FRONT_RIGHT_CAN_CODER_OFFSET = 104.0;

  //Back Left CANCoder
  public static final CANCoder BACK_LEFT_CAN_CODER = new CANCoder(11);
  public static final boolean BACK_LEFT_CAN_CODER_DIRECTION = false;
  public static final double BACK_LEFT_CAN_CODER_OFFSET = 15.0;

  //Back Right CANCoder
  public static final CANCoder BACK_RIGHT_CAN_CODER = new CANCoder(10);
  public static final boolean BACK_RIGHT_CAN_CODER_DIRECTION = false;
  public static final double BACK_RIGHT_CAN_CODER_OFFSET = 91.4; 
  
}

final class DriveConstants {
  public static final double MAX_SPEED_METERS_PER_SECOND = 1.2; //IF YOU UP THE SPEED CHANGE ACCELERATION
  
  //Turning Motors
  public static final boolean FrontLeftTurningMotorReversed = true;
  public static final boolean BackLeftTurningMotorReversed = true;
  public static final boolean FrontRightTurningMotorReversed = true;
  public static final boolean BackRightTurningMotorReversed = true;
  
  public static final CANSparkMax.IdleMode FrontLeftTurningMotorBrake = IdleMode.kCoast;
  public static final CANSparkMax.IdleMode BackLeftTurningMotorBrake = IdleMode.kCoast;
  public static final CANSparkMax.IdleMode FrontRightTurningMotorBrake = IdleMode.kCoast;
  public static final CANSparkMax.IdleMode BackRightTurningMotorBrake = IdleMode.kCoast;
  
  //Drive Motors
  public static final boolean FrontLeftDriveMotorReversed = false;
  public static final boolean BackLeftDriveMotorReversed = false;
  public static final boolean FrontRightDriveMotorReversed = false;
  public static final boolean BackRightDriveMotorReversed = false;
  
  public static final CANSparkMax.IdleMode FrontLeftDriveMotorBrake = IdleMode.kBrake;
  public static final CANSparkMax.IdleMode BackLeftDriveMotorBrake = IdleMode.kBrake;
  public static final CANSparkMax.IdleMode FrontRightDriveMotorBrake = IdleMode.kBrake;
  public static final CANSparkMax.IdleMode BackRightDriveMotorBrake = IdleMode.kBrake;
  
  //Wheel Base
  public static final double WHEEL_BASE_WIDTH = Units.inchesToMeters(25);
  public static final double WHEEL_BASE_LENGTH = Units.inchesToMeters(32);
  
  public static final boolean GYRO_REVERSED = true;
  
  public static final Translation2d FRONT_LEFT_MODULE = new Translation2d(WHEEL_BASE_LENGTH/2, WHEEL_BASE_WIDTH/2);
  public static final Translation2d FRONT_RIGHT_MODULE = new Translation2d(WHEEL_BASE_LENGTH/2, -WHEEL_BASE_WIDTH/2);
  public static final Translation2d BACK_LEFT_MODULE = new Translation2d(-WHEEL_BASE_LENGTH/2, WHEEL_BASE_WIDTH/2);
  public static final Translation2d BACK_RIGHT_MODULE = new Translation2d(-WHEEL_BASE_LENGTH/2, -WHEEL_BASE_WIDTH/2);
  
  public static final SwerveDriveKinematics KINEMATICS =
  new SwerveDriveKinematics(
    FRONT_LEFT_MODULE, 
    FRONT_RIGHT_MODULE, 
    BACK_LEFT_MODULE, 
    BACK_RIGHT_MODULE
    );
    
    /*
      
      public static final double MAX_ROTATION_RADIANS_PER_SECOND = (Math.PI/2);
      public static final double MAX_ROTATION_RADIANS_PER_SECOND_PER_SECOND = Math.PI;
      */
      
  public static final double GO_STRAIGHT_GAIN = 0.02;
}


final class Motors {
  //drive
  public static final CANSparkMax DRIVE_FRONT_LEFT = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
  public static final CANSparkMax DRIVE_FRONT_RIGHT = new CANSparkMax(8, CANSparkMaxLowLevel.MotorType.kBrushless);
  public static final CANSparkMax DRIVE_BACK_LEFT = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
  public static final CANSparkMax DRIVE_BACK_RIGHT = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
  
  //turn
  public static final CANSparkMax ANGLE_FRONT_LEFT = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
  public static final CANSparkMax ANGLE_FRONT_RIGHT = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);
  public static final CANSparkMax ANGLE_BACK_LEFT = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
  public static final CANSparkMax ANGLE_BACK_RIGHT = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
}


public class DriveSubsystem extends SubsystemBase{
  
  public static boolean isTeleOp = false;

  public boolean isTurning = false;
  public double targetAngle = 0.0;
  public double turnCommand = 0.0;
  public double forwardCommand = 0;
  public double strafeCommand = 0;

  private final SwerveModule frontLeft = new SwerveModule(
    Motors.DRIVE_FRONT_LEFT, Motors.ANGLE_FRONT_LEFT, 
    CANCoders.FRONT_LEFT_CAN_CODER, 
    CANCoders.FRONT_LEFT_CAN_CODER_OFFSET,
    CANCoders.FRONT_LEFT_CAN_CODER_DIRECTION);

  private final SwerveModule frontRight = new SwerveModule(
    Motors.DRIVE_FRONT_RIGHT, Motors.ANGLE_FRONT_RIGHT, 
    CANCoders.FRONT_RIGHT_CAN_CODER, 
    CANCoders.FRONT_RIGHT_CAN_CODER_OFFSET,
    CANCoders.FRONT_RIGHT_CAN_CODER_DIRECTION);

  private final SwerveModule backLeft = new SwerveModule(
    Motors.DRIVE_BACK_LEFT, Motors.ANGLE_BACK_LEFT, 
    CANCoders.BACK_LEFT_CAN_CODER, 
    CANCoders.BACK_LEFT_CAN_CODER_OFFSET,
    CANCoders.BACK_LEFT_CAN_CODER_DIRECTION);

  private final SwerveModule backRight = new SwerveModule(
    Motors.DRIVE_BACK_RIGHT, Motors.ANGLE_BACK_RIGHT, 
    CANCoders.BACK_RIGHT_CAN_CODER, 
    CANCoders.BACK_RIGHT_CAN_CODER_OFFSET,
    CANCoders.BACK_RIGHT_CAN_CODER_DIRECTION);

  private final Pigeon2 gyro = Gyro.gyro;

  private final SwerveDriveKinematics kinematics = DriveConstants.KINEMATICS;

  public final SwerveDrivePoseEstimator odometry;

  private Pose2d pose = new Pose2d();
  
  private double pitchOffset;

  public DriveSubsystem() {
    pitchOffset = 0;

    this.odometry = new SwerveDrivePoseEstimator(kinematics, getGyroRotation2d(), getModulePositions(), pose);

    pitchOffset = gyro.getRoll();

    reset();

    ConfigMotorDirections();
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if(isTeleOp) {
      if(Math.abs(rot) > 0){
        isTurning = true;
        targetAngle = gyro.getYaw();
      } 
      else if(rot == 0 && isTurning) isTurning = false;

      if(isTurning) turnCommand = rot;
      else turnCommand = (targetAngle - gyro.getYaw()) * DriveConstants.GO_STRAIGHT_GAIN;

      forwardCommand = xSpeed;
      strafeCommand = ySpeed;
    }
    

    var swerveModuleStates =
        kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnCommand, getGyroRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
  
    setDesiredState(swerveModuleStates);
  }

  public void teleopInit() {
    targetAngle = gyro.getYaw();
  }

  public void setDesiredState(SwerveModuleState[] swerveModuleStates) {
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void updateOdometry() {
    pose = odometry.update(
        getGyroRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        });
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getGyroRotation2d(), getModulePositions(), pose);
  }
  
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    };
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

  public void resetModules() {
    frontLeft.zeroModule();
    frontRight.zeroModule();
    backLeft.zeroModule();
    backRight.zeroModule();
  }

  public void reset() {
    resetGyro();
    resetModules();
    resetOdometry(pose);
  }  

  public void ConfigMotorDirections() {
    Motors.ANGLE_FRONT_LEFT.setInverted(DriveConstants.FrontLeftTurningMotorReversed);
    Motors.ANGLE_FRONT_RIGHT.setInverted(DriveConstants.FrontRightTurningMotorReversed);
    Motors.ANGLE_BACK_LEFT.setInverted(DriveConstants.BackLeftTurningMotorReversed);
    Motors.ANGLE_BACK_RIGHT.setInverted(DriveConstants.BackRightTurningMotorReversed);
    Motors.DRIVE_FRONT_LEFT.setInverted(DriveConstants.FrontLeftDriveMotorReversed);
    Motors.DRIVE_FRONT_RIGHT.setInverted(DriveConstants.FrontRightDriveMotorReversed);
    Motors.DRIVE_BACK_LEFT.setInverted(DriveConstants.BackLeftDriveMotorReversed);
    Motors.DRIVE_BACK_RIGHT.setInverted(DriveConstants.BackRightDriveMotorReversed);

    Motors.DRIVE_FRONT_LEFT.setIdleMode(DriveConstants.FrontLeftDriveMotorBrake);
    Motors.DRIVE_FRONT_RIGHT.setIdleMode(DriveConstants.FrontRightDriveMotorBrake);
    Motors.DRIVE_BACK_LEFT.setIdleMode(DriveConstants.BackLeftDriveMotorBrake);
    Motors.DRIVE_BACK_RIGHT.setIdleMode(DriveConstants.BackRightDriveMotorBrake);

    Motors.ANGLE_FRONT_LEFT.setIdleMode(DriveConstants.FrontLeftTurningMotorBrake);
    Motors.ANGLE_FRONT_RIGHT.setIdleMode(DriveConstants.FrontRightTurningMotorBrake);
    Motors.ANGLE_BACK_LEFT.setIdleMode(DriveConstants.BackLeftTurningMotorBrake);
    Motors.ANGLE_BACK_RIGHT.setIdleMode(DriveConstants.BackRightTurningMotorBrake);
  }
/* 
  public Command moveToImage() {
    PathPlannerTrajectory toImage = PathPlanner.generatePath(
      new PathConstraints(SwerveConstants.DriveConstants.MAX_SPEED_METERS_PER_SECOND * 3, 1), 
      new PathPoint(new Translation2d(0, 0), getHeading(), getGyroRotation2d(), getVelocity()),
      pointFromLimelight(new Translation2d(Limelight.getTargetPose()[0], Limelight.getTargetPose()[1])) 
    );
    
    return new PPSwerveControllerCommand(
      toImage, 
      this::getPose2d, // Pose supplier
      SwerveConstants.DriveConstants.KINEMATICS, // SwerveDriveKinematics
      new PIDController(4.35, 0.0, 0.09), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
      new PIDController(4.35, 0.0, 0.09), // Y controller (usually the same values as X controller)
      new PIDController(1.3, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
      this::setDesiredState, // Module states consumer
      true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      this // Requires this drive subsystem
    );
  }

  public PathPoint pointFromLimelight(Translation2d imagePose) {
    double xDistanceFromCenter = 0;
    double yDistanceFromCenter = 0;
    double desiredDistanceFromImage = 0;

    if(Limelight.getPipeline() == 0) desiredDistanceFromImage = 1;
    else if(Limelight.getPipeline() == 1) desiredDistanceFromImage = 2;

    Translation2d goalTranslation = new Translation2d(
      imagePose.getX() - xDistanceFromCenter, 
      imagePose.getY() - yDistanceFromCenter - desiredDistanceFromImage);
    return new PathPoint(goalTranslation, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0);
  }

  public void strafe(boolean right) {
    if(!LimelightHelpers.getTV("Limelight")) {
      double strafe = .1;
      if(!right) strafe *= -1;
      drive(0, strafe, 0, true);
    }
    else moveToImage();
  }

  public void strafeRight() {
    strafe(true);
  }

  public void strafeLeft() {
    strafe(false);
  }
  public Command buildAuto(HashMap<String, Command> eventMap, String pathName) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
      pathName, 
      new PathConstraints(SwerveConstants.DriveConstants.MAX_SPEED_METERS_PER_SECOND / 1.1, 1));

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      this::getPose2d, // Pose2d supplier
      this::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
      SwerveConstants.DriveConstants.KINEMATICS, // SwerveDriveKinematics
      new PIDConstants(4.35, 0.0, 0.09), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(2.55, 0.0, 0.15), // PID constants to correct for rotation error (used to create the rotation controller)
      this::setDesiredState, // Module states consumer used to output to the drive subsystem
      eventMap,
      true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      this // The drive subsystem. Used to properly set the requirements of path following commands
    );
    
    return autoBuilder.fullAuto(pathGroup);
  }
*/
}