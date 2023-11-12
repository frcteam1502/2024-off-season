package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Driver;
import frc.robot.PowerManagement.AdaptiveSpeedController;
import frc.robot.PowerManagement.IBrownOutDetector;
import frc.robot.subsystems.DriveSubsystem;

final class DriveConstants {
  public static final double MAX_SPEED_METERS_PER_SECOND = 1.2; //IF YOU UP THE SPEED CHANGE ACCELERATION
}

public class ControllerCommands extends CommandBase {
  private final DriveSubsystem drive;
  private final AdaptiveSpeedController speedController;
  
  public ControllerCommands(DriveSubsystem drive, IBrownOutDetector brownOutDetector) {
    this.drive = drive;
    this.speedController = new AdaptiveSpeedController(brownOutDetector, 3.0, 0.3, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    addRequirements(drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    var speedCommand = speedController.GetSpeedCommand(
      Driver.getLeftY(), // Forward
      Driver.getLeftX(), // Strafe
      Driver.getRightX(), // Rotate
      Driver.XboxButtons.LeftBumper.getAsBoolean()); // brake
  
    drive.drive(-speedCommand.forwardSpeed, -speedCommand.strafeSpeed, -speedCommand.rotationSpeed, true);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}