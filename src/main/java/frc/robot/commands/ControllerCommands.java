package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Driver;
import frc.robot.PowerManagement.AdaptiveSpeedController;
import frc.robot.PowerManagement.IBrownOutDetector;
import frc.robot.subsystems.DriveSubsystem;

final class DriveConstants {
  public static final double MAX_TELEOP_SPEED = .75; //Range 0 to 1
  public static final double MAX_FINESSE_SPEED = .3;
}

public class ControllerCommands extends CommandBase {
  private final DriveSubsystem drive;
  private final AdaptiveSpeedController speedController;
  
  public ControllerCommands(DriveSubsystem drive, IBrownOutDetector brownOutDetector) {
    this.drive = drive;
    this.speedController = new AdaptiveSpeedController(brownOutDetector, 3.0, DriveConstants.MAX_FINESSE_SPEED, DriveConstants.MAX_TELEOP_SPEED);
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