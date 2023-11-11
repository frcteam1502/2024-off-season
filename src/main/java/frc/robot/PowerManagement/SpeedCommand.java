package frc.robot.PowerManagement;

public class SpeedCommand {
  public double forwardSpeed;
  public double strafeSpeed;
  public double rotationSpeed;

  public SpeedCommand(double forwardSpeed, double strafeSpeed, double rotationSpeed) {
    this.forwardSpeed = forwardSpeed;
    this.strafeSpeed = strafeSpeed;
    this.rotationSpeed = rotationSpeed;
  }
}