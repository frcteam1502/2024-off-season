package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public final class Driver {
  public static final XboxController Controller = new XboxController(OperatorConstants.kDriverControllerPort);

  public static final class XboxButtons {
    public static final JoystickButton LeftBumper = new JoystickButton(Controller, XboxController.Button.kLeftBumper.value); 
    public static final JoystickButton RightBumper = new JoystickButton(Controller, XboxController.Button.kRightBumper.value); 
    public static final JoystickButton Y = new JoystickButton(Controller, XboxController.Button.kY.value); 
    public static final JoystickButton A = new JoystickButton(Controller, XboxController.Button.kA.value); 
    public static final JoystickButton X = new JoystickButton(Controller, XboxController.Button.kX.value); 
    public static final JoystickButton B = new JoystickButton(Controller, XboxController.Button.kB.value); 
    public static final JoystickButton Back = new JoystickButton(Controller, 7);
    public static final JoystickButton Start = new JoystickButton(Controller, 8);
    public static final JoystickButton LeftStick = new JoystickButton(Controller, 12);
    public static final JoystickButton RightStick = new JoystickButton(Controller, 13);
    public static final JoystickButton LeftTrigger = new JoystickButton(Controller, 14);
    public static final JoystickButton RightTrigger = new JoystickButton(Controller, 15);
    public static final POVButton DpadUp = new POVButton(Controller, 0);
    public static final POVButton DpadRight = new POVButton(Controller, 90);
    public static final POVButton DpadDown = new POVButton(Controller, 180);
    public static final POVButton DpadLeft = new POVButton(Controller, 270);
  }
}