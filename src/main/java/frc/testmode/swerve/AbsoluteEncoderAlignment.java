package frc.testmode.swerve;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

final class CANCoders {
    public static final CANCoder FRONT_LEFT_CAN_CODER = new CANCoder(16);
    public static final CANCoder FRONT_RIGHT_CAN_CODER = new CANCoder(10);
    public static final CANCoder BACK_LEFT_CAN_CODER = new CANCoder(4);
    public static final CANCoder BACK_RIGHT_CAN_CODER = new CANCoder(8);
}
  
public class AbsoluteEncoderAlignment {
    public AbsoluteEncoderAlignment() {
        zeroPositions();
    }
    
    public void testInit() {
        zeroPositions();
    }
    
    public void testPeriodic() {
        updateDashboard();
    }

    public void updateDashboard(){
        SmartDashboard.putNumber("16", CANCoders.FRONT_LEFT_CAN_CODER.getAbsolutePosition());
        SmartDashboard.putNumber("10", CANCoders.FRONT_RIGHT_CAN_CODER.getAbsolutePosition());
        SmartDashboard.putNumber("4", CANCoders.BACK_LEFT_CAN_CODER.getAbsolutePosition());
        SmartDashboard.putNumber("8", CANCoders.BACK_RIGHT_CAN_CODER.getAbsolutePosition());
    }
    
    public void zeroPositions(){
        CANCoders.FRONT_LEFT_CAN_CODER.configMagnetOffset(0);
        CANCoders.FRONT_RIGHT_CAN_CODER.configMagnetOffset(0);
        CANCoders.BACK_LEFT_CAN_CODER.configMagnetOffset(0);
        CANCoders.BACK_RIGHT_CAN_CODER.configMagnetOffset(0);
    }    
}
