package team1502.configuration.Builders;

import java.util.function.Function;

public class Encoder extends Builder {
  private Function<Encoder, Builder> buildFunction;
  /*
 *   public static final Pigeon2 Encoder = new Pigeon2(14);
 *   Same ctor for CANCoder
 * 
     driveEncoder = driveMotor.getEncoder();

    // Set the distance per pulse for the drive encoder. 
    driveEncoder.setPositionConversionFactor(ModuleConstants.DRIVE_METERS_PER_ENCODER_REV);

    // Set the velocity per pulse for the drive encoder
    driveEncoder.setVelocityConversionFactor(ModuleConstants.DRIVE_ENCODER_MPS_PER_REV);
 
    False (default) means positive rotation occurs when magnet
    is spun counter-clockwise when observer is facing the LED side of CANCoder.
   public static final boolean FRONT_LEFT_CAN_CODER_DIRECTION = false;
  public static final double FRONT_LEFT_CAN_CODER_OFFSET = 241.348 - 180.0;

    this.absEncoder.configSensorDirection(CANCoderDirection);
    this.absEncoder.configMagnetOffset(-absOffset);

 */
        //Define
        public Encoder(String name, Function<Encoder, Builder> fn) {
          super("EncoderSensor", name, null);
          buildFunction = fn;
      }
  
      //Build
      public Encoder(Function<Encoder, Builder> fn) {
          super("EncoderSensor");
          buildFunction = fn;
      }
      
      @Override
      public Builder createBuilder() {
          return new Encoder((Function<Encoder, Builder>)buildFunction);
      }
  
}
