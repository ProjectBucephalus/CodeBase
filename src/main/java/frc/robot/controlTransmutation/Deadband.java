package frc.robot.controlTransmutation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants.ControlConstants;

/** 
 * Applies a deadband region to the input 
 * @author 5985
 */
public class Deadband implements InputTransmuter
{
  protected double deadbandVal;

  /**
   * Creates a deadband filter to zero any inputs below a threshold
   * @param deadbandVal Optional, absolute value of input below which the output will be zero. Defaults to {@link ControlConstants#stickDeadband stickDeadband}
   */
  public Deadband()
    {this(ControlConstants.stickDeadband);}
  
  /**
   * Creates a deadband filter to zero any inputs below a threshold
   * @param deadband Optional, absolute value of input below which the output will be zero. Defaults to {@link ControlConstants#stickDeadband stickDeadband}
   */
  public Deadband(double deadband)
    {this.deadbandVal = MathUtil.clamp(deadband,0,1);}
  
  /** Zeroes the input if its normal is below the threshold */
  @Override
  public Translation2d process(Translation2d controlInput)
  {
    //gets the modulus of control input
    double mod = controlInput.getNorm();
    //gets the normal of control input
    Translation2d norm = controlInput.div(controlInput.getNorm());
    if (controlInput.getNorm() <= deadbandVal || deadbandVal == 1)
    {
      return Translation2d.kZero;
    } else {
      return norm.times((mod-deadbandVal)/(1-deadbandVal));
    }
  }
}