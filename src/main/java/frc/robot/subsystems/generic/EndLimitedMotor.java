package frc.robot.subsystems.generic;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** s
 * Generic subclass for a range-limited motor with a binary switch reading {@code true} at an endstop 
 * @author 5985
 */
@Logged(strategy = Strategy.OPT_IN)
public class EndLimitedMotor extends LimitedMotor
{
  @Logged
  private final boolean safeSpeedValid;

  private boolean sensorValid = false;
  private boolean calibrated = false;
  private boolean homeAtMin = false;
  private boolean homeLastCycle = false;

  private TalonFXConfiguration motorConfig;

  /**
   * Creates generic limited motor system
   * @param motorCAN CAN-ID of underlying motor
   * @param limitIO DIO-ID of home limit-sensor. Set to {@code -1} to use motor stall instead of limit switch
   * @param invertLimit Whether the limit switch is inverted (i.e. false is at limit). Ignored if using motor stall
   * @param minRotations Minimum position in mechanism rotations
   * @param maxRotations Maximum position in mechanism rotations
   * @param homeAtMin Is the home position at the minimum (true) or maximum (false)
   * @param configs Motor configuration object, uses Slot1 if present when not calibrated <br>
   *                {@code CustomParam0} is used for the stall current value if using motor stall
   */
  public EndLimitedMotor(int motorCAN, int limitIO, boolean invertLimit, double minRotations, double maxRotations, boolean homeAtMin, TalonFXConfiguration configs)
  {
    
    super(motorCAN, limitIO, invertLimit, minRotations, maxRotations, 0, configs);

    motorConfig = configs;
    this.homeAtMin = homeAtMin;
    this.safeSpeedValid = super.safeSpeedValid;
    // Sanitise inputs
    if (this.homeAtMin)
    {
      super.homeRotations = minRotations;
    } else {
      super.homeRotations = maxRotations;
    }
  }

  @Override
  /** @return Command to run basic calibration cycle, calibrating on the high edge of the sensor if possible */
  public Command calibrateCmd()
  {
    return Commands.sequence
    (
      Commands.sequence
      (
        gotoTargetCmd(homeRotations)
      ).until(this::atLimit),
      adjustTargetCmd(() -> homeAtMin ? 0.05 : -0.05).until(() -> calibrated),
      setTargetCmd(homeRotations)
    );
  }

  @Override
  public void periodic() 
  {
    // Initialise position when we first move
    if (!sensorValid && active)
    {
      // First cycle active becomes true, marking that the sensor is now definitely valid
      sensorValid = true;

      double position;
      if (atLimit())
      {
        position = homeRotations;
      }
      else
      {
        // If the switch is initially false:
        // Set the position to the other endpoint
        if (homeAtMin)
        {
          position = maxRotations;
        } else {
          position = minRotations;
        }
      }
      m_Position.setPosition(position);
    }

    // Attempt calibrating once we have started moving
    if (active)
    {
      if (atLimit())
      {  
        if (!homeLastCycle && !calibrated)
        { 
          // When the sensor *becomes* true while not calibrated, set position without flagging as calibrated
          m_Position.setPosition(homeRotations);
        }
        // Flag when the sensor is true
        homeLastCycle = true;

        // If sensor is at endstop, stop
        if 
        (
          (homeAtMin && request.Position <= getAngle())
          ||
          (!homeAtMin && request.Position >= getAngle())
        ) stop();
      }
      else // if not at limit
      {
        // When the sensor first *becomes* false, calibrate
        if (homeLastCycle && !calibrated)
        {
          calibrated = true;
          m_Position.setPosition(homeRotations);
          if (safeSpeedValid)
            {m_Position.getConfigurator().apply(motorConfig);}
        }

        homeLastCycle = false;
      }
    }
  }
}

