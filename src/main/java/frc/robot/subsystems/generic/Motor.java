package frc.robot.subsystems.generic;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/*
 * void setTarget(double)
 * void setVelocity(double)
 * void setPower(double)
 * 
 * void stop()
 * 
 * Command setTargetCmd(DoubleSupplier)
 * Command gotoTargetCmd(DoubleSupplier) // waits for it to reach the target
 * Command adjustTargetCmd(DoubleSupplier) // continuously adjusts the target point
 * 
 * Command setSpeedCmd(DoubleSupplier)
 * Command runCmd(DoubleSupplier) // sets speed and stops when the command ends
 * Command setPowerCmd(DoubleSupplier)
 */

/** 
 * A subsystem wrapped around a TalonFX to provide a simple interface for controlling a motor with mechanism ratio
 * @author 5985
 */
@Logged(strategy = Strategy.OPT_IN)
public class Motor extends SubsystemBase 
{
  enum ControlType { Position, Velocity }

  protected final TalonFX m_Motor;

  protected final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
  protected final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0);
  protected final DutyCycleOut powerRequest = new DutyCycleOut(0);

  private ControlType lastControlType;
  
  private final DCMotorSim motorSim;
  private final double motorGearRatio;

  /**
   * Creates a wrapper around a TalonFX to provide velocity control
   * 
   * @param id the id of the motor
   * @param config the config to apply to the wrapped motor
   */
  public Motor(int id, TalonFXConfiguration config) 
  {
    m_Motor = new TalonFX(id);

    motorGearRatio = config.Feedback.SensorToMechanismRatio * config.Feedback.RotorToSensorRatio;
    motorSim = new DCMotorSim
    (
      LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.1, motorGearRatio),
      DCMotor.getKrakenX60(1)
    );

    if (RobotBase.isSimulation())
    {
      config = config.clone();
      config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    }

    m_Motor.getConfigurator().apply(config);
  }

  /**
   * Sets the target point for the motor 
   * @param target position in mechanism rotations
   */
  public void setTarget(double pos)
  {
    lastControlType = ControlType.Position;
    m_Motor.setControl(positionRequest.withPosition(pos));
  }

  /**
   * Set the speed of the motor
   * 
   * @param speed the desired speed, in mechanism rotations per second
   */
  public void setSpeed(double speed)
  {
    lastControlType = ControlType.Velocity;
    m_Motor.setControl(velocityRequest.withVelocity(speed));
  }

  /** 
   * Set the power of the motor, as a percentage of the motor's supply voltage.
   * 
   * @param power the power
   */
  public void setPower(double power)
  {       
    if (power == 0.0)
    {
      switch (lastControlType)
      {
        case Position -> m_Motor.setControl(positionRequest.withPosition(getPosition()));
        case Velocity -> m_Motor.setControl(velocityRequest.withVelocity(0.0));
      }
    }
    else 
    {
      m_Motor.setControl(powerRequest.withOutput(power));
    }
  }

  /** Stops the motor */
  public void stop()
    {m_Motor.set(0);}

  /**
   * Creates a command to set the target point for the motor when scheduled <p>
   * @param targetSup mechanism rotations, supply a target position
   * @return the Command that sets the target once
   */
  public Command setTargetCmd(DoubleSupplier targetSup)
    {return runOnce(() -> setTarget(targetSup.getAsDouble()));}

  /**
   * Creates a command to set the target point for the motor and wait until we reach that target point <p>
   * @param targetSup mechanism rotations
   * @return the Command
   */
  public Command gotoTargetCmd(DoubleSupplier targetSup)
    {return setTargetCmd(targetSup).andThen(Commands.waitUntil(this::atTarget));}

  /**
   * Creates a command to continuously adjust the target point of the motor by a dynamic amount <p>
   * Primarily intended for joystick control
   * @param shiftSup A supplier for the amount to adjust the target by in mechanism rotations
   * @return the Command
   */
  public Command adjustTargetCmd(DoubleSupplier shiftSup) 
    {return run(() -> {if (shiftSup.getAsDouble() != 0) setTarget(getPosition() + shiftSup.getAsDouble());});}
  

  /**
   * Construct a command that sets the speed of the motor <p>
   * 
   * @param speedSup A supplier providing the desired speed, in mechanism rotations per second
   * @return the {@link Command}
   */
  public Command setSpeedCmd(DoubleSupplier speedSup)
    {return runOnce(() -> setSpeed(speedSup.getAsDouble()));}

  public Command runCmd(DoubleSupplier speedSup)
    {return runEnd(() -> setSpeed(speedSup.getAsDouble()), () -> setSpeed(0));}

  /** 
   * Set the power of the motor, as a percentage of the motor's supply voltage.
   * 
   * @param power the power
   */
  public Command setPowerCmd(DoubleSupplier powerSup)
    {return runOnce(() -> setPower(powerSup.getAsDouble()));}
  
  /** @return Current position of the motor, in mechanism rotations */
  @Logged(name = "position Rotations")
  public double getPosition() 
    {return m_Motor.getPosition().getValue().in(Units.Rotations);}

  /** @return Current target position of the motor, in mechanism rotations */
  @Logged(name = "target Rotations")
  public double getTarget() 
    {return positionRequest.Position;}

  /** @return Current speed of the motor, in mechanism rotations per second */
  @Logged(name = "speed RevPerSec")
  public double getSpeed() 
    {return m_Motor.getVelocity().getValue().in(Units.RotationsPerSecond);}

  @Logged(name = "power")
  public double getPower() 
    {return m_Motor.getClosedLoopOutput().getValue();}

  /** @return {@code true} if all CAN devices are connected */
  public boolean devicesValid()
    {return m_Motor.isConnected();}

  /** @return {@code true} when the motor is close to target */
  public boolean atTarget()
    {return MathUtil.isNear(getTarget(), getPosition(), 0.1);}

  @Override
  public void simulationPeriodic()
  {
    var motorSimState = m_Motor.getSimState();
    motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // get the motor voltage of the TalonFX
    var motorVoltage = motorSimState.getMotorVoltageMeasure();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    motorSim.setInputVoltage(motorVoltage.in(Units.Volts));
    motorSim.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    motorSimState.setRawRotorPosition(motorSim.getAngularPosition().times(motorGearRatio));
    motorSimState.setRotorVelocity(motorSim.getAngularVelocity().times(motorGearRatio));
  }
}
