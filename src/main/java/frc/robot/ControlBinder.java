package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.constants.FieldConstants.GeoFencing.*;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot.ClimbPosition;
import frc.robot.Robot.RobotState;
import frc.robot.constants.IDConstants;
import frc.robot.constants.Constants.ControlConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.util.PBDash;
import frc.robot.controlTransmutation.Brake;
import frc.robot.controlTransmutation.JoystickTransmuter;
import frc.robot.subsystems.*;

public record ControlBinder
(
  RobotState state,
  CommandXboxController driver,
  CommandXboxController operator,
  CommandGenericHID switchboard,
  JoystickTransmuter driverStick,
  Brake driverBrake,
  CommandSwerveDrivetrain s_Swerve,
  Vision s_Vision,
  Limelight s_PhotonPort,
  Limelight s_PhotonStbd
)
{
  private static boolean bound = false;
  private static Trigger switchboardConnected;
    
  public void bind()
  {
    // Guard against being called multiple times
    if (bound) 
    {
      bound = false;
      return;
    }

    switchboardConnected = switchboard.button(1).or(switchboard.button(2)).or(switchboard.button(3));

    bindState();
    bindDrive();
  }

  private void bindState()
  {
    // Fencing
    switchboard.button(IDConstants.fencingSwitchID)
      .onChange(
        runOnce(() -> PBDash.IO_FENCE.put(switchboard.button(IDConstants.fencingSwitchID).getAsBoolean()))
        .onlyIf(switchboardConnected)
        .ignoringDisable(true)
      );

    // Vision
    switchboard.button(IDConstants.visionSwitchID)
      .onChange(
        runOnce(() -> PBDash.IO_LL.put(switchboard.button(IDConstants.visionSwitchID).getAsBoolean()))
        .onlyIf(switchboardConnected)
        .ignoringDisable(true)
      );

    // Power Save
    new Trigger(PBDash.IO_POWER_DRIVE::get)
      .onTrue(runOnce(() -> {
        PBDash.IO_MAX_THROTTLE.put(ControlConstants.powerSaveThrottle);
      }))
      .onFalse(runOnce(() -> {
        PBDash.IO_MAX_THROTTLE.put(ControlConstants.maxThrottle);
      }));
  }

  private void bindDrive()
  {
    // Nudging
    driver.b().onTrue(runOnce(() -> state.nudging = false).ignoringDisable(true));
    driver.a().onTrue(runOnce(() -> state.nudging = true).ignoringDisable(true));

    // Heading reset
    driver.start()
      .onTrue(runOnce(() -> s_Swerve.resetRotation(Rotation2d.kZero)).ignoringDisable(true));

    s_Swerve.setDefaultCommand(DriveBuilder.manual());

    // Bump nudging
    bumpTrigger
      .and(() -> state.nudging)
      .whileTrue(DriveBuilder.nonCardinal(bumpRotationTolerance));
    
    // Trench nudging
    trenchTrigger
      .and(() -> state.nudging)
      .whileTrue(DriveBuilder.trenchNudge());

    // Unlock heading
    driver.axisMagnitudeGreaterThan(XboxController.Axis.kRightX.value, ControlConstants.stickDeadband)
      .onTrue(s_Swerve.getDefaultCommand());

    // Climb heading lock
    driver.x()
      .and(() -> state.climbPos != ClimbPosition.None)
      .or(driver.povLeft())
      .or(driver.povRight())
      .onTrue
      (
        DriveBuilder.headingLocked
        (
          () -> switch (state.climbPos) 
          {
            case Left -> Rotation2d.kCCW_90deg;
            case Right -> Rotation2d.kCW_90deg;
            case None -> Rotation2d.kZero; // Shouldn't actually happen due to trigger condition
          }
        ).onlyWhile(() -> state.climbPos != ClimbPosition.None)
      );

    // Update throttle limits
    PBDash.IO_MAX_THROTTLE.asPulse()
      .onTrue(runOnce(() -> driverBrake.withMaxThrottle(PBDash.IO_MAX_THROTTLE.get())));
    PBDash.IO_MIN_THROTTLE.asPulse()
      .onTrue(runOnce(() -> driverBrake.withMinThrottle(PBDash.IO_MIN_THROTTLE.get())));
  }

  /** Mutually exclusive to {@link ControlBinder#bind bind()} */
  public void bindSysId()
  {
    // Guard against being called multiple times
    if (bound) 
    {
      bound = false;
      return;
    }

    s_Swerve.setDefaultCommand(DriveBuilder.manual());

    driver.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    driver.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

    /*
    * Joystick Y = quasistatic forward
    * Joystick A = quasistatic reverse
    * Joystick B = dynamic forward
    * Joystick X = dyanmic reverse
    */
    driver.y().whileTrue(s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    driver.a().whileTrue(s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    driver.b().whileTrue(s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
    driver.x().whileTrue(s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }
}
