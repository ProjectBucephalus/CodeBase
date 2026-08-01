package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.constants.FieldConstants.GeoFencing.*;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot.ClimbPosition;
import frc.robot.Robot.RobotState;
import frc.robot.Robot.ShootersState;
import frc.robot.constants.IDConstants;
import frc.robot.constants.Path;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.constants.Constants.ControlConstants;
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.Constants.IntakeConstants.ExtensionConstants;
import frc.robot.constants.FieldConstants.FieldTuning;
import frc.robot.constants.FieldConstants.GeoFencing;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.util.Conversions;
import frc.robot.util.FieldUtils;
import frc.robot.util.PBDash;
import frc.robot.controlTransmutation.Brake;
import frc.robot.controlTransmutation.JoystickTransmuter;
import frc.robot.subsystems.*;
import frc.robot.subsystems.generic.LinearExtension;
import frc.robot.subsystems.generic.PositionMotor;

public record ControlBinder
(
  RobotState state,
  //Supplier<Pose2d> poseSup,
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
  private static AtomicBoolean bound = new AtomicBoolean(false);
  private static Trigger switchboardConnected;
  private static Trigger shootZoneTrigger;
    
  public void bind()
  {
    if (bound.compareAndExchange(false, true)) return; // Guard against being called multiple times

    switchboardConnected = switchboard.button(1).or(switchboard.button(2)).or(switchboard.button(3));

    bindState();
    bindDrive();
  }

  private void bindState()
  {
    // Auto Score
    switchboard.button(IDConstants.shootHubSwitchID)
      .onChange(runOnce(() -> PBDash.IO_SHOOT_HUB.put(switchboard.button(IDConstants.shootHubSwitchID).getAsBoolean())).onlyIf(switchboardConnected).ignoringDisable(true));

    // Auto Pass
    switchboard.button(IDConstants.shootPassSwitchID)
      .onChange(runOnce(() -> PBDash.IO_SHOOT_PASS.put(switchboard.button(IDConstants.shootPassSwitchID).getAsBoolean())).onlyIf(switchboardConnected).ignoringDisable(true));

    // Fencing
    switchboard.button(IDConstants.fencingSwitchID)
      .onChange(runOnce(() -> PBDash.IO_FENCE.put(switchboard.button(IDConstants.fencingSwitchID).getAsBoolean())).onlyIf(switchboardConnected).ignoringDisable(true));

    // Vision
    switchboard.button(IDConstants.visionSwitchID)
      .onChange(runOnce(() -> PBDash.IO_LL.put(switchboard.button(IDConstants.visionSwitchID).getAsBoolean())).onlyIf(switchboardConnected).ignoringDisable(true));

    // Climber Wiggle
    switchboard.button(IDConstants.climbWiggleSwitchID).negate()
      .and(switchboardConnected)
      .onChange(runOnce(() -> PBDash.IO_CLIMB_WIGGLE.put(switchboard.button(IDConstants.climbWiggleSwitchID).getAsBoolean())).ignoringDisable(true));

    // Power Save
    new Trigger(PBDash.IO_POWER_DRIVE::get)
      .onTrue
      (runOnce(() -> {
        PBDash.IO_MAX_THROTTLE.put(ControlConstants.powerSaveThrottle);
        PBDash.IO_INTAKE_SPEED.put(IntakeConstants.RollerConstants.intakeMinSpeed);
      }))
      .onFalse
      (runOnce(() -> {
        PBDash.IO_MAX_THROTTLE.put(ControlConstants.maxThrottle);
        PBDash.IO_INTAKE_SPEED.put(IntakeConstants.RollerConstants.intakeMaxSpeed);
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
    if (bound.compareAndExchange(false, true)) return; // Guard against being called multiple times

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
