package frc.robot.commands.swerve;

import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.FieldUtils;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class SwerveCommand extends Command
{
  protected final double deadband = Constants.Control.stickDeadband;

  protected CommandSwerveDrivetrain s_Swerve;

  protected Supplier<Translation2d> joystickSupplier;
  
  protected Translation2d motionXY;
  protected Translation2d robotXY;  //TODO used but not being set

  protected boolean redAlliance;

  /** Creates a new SwerveCommand. This has no rotation or drive-request methods or objects */
  public SwerveCommand(CommandSwerveDrivetrain s_Swerve, Supplier<Translation2d> joystickSupplier) 
  {
    this.s_Swerve = s_Swerve;
    
    this.joystickSupplier = joystickSupplier;

    addRequirements(s_Swerve);
  }

  @Override
  public void initialize() 
  {
    redAlliance = FieldUtils.isRedAlliance();

    initDriveConstraints();
  }

  /** Override to add additional initialization functionality */
  protected void initDriveConstraints() {}
}
