package frc.robot.autobuilder;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import frc.robot.DriveBuilder;
import frc.robot.Robot.RobotState;
import frc.robot.autobuilder.ParsedRepr.*;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Path;
import frc.robot.util.FieldUtils;
import frc.robot.util.PBDash;

/** Builds the final auto command from a list of {@link ParsedRepr.Instruction Instructions} */
public class CommandGen 
{
  /* Robot related values used by the produced command */
  private final RobotState state;

  /** The final command group that gets built from the instructions */
  private SequentialCommandGroup commands;
  /** The next instruction is to drive */
  private boolean nextIsDrive;
  /** Used to track the pose the robot will be at over the course of the auto. ALWAYS HANDLED AS BLUE ALLIANCE */
  private Pose2d currPose;

  /**
   * Creates a new command generator, storing all the provided robot values internally for use in the produced command
   * @param state The robot's state object
   */
  public CommandGen
  (
    RobotState state
  )
  {
    this.state = state;
  }

  /**
   * Runs the actual compilation process, compiling each instruction into a command and adding that to the final command group
   * @return The final auto command
   */
  public Command compile(List<Instruction> instrs)
  {
    commands = new SequentialCommandGroup();
    currPose = FieldUtils.allianceRotatePose(state.swerve.Pose);
    PBDash.putFieldObject("Auto Path", currPose);

    // Iterate over each instruction, calling a seperate function that handles the actual compilation logic and handling any errors that arise
    // This design means that the actual compilation logic is seperated from the error handling, and doesn't have to consider them
    for (int pos = 0; pos < instrs.size(); pos++)
    {
      var instr = instrs.get(pos);

      if (pos == instrs.size() - 1)
        nextIsDrive = false;
      else
        switch (instrs.get(pos + 1).type())
        {
          case driveto, driveby, follow -> nextIsDrive = true;
          default -> nextIsDrive = false;
        }

      try 
      {
        compileInstr(instr);
      }
      catch (TypeMismatchException e)
      {
        // Example output: "argument `false` of instruction 0 (`intake`) has type Text, expected Bool"
        AutoBuilder.error
        (
          "argument `", 
          e.found.value(), 
          "` of instruction ",
          pos + 1, 
          " (`", 
          instr.type(), 
          "`) has type ", 
          e.found.type(), 
          ", expected ", 
          e.expected
        );
      }
      catch (ArgCountException e) 
      {
        // Example output: "instruction 0 (`follow`) has 0 arguments, expected 1"
        AutoBuilder.error
        (
          "instruction ",
          pos + 1, 
          " (`", 
          instr.type(), 
          "`) has ",
          e.found,
          " arguments, expected ",
          e.expected
        );
      }
      catch (GeneralException e) 
      {
        AutoBuilder.error((Object[])e.msg);
      }
    }

    return commands.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  /**
   * Performs the actual compilation logic for each instruction type, seperate from the error handling done in {@link #compile()} <p>
   * 
   * Most of the errors are handled as exceptions, which are thrown by helper methods
   * and implicitly get rethrown by this to be caught in {@link #compile()}
   */
  private void compileInstr(Instruction instr) throws TypeMismatchException, ArgCountException, GeneralException
  {
    switch (instr.type())
    {
      // driveto x y r - Go to pose `x`, `y`, `r` (alliance origin relative). `r` optional, maintains current rotation if omitted
      case driveto -> compileDriveTo(instr);
      // driveby x y - Relative drive
      case driveby -> compileDriveBy(instr);
      // follow n - Follow the path with name `n` in Path.autoPaths
      case follow -> compileFollow(instr);
      // waitfor d - Wait for duration `d`
      case waitfor, wait -> 
      {
        instr.assertArgCount(1);

        commands.addCommands(DriveBuilder.waitCommand().withTimeout(instr.arg(0).asNum()));
      }
      // waituntil d - wait until time `d`
      case waituntil -> 
      {
        instr.assertArgCount(1);

        double duration = instr.arg(0).asNum();
        commands.addCommands(DriveBuilder.waitCommand().until(() -> Timer.getMatchTime() < (15 - duration)));
      }
      // passing b - sets auto passing on or off based on `b`
      case passing -> 
      {
        instr.assertArgCount(1);

        boolean passState = instr.arg(0).asBool();
        commands.addCommands(Commands.runOnce(() -> PBDash.IO_SHOOT_PASS.put(passState)));
      }
    }
  }

  private void compileDriveTo(Instruction instr) throws TypeMismatchException, ArgCountException 
  {
    Rotation2d rotationTarget;
    // Some extra handling is required due to the optional argument
    if (instr.args().length > 2) 
    {
      instr.assertArgCount(3);
      rotationTarget = Rotation2d.fromDegrees(instr.arg(2).asNum());
    }
    else
    {
      instr.assertArgCount(2);
      rotationTarget = currPose.getRotation();
    }

    // Clamp the target pose to at least half a meter from the field walls and the midline for safety and to handle mis-inputs
    // If either value changes as a result of this clamping, we provide a warning but still continue
    double xArg = instr.arg(0).asNum();
    double x = MathUtil.clamp(xArg, 0.5, (FieldConstants.fieldCentre.getX()) - 0.5);
    if (x != xArg) {AutoBuilder.error("warning: x = `", xArg, "` was clamped to `", x, "`");}
    double yArg = instr.arg(1).asNum();
    double y = MathUtil.clamp(yArg, 0.5, FieldConstants.fieldWidth - 0.5);
    if (y != yArg) {AutoBuilder.error("warning: y = `", yArg, "` was clamped to `", y, "`");}

    currPose = new Pose2d(new Translation2d(x, y), rotationTarget);
    Pose2d targetPose = FieldUtils.allianceRotatePose(currPose);

    PBDash.addToFieldObject("Auto Path", targetPose);
    // All prior handling was done using a blue alliance origin pose, and we now rotate the pose to match our actual alliance
    commands.addCommands(DriveBuilder.pathFollow(targetPose, nextIsDrive));
  }

  private void compileDriveBy(Instruction instr) throws TypeMismatchException, ArgCountException 
  {
    instr.assertArgCount(2);

    Translation2d offset = new Translation2d(instr.arg(0).asNum(), instr.arg(1).asNum());
    currPose = new Pose2d(currPose.getTranslation().plus(offset), currPose.getRotation());
    // All prior handling was done using a blue alliance origin pose, and we now rotate the pose to match our actual alliance
    Pose2d targetPose = FieldUtils.allianceRotatePose(currPose);

    PBDash.addToFieldObject("Auto Path", targetPose);

    commands.addCommands(DriveBuilder.pathFollow(targetPose, nextIsDrive));
  }

  private void compileFollow(Instruction instr) throws TypeMismatchException, ArgCountException, GeneralException
  {
    instr.assertArgCount(1);

    var pathName = instr.arg(0).asText();
    var path = Path.autoPaths.get(pathName);

    if (path == null) throw new GeneralException("no path `" + pathName + "`");

    currPose = path.targetPose();

    var alliancePath = path.allianceRotated();
    alliancePath.display("Auto Path");
    commands.addCommands(DriveBuilder.pathFollow(alliancePath, nextIsDrive));
  }
}
