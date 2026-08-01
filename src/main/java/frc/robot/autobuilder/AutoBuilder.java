package frc.robot.autobuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot.RobotState;
import frc.robot.util.PBDash;

/**
 * Dynamically creates an autonomous Command from an input string of instructions
 * @author 5985
 */
public class AutoBuilder 
{
  private final Parser parser;
  private final CommandGen commandGen;

  /**
   * Constructs the autobuilder, storing all the values that will be later needed
   * @param state
   */
  public AutoBuilder(RobotState state)
  {
    parser = new Parser();
    commandGen = new CommandGen(state);
  }

  /**
   * Compiles an auto string into a command
   * 
   * @param commandInput The auto string, comprised of instructions seperated by commas.
   *                     Each instruction is a name followed by arguments. Whitespace is ignored. 
   *                     For example, {@code driveto 1 2, waitfor 3, driveto 4 5 6}
   * @return A command that executes the auto string's instructions in sequence
   */
  public Command compile(String source)
  {
    // Wipe any previous errors
    PBDash.AUTO_ERRS.init();
    // Clear the field object we use to display the trajectory
    PBDash.removeFieldObject("Auto Path");
    // driveto 1 2 becomes [Instruction(driveto, [Value(Num, 1), Value(Num, 1)])]
    var instrs = parser.parse(source); 
    return commandGen.compile(instrs);
  }

  /** 
   * Appends the provided error message to {@link PBDash#AUTO_ERRS}, followed by a comma <p>
   * Uses a StringBuilder internally to optimise long, multi-part error messages 
   */
  protected static void error(Object... message) 
  {
    var builder = new StringBuilder();
    for (var msgPart : message) builder.append(msgPart);
    builder.append(", ");

    PBDash.AUTO_ERRS.append(builder.toString());
  }  
}
