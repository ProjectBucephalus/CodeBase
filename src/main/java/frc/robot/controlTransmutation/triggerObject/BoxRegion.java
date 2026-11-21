package frc.robot.controlTransmutation.triggerObject;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.PBDash;

import static frc.robot.constants.FieldConstants.GeoFencing.*;

/** 
 * A rectangle shaped {@link TriggerRegion}
 * @author 5985
 */
public class BoxRegion extends TriggerRegion 
{
  private double xA;
  private double yA;
  private double xB;
  private double yB;

  /**
   * Rectangle shaped region
   * @param xA X-coordinate of the first point
   * @param yA Y-coordinate of the first point
   * @param xB X-coordinate of the second point
   * @param yB Y-coordinate of the second point
   * @param radius Extra radius of region around the box (produces a rounded rectangle shape)
   * @param buffer Buffer around the object over which the speed is reduced
   */
  public BoxRegion(double xA, double yA, double xB, double yB, double radius, double buffer)
  {
    super(new Translation2d((xA + xB)/2, (yA + yB)/2), radius, buffer);

    this.xA = Math.min(xA, xB);
    this.yA = Math.min(yA, yB);
    this.xB = Math.max(xA, xB);
    this.yB = Math.max(yA, yB);

    checkRadius = (Math.hypot(xB - xA, yB - yA)/2) + radius + buffer;
  }

  /**
   * Rectangle shaped region with minimum radius and buffer size
   * @param xA X-coordinate of the first point
   * @param yA Y-coordinate of the first point
   * @param xB X-coordinate of the second point
   * @param yB Y-coordinate of the second point
   */
  public BoxRegion(double xA, double yA, double xB, double yB)
    {this(xA, yA, xB, yB, minRadius, minBuffer);}

  @Override
  public double getDistance(Translation2d testPos)
  {
    double distance = 0;

    if (testPos.getX() < xA)
    {
      if (testPos.getY() < yA)
        {distance = Math.hypot(xA - testPos.getX(), yA - testPos.getY());}
      else if (testPos.getY() > yB)
        {distance = Math.hypot(xA - testPos.getX(), testPos.getY() - yB);}
      else 
        {distance = xA - testPos.getX();}
    }
    else if (testPos.getX() > xB)
    {
      if (testPos.getY() < yA)
        {distance = Math.hypot(testPos.getX() - xB, yA - testPos.getY());}
      else if (testPos.getY() > yB)
        {distance = Math.hypot(testPos.getX() - xB, testPos.getY() - yB);}
      else
        {distance = testPos.getX() - xB;}
    }
    else
    {
      distance = Math.max
      (
        Math.max
        (
          xA - testPos.getX(), 
          yA - testPos.getY()
        ),
        Math.max
        (
          testPos.getX() - xB, 
          testPos.getY() - yB
        )
      );
    }

    return distance - radius;
  }

  @Override
  public void display() 
  {
    var points = new Translation2d[] 
    {
      // Bottom left
      new Translation2d(xA - radius, yA),
      new Translation2d(xA, yA - radius),
      // Bottom right
      new Translation2d(xB, yA - radius),
      new Translation2d(xB + radius, yA),
      // Top right
      new Translation2d(xB + radius, yB),
      new Translation2d(xB, yB + radius),
      // Top left
      new Translation2d(xA, yB + radius),
      new Translation2d(xA - radius, yB),
      // Bottom left
      new Translation2d(xA - radius, yA),
      new Translation2d(xA, yA - radius),
    };

    PBDash.putFieldObject(displayName, points);
  }
}
