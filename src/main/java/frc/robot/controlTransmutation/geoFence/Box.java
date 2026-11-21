package frc.robot.controlTransmutation.geoFence;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Conversions;
import frc.robot.util.PBDash;

import static frc.robot.constants.FieldConstants.GeoFencing.*;

/**
 * A rectangle shaped {@link GeoFence} </p>
 * A cardinal rectangular region defined by two corners
 * @author 5985
 */
public class Box extends GeoFence
{
  private double xA;
  private double yA;
  private double xB;
  private double yB;

  /**
   * Rectangle shaped GeoFence
   * @param xA X-coordinate of the first point
   * @param yA Y-coordinate of the first point
   * @param xB X-coordinate of the second point
   * @param yB Y-coordinate of the second point
   * @param radius Extra radius of geofence zone around the box (produces a rounded rectangle shape)
   * @param buffer Buffer around the object over which the speed is reduced
   */
  public Box(double xA, double yA, double xB, double yB, double radius, double buffer)
  {
    this.xA = Math.min(xA, xB);
    this.yA = Math.min(yA, yB);
    this.xB = Math.max(xA, xB);
    this.yB = Math.max(yA, yB);

    this.radius = radius;
    this.buffer = buffer;

    centre = new Translation2d((xA + xB)/2, (yA + yB)/2);

    checkRadius = (Math.hypot(xB - xA, yB - yA)/2) + radius + buffer;
  }

  /**
   * Rectangle shaped GeoFence with minimum radius and buffer
   * @param xA X-coordinate of the first point
   * @param yA Y-coordinate of the first point
   * @param xB X-coordinate of the second point
   * @param yB Y-coordinate of the second point
   */
  public Box(double xA, double yA, double xB, double yB)
  {
    this(xA, yA, xB, yB, minRadius, minBuffer);
  }

  @Override
  public double getDistance(Translation2d testPos)
  {
    double distance = 0;

    if (testPos.getX() < xA)
    {
      if (testPos.getY() < yA) // SW Corner
        {distance = Math.hypot(xA - testPos.getX(), yA - testPos.getY());}
      else if (testPos.getY() > yB) // NW Corner
        {distance = Math.hypot(xA - testPos.getX(), testPos.getY() - yB);}
      else // W Cardinal
        {distance = xA - testPos.getX();}
    }
    else if (testPos.getX() > xB)
    {
      if (testPos.getY() < yA) // SE Corner
        {distance = Math.hypot(testPos.getX() - xB, yA - testPos.getY());}
      else if (testPos.getY() > yB) // NE Corner
        {distance = Math.hypot(testPos.getX() - xB, testPos.getY() - yB);}
      else // E Cardinal
        {distance = testPos.getX() - xB;}
    }
    else
    {
      // Inside, S Cardinal, or N Cardinal
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

    // Account for radius
    return distance - radius;
  }

  @Override
  protected Translation2d dampMotion(Translation2d motionXY)
  {
    double motionX = motionXY.getX();
    double motionY = motionXY.getY();
    double distanceToEdge;

    if (robotPos.getX() < xA)
    {
      if (robotPos.getY() < yA) // SW Corner
        {return pointDamping(xA, yA, motionXY);}
      else if (robotPos.getY() > yB) // NW Corner
        {return pointDamping(xA, yB, motionXY);}
      else // W Cardinal
      {
        distanceToEdge = (xA - radius) - (robotPos.getX() + robotRadius);
        motionX = Math.min(motionX, (Conversions.clamp(distanceToEdge, 0, buffer)) / buffer);
      }
    }
    else if (robotPos.getX() > xB)
    {
      if (robotPos.getY() < yA) // SE Corner
        {return pointDamping(xB, yA, motionXY);}
      else if (robotPos.getY() > yB) // NE Corner
        {return pointDamping(xB, yB, motionXY);}
      else // E Cardinal
      {
        distanceToEdge = (robotPos.getX() - robotRadius) - (xB + radius);
        motionX = Math.max(motionX, (-Conversions.clamp(distanceToEdge, 0, buffer)) / buffer);
      }
    }
    else 
    {
      if (robotPos.getY() < yA) // S Cardinal
      {
        distanceToEdge = (yA - radius) - (robotPos.getY() + robotRadius);
        motionY = Math.min(motionY, (Conversions.clamp(distanceToEdge, 0, buffer)) / buffer);
      } 
      else if (robotPos.getY() > yB) // N Cardinal
      {
        distanceToEdge = (robotPos.getY() - robotRadius) - (yB + radius);
        motionY = Math.max(motionY, (-Conversions.clamp(distanceToEdge, 0, buffer)) / buffer);
      }
      else // Center (you've met a terrible fate *insert kazoo music here*)
        {return pointDamping(centre.getX(), centre.getY(), motionXY);}
    }

    // Check for collision
    if (distanceToEdge <= 0.05)
    {
      touchingObject = true; 
    } 

    return new Translation2d(motionX, motionY);
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