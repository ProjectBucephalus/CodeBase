package frc.robot.controlTransmutation.geoFence;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Conversions;
import frc.robot.util.PBDash;

import static frc.robot.constants.FieldConstants.GeoFencing.*;

/**
 * A wall shaped {@link GeoFence} </p>
 * The outer wall that the robot must stay within <p>
 * A cardinal rectangular region defined by two corners
 * @author 5985
 */
public class Fence extends GeoFence
{
  private double xA;
  private double yA;
  private double xB;
  private double yB;

  /**
   * Wall shaped GeoFence
   * @param xA X-coordinate of the first point
   * @param yA Y-coordinate of the first point
   * @param xB X-coordinate of the second point
   * @param yB Y-coordinate of the second point
   * @param radius Extra radius of geofence zone on the inner walls of the fence
   * @param buffer Buffer within the walls over which the speed is reduced
   */
  public Fence(double xA, double yA, double xB, double yB, double radius, double buffer)
  {
    this.xA = Math.min(xA, xB);
    this.yA = Math.min(yA, yB);
    this.xB = Math.max(xA, xB);
    this.yB = Math.max(yA, yB);

    this.radius = radius;
    this.buffer = buffer;

    centre = new Translation2d((xA + xB)/2, (yA + yB)/2);

    checkRadius = radius + buffer;
  }

  /**
   * Wall shaped GeoFence with minimum radius and buffer
   * @param xA X-coordinate of the first point
   * @param yA Y-coordinate of the first point
   * @param xB X-coordinate of the second point
   * @param yB Y-coordinate of the second point
   */
  public Fence(double xA, double yA, double xB, double yB)
  {
    this(xA, yA, xB, yB, minRadius, minBuffer);
  }

  @Override
  public double getDistance(Translation2d testPos)
  {
    return Math.abs
    (
      Math.min
      (
        Math.min(testPos.getX() - (xA + radius), (xB - radius) - testPos.getX()),
        Math.min((yB - radius) - testPos.getY(), testPos.getY() - (yA + radius))
      )
    );
  }

  @Override
  protected boolean checkPosition()
  {
    return
    (
      (robotPos.getX() >= xB - (checkRadius + robotRadius)) ||  // Close to inside of +X barrier
      (robotPos.getX() <= xA + (checkRadius + robotRadius)) ||  // Close to inside of -X barrier
      (robotPos.getY() >= yB - (checkRadius + robotRadius)) ||  // Close to inside of +Y barrier
      (robotPos.getY() <= yA + (checkRadius + robotRadius))     // Close to inside of -Y barrier
    );
  }

  @Override
  protected Translation2d dampMotion(Translation2d motionXY)
  {
    // Calculates distance to the relevant edge of the field
    // Calculates edge position, and subtracts robot position + radius from edge position.

    // Sets the motion in the relevant direction to the minimum of the current motion
    // And the distance from the edge clamped between 0 and the edge buffer, and normalised to a maximum of 1.
    // This ensures the motion in that direction does not go above the clamped + normalised distance from the edge, to cap speed.
    
    double motionX = motionXY.getX();
    double motionY = motionXY.getY();

    // Holding values initialised as non-zero to allow for collision check
    double distanceToEdgeX = 1;
    double distanceToEdgeY = 1;
    
    if (motionX > 0)
    {   
      distanceToEdgeX = (xB - radius) - (robotPos.getX() + robotRadius); 
      motionX = Math.min(motionX, (Conversions.clamp(distanceToEdgeX, 0, buffer)) / buffer);
    }
    else if (motionX < 0)
    {   
      distanceToEdgeX = (robotPos.getX() - robotRadius) - (xA + radius);
      motionX = Math.max(motionX, (-Conversions.clamp(distanceToEdgeX, 0, buffer)) / buffer);
    }

    if (motionY > 0)
    {   
      distanceToEdgeY = (yB - radius) - (robotPos.getY() + robotRadius);
      motionY = Math.min(motionY, (Conversions.clamp(distanceToEdgeY, 0, buffer)) / buffer);
    }
    else if (motionY < 0)
    {   
      distanceToEdgeY = (robotPos.getY() - robotRadius) - (yA + radius);
      motionY = Math.max(motionY, (-Conversions.clamp(distanceToEdgeY, 0, buffer)) / buffer);
    }

    // Check for collision
    if (distanceToEdgeX <= 0.05 || distanceToEdgeY <= 0.05)
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
      new Translation2d(xA, yA),
      // Bottom right
      new Translation2d(xB, yA),
      // Top right
      new Translation2d(xB, yB),
      // Top left
      new Translation2d(xA, yB),
      // Bottom left
      new Translation2d(xA, yA),
    };

    PBDash.putFieldObject(displayName, points);
  }
}