package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Constant values for mechanism geometry, motor configuration, input parameters, targets, etc. <p>
 * @Note For coordinate definitions, see Robot
 * <li> When possible, all units are Metres and Degrees (right-hand rotation)
 * <li> Gear ratios are defined by Teeth-Out/Teeth-In (except for pre-made gearboxes)
 * 
 * @author 5985
 */
public final class Constants 
{
  public static final class RumblerConstants 
  {
    public static final double driverDefault = 0.1;
    public static final double operatorDefault = 0.1;
  }

  /** Values for controller input and general driving behaviours */
  public static final class ControlConstants
  {
    public static final double stickDeadband = 0.15;
    /** Normal maximum robot speed, relative to maximum uncapped speed */
    public static final double maxThrottle = 0.8;
    /** Minimum robot speed when braking, relative to maximum uncapped speed */
    public static final double minThrottle = 0.2;
    /** Normal maximum rotational robot speed, relative to maximum uncapped rotational speed */
    public static final double maxRotThrottle = 0.8;
    /** Minimum rotational robot speed when braking, relative to maximum uncapped rotational speed */
    public static final double minRotThrottle = 0.3;
    /** How far a trigger must be pressed to be considered on, [0..1] */
    public static final double triggerThreshold = 0.8;
    /** Translation lineup tolerance, meters */
    public static final double lineupTolerance = 0.1;
    /** Rotation lineup tolerance, degrees */
    public static final double angleLineupTolerance = 4;

    /** Maximum robot speed to reduce power consumption, relative to maximum uncapped speed */
    public static final double powerSaveThrottle = 0.5;

    /** Attractor minimum angle tolerance, degrees */
    public static final double minAngleTolerance = 20;
    /** Attractor maximum angle tolerance, degrees */
    public static final double maxAngleTolerance = 60;

    public static final double manualControlDeadband = 0.25;

    @SuppressWarnings("unchecked") // No way to make it work that doesn't give warning afaik
    /** First element is the default */
    public static final Pair<String, String>[] autoPresets = new Pair[]
    {
      new Pair<>("Blank", "")
    };
  }

  /** Geometry and tuning data for drivebase */
  public static final class SwerveConstants
  {
    /** Forward offset between the centre of the drivebase and Robot coordinate origin, metres */
    public static final double drivebaseOffset = 0.095;
    /** Offset from typical centre of rotation to centre of drivebase, metres Fore/Port */
    public static final Translation2d retractedCentreOffset = new Translation2d(-drivebaseOffset,0);
    /** Centre-centre distance between wheels port-stbd, metres */
    public static final double drivebaseWidth = 0.56;
    /** Centre-centre distance between wheels fore-aft, metres */
    public static final double drivebaseLength = drivebaseWidth;
    /** Wheel-centre to Robot-centre distance to Port wheels, metres */
    public static final double wheelPortY = drivebaseWidth/2;
    /** Wheel-centre to Robot-centre distance to Stbd wheels, metres */
    public static final double wheelStbdY = -drivebaseWidth/2;
    /** Wheel-centre to Robot-centre distance to Fore wheels, metres */
    public static final double wheelForeX = (drivebaseLength/2) - drivebaseOffset;
    /** Wheel-centre to Robot-centre distance to Aft wheels, metres */
    public static final double wheelAftX = (-drivebaseLength/2) - drivebaseOffset;

    public static final double initialHeading = 0;

    /* Auto Drive PID Values, Meters */
    public static final double driveKP = 2.25;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;

    /* Auto Rotation PID Values, Degrees */
    public static final double rotationKP = 2.5;
    public static final double rotationKI = 0.0;
    public static final double rotationKD = 0.03;

    /* Swerve Limit Values */
    /** Mechanical maximum staright-line robot speed, Meters per Second */
    public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    /** Mechanical maximum robot rotation rate, Radians per Second */
    public static final double maxAngularVelocity = 4;

    /** Radius from robot centre in metres where geofence is triggered for slow movements */
    public static final double robotRadiusInscribed = 0.42;
    /** Radius from robot centre in metres where geofence is triggered for fast movements */
    public static final double robotRadiusCircumscribed = 0.6;
    /** Radius enclosing robot when extended in metres where geofence is triggered for most movement */
    public static final double robotRadiusExpanded = 0.66;
    /** Speed threshold at which the robot changes between radii, m/s */
    public static final double robotSpeedThreshold = 1.5;
  }

  /** Geometry, tag, and tuning data for Vision system */
  public static final class VisionConstants
  {
    /** X/Y offset of camera in turret space, metres aft/stbd */
    public static final Transform2d flatCameraToTurret = new Transform2d(-0.156, 0, Rotation2d.kZero);
    private static final Translation2d baseTurretToCamera = new Translation2d(0.156, 0.195);
    /** Pitch of camera in turret space, degrees */
    private static final double turretPitch = -12.9;
    /** X/Z offset of camera in camera space, metres fore/up */
    private static final Translation2d turretToCamera = baseTurretToCamera.rotateBy(Rotation2d.fromDegrees(turretPitch));

    /** 3D offset from centre of rotation of turret at floor level to centre of camera lens, metres fore*2/port/down, degrees roll/pitch/yaw */
    public static final Transform3d portLimelightOffset = new Transform3d(turretToCamera.getX(), 0, turretToCamera.getY(), new Rotation3d(0, Math.toRadians(turretPitch), 0));
    /** 3D offset from centre of rotation of turret at floor level to centre of camera lens, metres fore*2/port/down, degrees roll/pitch/yaw */
    public static final Transform3d stbdLimelightOffset = new Transform3d(turretToCamera.getX(), 0, turretToCamera.getY(), new Rotation3d(0, Math.toRadians(turretPitch), 0));
    /** Maximum time between vision estimates before switching to odometry only, seconds */
    public static final double visionFrequencyThreshold = 2;
    /** How many seconds into the past we store turret azimuth readings */
    public static final double azimuthBufLength = 10;

    /** Baseline 1 meter, 1 tag stddev for x and y, meters */
    public static final double linearStdDevBaseline = 0.2;
    /** Baseline 1 meter, 1 tag stddev rotation, radians */
    public static final double rotStdDevBaseline = Math.toRadians(40);
  }
}
