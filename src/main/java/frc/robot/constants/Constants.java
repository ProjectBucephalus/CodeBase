package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.util.Color;

public final class Constants 
{
  public static final class RumblerConstants 
  {
    public static final double driverDefault = 0.1;
    public static final double operatorDefault = 0.1;
  }

  public static final class Control
  {
    public static final double manualDiffectorDeadband = 0.25;
    public static final double stickDeadband = 0.15;
    /** Normal maximum robot speed, relative to maximum uncapped speed */
    public static final double maxThrottle = 0.5;
    /** Minimum robot speed when braking, relative to maximum uncapped speed */
    public static final double minThrottle = 0.3;
    /** Normal maximum rotational robot speed, relative to maximum uncapped rotational speed */
    public static final double maxRotThrottle = 1;
    /** Minimum rotational robot speed when braking, relative to maximum uncapped rotational speed */
    public static final double minRotThrottle = 0.5;
    /** Angle tolerance to consider something as "facing" the drivers, degrees */
    public static final double driverVisionTolerance = 5;
    /** Scalar for manual diffector elevation control */
    public static final double manualDiffectorElevationScalar = 2;
    /** Scalar for manual diffector rotation control */
    public static final double manualDiffectorRotationScalar = 2;
    /** Scalar for braking effect of diffector arm being higher than 1m */
    public static final double armBrakeRate = 1.5;
    public static final double manualClimberScale = 1;
    public static final double driveSnappingRange = 1.5;
    public static final double cageFaceDistance = 1.5;
    /** Translation lineup tolerance, in meters */
    public static final double lineupTolerance = 0.05;
    /** Rotation lineup tolerance, in degrees */
    public static final double angleLineupTolerance = 1.5;
  }

  public static final class Swerve
  {
    /** Centre-centre distance (length and width) between wheels, metres */
    public static final double drivebaseWidth = 0.485;
    public static final double initialHeading = 0;

    /* Drive PID Values */
    public static final double driveKP = 2.4;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;

    /* Rotation Control PID Values */
    public static final double rotationKP = 6;
    public static final double rotationKI = 0;
    public static final double rotationKD = 0;
    
    /* Rotation Control PID Values when holding Algae */
    public static final double rotationKPAlgae = 5;
    public static final double rotationKIAlgae = 0;
    public static final double rotationKDAlgae = 1;

    /* Swerve Limit Values */
    /** Meters per Second */
    public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    /** Radians per Second */
    public static final double maxAngularVelocity = 4;
  }

  public static final class Coral
  {
    public static final double forwardSpeed = -0.15;
    public static final double reverseSpeed = 0.15;
  }

  public static final class Vision
  {
    /* public static final int[] validIDs = 
    {
      //1, 2, 3,               // Red Human Player Stations
      //4, 5,                  // Red Barge
      //6, 7, 8, 9, 10, 11,      // Red Reef
      //12, 13, 16,            // Blue Human Player Stations
      //14, 15,                // Blue Barge
      17, 18, 19, 20, 21, 22   // Blue Reef
    };*/

    public static final int[] reefIDs = 
    {
      6, 7, 8, 9, 10, 11,    // Red Reef
      17, 18, 19, 20, 21, 22, // Blue Reef
      1, 2, 3,   // Red Human Player Stations
      12, 13, 16 // Blue Human Player Stations
    };

    public static final int[] bargeIDs = 
    {
      4, 5,  // Red Barge
      14, 15 // Blue Barge
    };

    public static final int[] humanPlayerStationIDs = 
    {
      1, 2, 3,   // Red Human Player Stations
      12, 13, 16 // Blue Human Player Stations
    };

    /** Baseline 1 meter, 1 tag stddev for x and y, in meters */
    public static final double linearStdDevBaseline = 0.08;
    /** Baseline 1 meter, 1 tag stddev rotation, in radians */
    public static final double rotStdDevBaseline = 999;
    /** How many good MT1 readings to get before setting rotation and moving to MT2 */
    public static final int mt1CyclesNeeded = 10;
  }

  public static final class LED 
  {
    /**
     * Constants used by the addressable LED classes.
     */
    /** # of LED's in the strip, if more than one strip daisy-chained, total # of LED's */
    public static final int lightsLen = 118; 
    /** default width for a partial display layer. a good number is about 1/4 lightsLen */
    public static final int viewWidth = 30;
    /** Start and end positions for Volaans displays */
    public static final int stbdStatusStart = 0; //86;
    public static final int stbdStatusWidth = 30; //30;
    public static final int portStatusStart = 83; //0;
    public static final int portStatusWidth = 30; //30;
    public static final int stbdHaloStart = 30; //58;
    public static final int stbdHaloWidth = 26; //28;
    public static final int portHaloStart = 56; //30;
    public static final int portHaloWidth = 27; //28;
    /** LED # at 0 degrees */
    public static final int startOffset = 1; 
    /** used to calculate the LED pointing in a particular direction */
    public static final double degreesPerLED = 360/lightsLen;  
    /** default background / off Color */
    public static final Color defaultBackColor = Color.kRed; 
    /** default foreground / on Color */
    public static final Color defaultFrontColor = Color.kGreen; 
    /** default Color for border dots */
    public static final Color displayBorderColor = Color.kBlue; 
    /** default # of segments in a status display */
    public static final int defaultStatusSegments = 3; 
    /** length of pointer layer above which a gradient will be applied rather than solid colour */
    public static final int pointerGradientThreshold = 5; 
    /** start and end LED #'s for the 'starboard' segment */
    public static final int stbdLEDsStart = 0; 
    public static final int stbdLEDsEnd = 57;
    /** start and end LED #'s for the 'port' segment */
    public static final int portLEDsStart = 58; 
    public static final int portLEDsEnd = 115;
    /** maximum colour layers in disco mode */
    public static final int discoMax = 10;  
    /** minimum colour layers in disco mode */
    public static final int discoMin = 3;  
    /**disco layers will randomly die of age between discoAgeLimit and 2x discoAgeLimit seconds */
    public static final int discoAgeLimit = 10; 
    /** the probability of accel or growthrate changing in any update is 1 - this: (1 - 0.9 = 0.1 = 10% chance of change) */
    public static final double discoChangeChance = 0.9; 
    /** used to calculate maxLen based on viewWidth. */
    public static final double discoMaxLenMultiplier = 0.3;
    /** used to calculate maxVel based on viewWidth, at maximum velocity it will take 1/this seconds to traverse the strip. */
    public static final double discoMaxVelMultiplier = 0.2; 
    /** used to calculate maxAccel, maxVel will be multiplied by this to get the value. */
    public static final double discoMaxAccelMultiplier = 0.02; 
    /** used to calculate maxGrow based on viewWidth. */
    public static final double discoMaxGrowMultiplier = 0.05; 
    /** multipied by maxGrow to get maxGrowRate */
    public static final double discoMaxGrowRateMultiplier = 0.01; 
    /** at least one color value (r,g,b) must be above this for the colour to be valid */
    public static final double discoColorThreshold = 0.5; 
  }
}
