List of available classes and brief descriptions of their functions

# Util #
AlliancePose2d/AllianceTranslation2d
- Wrapper on base geometry classes. 
- Input as Blue alliance version, use blue() or red() to get specific version
- Use get() to get alliance specific version, based on driverstation alliance

Conversions
- mod(value, base) returns mathematical modulus (default % doesn't handle negative correctly)
- wrap(int, min, max) returns value within endpoints inclusive (default only handles double)
- clamp() all variants allow limits in any order (default needs min,max)
  - int and double with endpoints
  - single input double clamps [-1..1]
  - input Translation2d clamps magnitude [0..1]
- round(value, places) returns value rounded to arbitrary precision
- buildPose() creates a new Pose2d from double values in metres/degrees
- atPose/Translation/Rotation() checks if input values are near eachother within standard tollerance (under ControlConstants)
- nearPose/Translation/Rotation() checks if input values are within input tollerances of eachother
- normaliseAngle(newAngle, currentAngle, maxAngle) for turret-style rotation pathing
  - Works for any range of motion. Max angle is symetric, e.g. max=180 -> range=[-180..180]
  - Favours the centre over shortest path if shortest is >135 degrees

FieldUtils
- alliance() various functions for getting and comparing the current alliance
- flipPose/Translation() mirrors pose over centreline
- rotatePose/Translation() rotates 180 around field centre
- inAllianceZone() checks if given position is fully within the defined alliance zone

Launchpad
- Interface for launchpad controller
- Predefined triggers for all buttons, can be used in the same way as normal controller buttons
- Can set colour to individual buttons, spans of buttons, or use predefined display grids

LockableXboxController
- Wrapper on CommandXboxController with a configurable button to unlock it with a rumble to confirm.
- All buttons read false and axis read 0 while controller is locked.
- Holding the unlock button for 1.5s locks the controller again, with a rumble to confirm.

MatchTime
- Time handler to convert ststem time to match time
- startAuto() to calibrate the timing of the Auto period
- startTele() to calibrate the timing of the Teleop period

PBDash
- Custom NetworkTable interface with configurable data keys
- Predefined keys can use any type, and are given a default value
  - Can be globally accessed by object name rather than by string
  - get() and put() automatically handle the associated data type
  - button() returns true when the value has changed then resets it
  - Various triggers can be set to monitor the value
- Generic access for Int, Double, Bool, and String, automatic default pushed on first use
- Various functions to add objects to the field render
  - putFieldObject() sets position of named object(s)
  - addToFieldObject() appends poses to existing object, with padding to ensure it renders as a trajectory
  - putFieldPath() renders a line between given points
  - trailFieldObject() renders a line of the last 50 poses given

# LEDs #
Block
- A defined string of LEDs within the whole strip, defined by length
- Has no internal reference to its position in the strip. Start position is given when render() is called
- Has option to render first 4 LEDs to network tables
- Patterns can be applied to sets of blocks simultaneously

Patterns
- dummy -> no render
- conditional -> takes any two patterns to render based on boolean condition
- supplied -> takes a colour supplier
- chase -> two colours of given lengths, move by one pixel at given frequency

# ControlTransmutation #
JoystickTransmuter
- 

Brake
- 

Deadband
- 

CrossDeadband
- 

InputCurve
- 

GeoFence
- 

TriggerObject
- 

# Subsystems #
CommandSwerveDrivetrain
- 

RumbleRequester
- 

LEDStrip
- 

Vision/Limelight
- 

BinaryMotor
- 

PositionMotor
- 

LimitedMotor/LinearExtension
- 

VelocityMotor
- 

BiMotor
- 

SpringMotor
- 

# AutoBuilder #
AutoBuilder
- Takes text input from dashboard and converts it into a chain of tokens which are then parsed to generate an auto command

CommandGen
- Contains the code to generate commands from input tokens
- Check TODO marks in ParsedRepr.java to add new instruction options to the list
- Check TODO marks in CommandGen.java to impliment new instruction options

