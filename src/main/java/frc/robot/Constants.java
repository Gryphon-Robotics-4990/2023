package frc.robot;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.ArmFeedforward;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

import edu.wpi.first.wpilibj.SPI;

import frc.robot.units.*;

//-1 usually means not yet set

public final class Constants {
    public static class Ports {
        //Laptop ports
        public static int PORT_JOYSTICK_DRIVE = 2;
        public static int PORT_JOYSTICK_OPERATOR = 1;

        //CAN Bus IDs
        public static int CAN_DRIVETRAIN_LEFT_FRONT_VICTOR = 1;
        public static int CAN_DRIVETRAIN_LEFT_REAR_TALONSRX = 15;
        public static int CAN_DRIVETRAIN_RIGHT_FRONT_VICTOR = 3;
        public static int CAN_DRIVETRAIN_RIGHT_REAR_TALONSRX = 11;

        public static int CAN_INTAKE_SPARKMAX = 8;

        public static int CAN_ARM_LEFT_SPARKMAX = 4;
        public static int CAN_ARM_RIGHT_SPARKMAX = 2;    

        public static SPI.Port SPI_PORT_GYRO = SPI.Port.kMXP;
        
        //Sensors
        public static int DIO_FRONT_LIMIT_SWITCH = 8;
        public static int DIO_BACK_LIMIT_SWITCH = 9;
    }

    public static class MotorConfig {
        //Talon information
        public static double TALON_ENCODER_RESOLUTION = 4096; // = EPR = CPR
        public static int TALON_TIMEOUT_MS = 5;
        public static int TALON_DEFAULT_PID_ID = 0;//0 is primary, 1 is auxilary
        public static TalonSRXFeedbackDevice TALON_DEFAULT_FEEDBACK_DEVICE = TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative;
        public static FollowerType DEFAULT_MOTOR_FOLLOWER_TYPE = FollowerType.PercentOutput;
    }

    public static class RobotMeasurements {
        public static double DRIVETRAIN_WHEEL_RADIUS_IN = 3.0;
        //public static double DRIVETRAIN_WHEEL_RADIUS_METERS = DRIVETRAIN_WHEEL_RADIUS_IN * Units.INCH.to(Units.METER);
        public static double DRIVETRAIN_WHEEL_RADIUS_METERS = DRIVETRAIN_WHEEL_RADIUS_IN * (0.0254);

        // Height of camera off the ground
        public static double CAMERA_HEIGHT_METERS = 1.3081;
        // Pitch of camera on robot (probably 0 if we mount it properly)
        //public static double CAMERA_PITCH_RADIANS = 0.00872665;
        public static double CAMERA_PITCH_RADIANS = 0;
        // Height of the AprilTag off the ground (find from field measurements)
        // It's 1.93 right now because that was the height of the tag when I tested it (but it results in negative distances)
        public static double TARGET_HEIGHT_METERS = 1.3716;
        public static double ARM_MOTION_REDUCTION = 288.0;

    }
    
    public static class Units {
        //Base units
        public static Unit METER = new BaseUnit(Dimension.Length, 1d);
        public static Unit KILOMETER = new BaseUnit(Dimension.Length, METER.getScalar() * 1000d);
        public static Unit FEET = new BaseUnit(Dimension.Length, METER.getScalar() * 3.280839895d);
        public static Unit INCH = new BaseUnit(Dimension.Length, FEET.getScalar() / 12d);

        public static Unit SECOND = new BaseUnit(Dimension.Time, 1d);
        public static Unit MINUTE = new BaseUnit(Dimension.Time, SECOND.getScalar() * 60d);
        public static Unit HOUR = new BaseUnit(Dimension.Time, MINUTE.getScalar() * 60d);
        public static Unit MILLISECOND = new BaseUnit(Dimension.Time, SECOND.getScalar() / 1000d);
        public static Unit ENCODER_TIME = new BaseUnit(Dimension.Time, MILLISECOND.getScalar() * 100d);
        
        public static Unit KILOGRAM = new BaseUnit(Dimension.Mass, 1d);

        public static Unit RADIAN = new BaseUnit(Dimension.Angle, 1d);
        public static Unit REVOLUTION = new BaseUnit(Dimension.Angle, RADIAN.getScalar() * 2d * Math.PI);
        public static Unit DEGREE = new BaseUnit(Dimension.Angle, REVOLUTION.getScalar() / 360d);
        // Angle represented by encoder ticks, i.e 4096 ticks is a full revolution
        public static Unit ENCODER_ANGLE = new BaseUnit(Dimension.Angle, REVOLUTION.getScalar() / MotorConfig.TALON_ENCODER_RESOLUTION);

        public static Unit AMPERE = new BaseUnit(Dimension.Current, 1d);

        //Compound units
        public static Unit ENCODER_VELOCITY_UNIT = new CompoundUnit(ENCODER_ANGLE, ENCODER_TIME);
        public static Unit ENCODER_ANGULAR_VELOCITY = new CompoundUnit(ENCODER_ANGLE, SECOND);

        public static Unit ANGULAR_VELOCITY = new CompoundUnit(RADIAN, SECOND);
        public static Unit METERS_PER_SECOND = new CompoundUnit(METER, SECOND);

        public static Unit METERS_PER_SECOND_2 = new CompoundUnit(METERS_PER_SECOND, SECOND);
        public static Unit NEWTON = new CompoundUnit(new Unit[] {KILOGRAM, METERS_PER_SECOND_2}, new Unit[] {});
        public static Unit JOULE = new CompoundUnit(new Unit[] {NEWTON, METER}, new Unit[] {});
        public static Unit COULOMB = new CompoundUnit(new Unit[] {AMPERE, SECOND}, new Unit[] {});
        public static Unit VOLTAGE = new CompoundUnit(JOULE, COULOMB);
        
    }

    public static class SubsystemConfig {

        //Drivetrain movement information
        public static double DRIVETRAIN_MAXIMUM_TESTED_ENCODER_VELOCITY = 3450;//Approx 4.03 meters per second
        public static double AUTO_MAX_VOLTAGE = 10.0;
        //Arm movement information 
        public static double kIz = 0.0;
        public static double kFF = 0.0;
        public static double kMaxOutput = 10000;
        public static double kMinOutput = -10000;
        public static double maxRPM = 0.0;
        public static double maxVel = 10000; //RPM
        public static double minVel = -10000;
        public static double maxAcc = 10000;
        public static double allowedErr = 0.0;
        public static double ARM_MAXIMUM_VELOCITY = 5;//in RPM; to be determined!!!
    }
    
    public static class MotionControl {
        //PID
        public static TalonSRXGains TEST_DRIVETRAIN_LEFT_PID = new TalonSRXGains(0.2, 0.0033, 12);
        public static TalonSRXGains TEST_DRIVETRAIN_RIGHT_PID = new TalonSRXGains(0.2, 0.0033, 12);
        // Actual PID values for drivetrain are below next to feedforward
        public static TalonSRXGains GRABBER_PID = new TalonSRXGains(0, 0, 0);
        // Tune this by hooking up phoenix tuner
        public static TalonSRXGains ROBOT_BALANCE_PID = new TalonSRXGains(0, 0, 0);
        public static TalonSRXGains ARM_PID = new TalonSRXGains(0.04, 0, 3); 
        
        //Feedforward
         
        // Feedforward/Feedback Gains for (test) drivetrain
        // Measured (2023) 0.73329
        // Theoretical value: ~0.5
        public static final double ksVolts = 1.1445;
        // Measured value: 2.4232
        // Theoretical value: 3
        public static final double kvVoltSecondsPerMeter = 20;
        // Ignore kA for most feedforward
        public static final double kaVoltSecondsSquaredPerMeter = 7.9791;
        
        // Theoretical values from ReCalc
        public static final double armkS = 3.0;
        public static final double armkV = 3.90;
        public static final double armkG = 0.36;

        // Drivetrain PID measured from SysId
        public static final double drivekP = 4.0847;
        public static final double drivekD = 0;
        
        // TODO: measure using drivetrain angular test
        // Measured (2023 drivetrain) 0.41
        public static final double kTrackwidthMeters = 0.41;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
        //Max Trajectory Velocity/Acceleration
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        //Ramsete Parameters (Reasonable Baselines)
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        // Feedforward objects for drivetrain
        // For now, set acceleration constant to 0 (its very hard to tune and not necessary for most drivetrain feedforward)
        public static SimpleMotorFeedforward DRIVETRAIN_FEEDFORWARD = new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter);
        public static TalonSRXGains DRIVETRAIN_LEFT_PID = new TalonSRXGains(0.28, 0.00001, 50);
        //public static TalonSRXGains DRIVETRAIN_LEFT_PID = new TalonSRXGains(0.15, 0.0033, 12);
        //public static TalonSRXGains DRIVETRAIN_RIGHT_PID = new TalonSRXGains(0.15, 0.0033, 12);
        public static TalonSRXGains DRIVETRAIN_RIGHT_PID = new TalonSRXGains(0.28, 0.00001, 50);

        public static TalonSRXGains DRIVETRAIN_RIGHT_VELOCITY_PID = new TalonSRXGains(0.30, 0.0033, 15);
        public static TalonSRXGains DRIVETRAIN_LEFT_VELOCITY_PID = new TalonSRXGains(0.30, 0.0033, 15);
        //Feedforward objects for arm
        public static ArmFeedforward ARM_FEEDFORWARD = new ArmFeedforward(armkS, armkG, armkV);

        public static double CLOSED_LOOP_RAMP_RATE = 0.6;
        public static double OPEN_LOOP_RAMP_RATE = 0.6;


    }
 
    //Driver settings
    public static double JOYSTICKF310_AXIS_DEADBAND = 0.05;
    public static double JOYSTICK_THROTTLE_EXPONENT = 1.5;
    public static double JOYSTICK_TURNING_EXPONENT = 3;
    ;

    //Operation config
    //@Config(name = "Rotation Input Multiplier", tabName = "Op Configuration")
    public static double ARCADE_ROTATION_MULTIPLIER = 0.25;

    //@Config(name = "Speed Input Multiplier", tabName = "Op Configuration")
    //Original value 0.45
    public static double ARCADE_SPEED_MULTIPLIER = 1;

    public static double SLOW_ARCADE_ROTATION_MULTIPLIER = 0.25;
    public static double SLOW_ARCADE_SPEED_MULTIPLIER = 0.45;
    public static double SLOW_JOYSTICK_THROTTLE_EXPONENT = 1;
    //Classes
    public static class TalonSRXGains extends SlotConfiguration {

        public TalonSRXGains(double kP, double kI, double kD) {
            super();
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }
    }
}
