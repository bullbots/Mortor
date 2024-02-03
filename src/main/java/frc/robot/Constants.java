package frc.robot;

public class Constants {

    // Drivetrain Ports
    public static final int RIGHT_MASTER_PORT = 1;
    public static final int RIGHT_SLAVE_PORT = 2;
    public static final int LEFT_MASTER_PORT = 3;
    public static final int LEFT_SLAVE_PORT = 4;

    // Shooter Ports
    public static final int SHOOTER_PORT = 5;


    // Climber Ports
    public static final int CLIMBER_PORT = 7;

    // Unassigned Falcons
    public static final int unassigned1 = 6;

    // Intake Ports
    public static final int INTAKE_SPINNER_PORT = 8;
    public static final int RAISE_LOWER_ARM_PORT = 9;
    public static final int INTAKE_SPIN_PORT = 10;
    public static final int INTAKE_ARM_SPINNER_PORT = 11;


    // Unassigned NEOs


    /**
     * ALL DRIVETRAIN RELATED CONSTANTS GO HERE
     */
    // TODO: These values need to be tuned
    public static final double LEFT_VELOCITY_P = 0.05;
    public static final double LEFT_VELOCITY_I = 0.0;
    public static final double LEFT_VELOCITY_D = 0.0;
    public static final double LEFT_VELOCITY_FF = 2048.0 / 21000.0;

    public static final double LEFT_MASTER_ACCELERATION = 0.0;
    public static final double LEFT_MASTER_VELOCITY = 0.0;

    public static final double RIGHT_VELOCITY_P = 0.05;
    public static final double RIGHT_VELOCITY_I = 0.0;
    public static final double RIGHT_VELOCITY_D = 0.0;
    public static final double RIGHT_VELOCITY_FF = 2048.0 / 21000.0;


    public static final double RIGHT_MASTER_ACCELERATION = 3300.0;
    public static final double RIGHT_MASTER_VELOCITY = 3300.0;

    public static final double WHEEL_RADIUS_FT = 0.333;
    public static final double WHEEL_DIAMETER_FT = 0.666;
    public static final double TRACK_WIDTH_METERS = 0.67945; // 26.75 Inches
    public static final double TRACK_WIDTH_FT = 2.229; // 26.75 Inches
    public static final double FTPERSEC_TOPSPEED = 16.827;
    public static final double MAX_ANGULAR_VELOCITY = FTPERSEC_TOPSPEED * WHEEL_RADIUS_FT;
    /**
     * ALL INTAKE RELATED CONSTANTS GO HERE
     */
    public static final double INTAKE_ARM_HOLD_THRESHOLD = -1.0;
    public static final double INTAKE_ARM_DROP_THRESHOLD = 1.0;
    public static final double INTAKE_ARM_UP = 3.0;
    public static final double INTAKE_ARM_DOWN = -40.0;
    public static final double INTAKE_ARM_HOLD = -6.0;

    public static final double INTAKE_FF = 0.000156; // Normalize to 1023 (Max Speed)
    public static final double INTAKE_P = 5e-5; // 0
    public static final double INTAKE_I = 1e-6; // 1e-6
    public static final double INTAKE_D = 0.00; // 0
    public static final double INTAKE_IZONE = 0.0;
    public static final int I_SlotIdx = 0;
    public static final double I_ALLOWED_ERROR = 0.0;
    public static final double I_MAXRPM = 60000.0; // TODO: How does this work!!!

    /**
     *  ALL SHOOTER RELATED CONSTANTS GO HERE
     */

    // TODO: These values need to be tuned
    public static final double SHOOTER_KFF = 0.0473; // Normalize to 1023 (Max Speed)
    public static final double SHOOTER_KP = 0.05; // 0
    public static final double SHOOTER_KI = 0.0; // 1e-6
    public static final double SHOOTER_KD = 0.005; // 0

    public static final double NEO_MAX_RPM = 5676.0; // http://www.revrobotics.com/rev-21-1650/ "Empirical Free Speed"

    /**
     * ALL CLIMB RELATED CONSTANTS GO HERE
     */
    public static final double CLIMBER_GRENADE = -12000.0;
    public static final double CLIMBER_DOWN = -21000.0;
    public static final double CLIMBER_UP = 330000.0;
    public static final int CLIMBER_LIMIT_THRESHOLD = 1000;
    public static final int kPIDLoopIdx = 0;

    public static final double CLIMBER_KP = 0.05;
    public static final double CLIMBER_KI = 0.0;
    public static final double CLIMBER_KD = 0.005; // kP / 10
    public static final double CLIMBER_KFF = 2048.0 / 21000.0;
    public static final double CLIMBER_IZONE = 0.0;
    public static final double CLIMBER_PEAK_OUTPUT = 1.0;

    public static final int kSlotIdx = 0;

    /**
     * MISCELLANEOUS CONSTANTS
     */
    public static final int kTIMEOUT_MS = 30;
}
