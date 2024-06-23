package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.DifferentialDriveDebug;
import frc.robot.util.NavX;
import frc.robot.util.PIDControllerDebug;
import frc.robot.util.SafeTalonFX;

/**
 * Initializes the drivetrain using falcon500
 */
public class DrivetrainFalcon extends SubsystemBase {

    public static Field2d m_fieldSim = new Field2d();
    // These values are used for Autonomous
    private final double ticks_per_wheel_revolution = 26112.0;

    // NEED THIS FOR AUTONOMOUS
    // private val max_ticks_per_hundred_milliseconds: Double = ticks_per_foot * Constants.MAX_SPEED_LOW_GEAR / 10
    private final double ticks_per_foot = ticks_per_wheel_revolution / (Constants.WHEEL_DIAMETER_FT * Math.PI); // .8 inches is diameter of wheel in feet
    // Initializing Master Falcon Motors
    private final SafeTalonFX leftMasterFalcon = new SafeTalonFX(Constants.LEFT_MASTER_PORT, true, false); // change to false for no PID?
    private final SafeTalonFX rightMasterFalcon = new SafeTalonFX(Constants.RIGHT_MASTER_PORT, true, false);
    // Initializing Slave Falcon Motors
    private final SafeTalonFX leftSlaveFalcon = new SafeTalonFX(Constants.LEFT_SLAVE_PORT, true, false);

    //    private val leftGroup = MotorControllerGroup(leftMasterFalcon, leftSlaveFalcon)
//    private val rightGroup = MotorControllerGroup(rightMasterFalcon, rightSlaveFalcon)
    private final SafeTalonFX rightSlaveFalcon = new SafeTalonFX(Constants.RIGHT_SLAVE_PORT, true, false);
    //    private val kinematics = DifferentialDriveKinematics(Constants.TRACK_WIDTH)
    private final DifferentialDriveDebug diffDrive = new DifferentialDriveDebug(leftMasterFalcon, rightMasterFalcon);
    private final NavX imu = new NavX();
    private final PIDControllerDebug leftPIDController = new PIDControllerDebug(0.02, 0.0, 0.0);
    private final PIDControllerDebug rightPIDController = new PIDControllerDebug(0.02, 0.0, 0.0);
    // TODO: ks and kv values need to be determined for the robot
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1.0, 3.0);
    private final double isFullSpeed = 1.0;
    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(imu.getRotation2d(), 0.0, 0.0);
    private int loopIdx = 0;

    public DrivetrainFalcon() {

        resetAll();

        configurePathPlanner();

        if (RobotBase.isReal()) {

            rightMasterFalcon.setInverted(true);
            rightSlaveFalcon.setInverted(true);
            leftMasterFalcon.setInverted(false);
            leftSlaveFalcon.setInverted(false);

            leftSlaveFalcon.setControl(new Follower(leftMasterFalcon.getDeviceID(), false));
            rightSlaveFalcon.setControl(new Follower(rightMasterFalcon.getDeviceID(), false));

            setNeutralMode(NeutralModeValue.Brake);

//             leftMasterFalcon.configClosedloopRamp(Constants.DRIVETRAIN_RAMP);
            // rightMasterFalcon.configClosedloopRamp(Constants.DRIVETRAIN_RAMP);

            // orchestra = new Orchestra();
            // orchestra.addInstrument(leftMasterFalcon);
            // orchestra.addInstrument(rightMasterFalcon);
            // orchestra.addInstrument(leftSlaveFalcon);
            // orchestra.addInstrument(rightSlaveFalcon);

            // orchestra.loadMusic("test.chrp");
//            diffDrive.setDeadband(0.02)
        }

        // diffDrive.setRightSideInverted(false);
        //diffDrive.isSafetyEnabled = false; //WARN:this line of kotlin commented out because I don't believe it does anything useful

        // shifter.shiftLow();

        configurePID();

//        resetEncoders();



        SmartDashboard.putData("Field", m_fieldSim);
    }

    private void resetAll() {
        var pose = new Pose2d();
        resetPose(pose);
    }

    private void configurePathPlanner() {
        AutoBuilder.configureRamsete(
                m_odometry::getPoseMeters,
                this::resetPose,
                this::getCurrentChassisSpeeds,
                this::curvatureDriveChassisSpeeds,
                // b?,
                // zeta?,
                new ReplanningConfig(), // Default path replanning config. See the API for the options here
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public double getIsFullSpeed() {
        return isFullSpeed;
    }

    private void setNeutralMode(NeutralModeValue coastMode) {
        rightMasterFalcon.setNeutralMode(coastMode);
        leftMasterFalcon.setNeutralMode(coastMode);
    }

    public double getAverageDist() {
        double leftDist = leftMasterFalcon.getPosition().getValue() / ticks_per_foot;
        double rightDist = rightMasterFalcon.getPosition().getValue() / ticks_per_foot;
        return (leftDist + rightDist) * 0.5;
    }

    public void resetPose(Pose2d pose) {
        System.out.println("INFO: DrivetrainFalcon.resetOdometry");
        resetEncoders();
        // Ideally these should both be zero now.
        var leftDist = leftMasterFalcon.getPosition().getValue();
        var rightDist = leftMasterFalcon.getPosition().getValue();

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            // We are using Blue-alliance always field orientation.
            if (alliance.get() == DriverStation.Alliance.Red) {
                imu.setAngleOffset(180.0);
            } else {
                imu.setAngleOffset(0.0);
            }
        }

        m_odometry.resetPosition(imu.getRotation2d(), leftDist, rightDist, pose);
    }

    public void configurePID() {
        // Set Velocity PID Constants in slot 0
//        leftMasterFalcon.config_kF(0, Constants.LEFT_VELOCITY_FF);
//        leftMasterFalcon.config_kP(0, Constants.LEFT_VELOCITY_P);
//        leftMasterFalcon.config_kI(0, Constants.LEFT_VELOCITY_I);
//        leftMasterFalcon.config_kD(0, Constants.LEFT_VELOCITY_D);
//
//        rightMasterFalcon.config_kF(0, Constants.RIGHT_VELOCITY_FF);
//        rightMasterFalcon.config_kP(0, Constants.RIGHT_VELOCITY_P);
//        rightMasterFalcon.config_kI(0, Constants.RIGHT_VELOCITY_I);
//        rightMasterFalcon.config_kD(0, Constants.RIGHT_VELOCITY_D);
    }

    public void configureMotionMagic() {
//        leftMasterFalcon.configMotionCruiseVelocity(Constants.LEFT_MASTER_VELOCITY, Constants.kTIMEOUT_MS);
//        leftMasterFalcon.configMotionAcceleration(Constants.LEFT_MASTER_ACCELERATION, Constants.kTIMEOUT_MS);
//
//        rightMasterFalcon.configMotionCruiseVelocity(Constants.RIGHT_MASTER_VELOCITY, Constants.kTIMEOUT_MS);
//        rightMasterFalcon.configMotionAcceleration(Constants.RIGHT_MASTER_ACCELERATION, Constants.kTIMEOUT_MS);
    }

    private void debugPeriodic() {
        loopIdx++;
        if (loopIdx == 10) {
            loopIdx = 0;

            if (RobotBase.isReal()) {
//                SmartDashboard.putNumber("L

//                SmartDashboard.putNumber("Heading", calcHeading())

//                leftCurrent.setNumber(leftMasterFalcon.statorCurrent)
                // leftPosition!!.setNumber(leftMasterFalcon.selectedSensorPosition)
//                 leftVelocity!!.setNumber(leftMasterFalcon.selectedSensorVelocity)

//                rightCurrent.setNumber(rightMasterFalcon.statorCurrent)
                // rightPosition!!.setNumber(rightMasterFalcon.selectedSensorPosition)
//                 rightVelocity!!.setNumber(rightMasterFalcon.selectedSensorVelocity)


            } else {
                double curLeftCurrent = 0.0;

                // if (simIter.hasNext()) {
                //   curLeftCurrent = simIter.next();
                // }
//                leftCurrent.setNumber(curLeftCurrent)
//                leftPosition.setNumber(0.0)
//                leftVelocity.setNumber(0.0)
//
//                rightCurrent.setNumber(0.0)
//                rightPosition.setNumber(0.0)
//                rightVelocity.setNumber(0.0)

            }
        }
    }

    private void curvatureDriveChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        double speed = chassisSpeeds.vxMetersPerSecond / Constants.MAX_METERS_PER_SECOND_VELOCITY;
        double rotation = chassisSpeeds.omegaRadiansPerSecond / Constants.MAX_ANGULAR_VELOCITY;
        curvatureDrive(speed, rotation);
    }

    private void curvatureDrive(double speed, double rotation) {
        curvatureDrive(speed, rotation, false);
    }

    public void curvatureDrive(double speed, double rotation, boolean isQuickTurn) {
        diffDrive.curvatureDrive(speed, rotation, isQuickTurn);
    }

    public void resetEncoders() {
        System.out.println("INFO: DrivetrainFalcon.resetEncoders");
        leftMasterFalcon.setPosition(0.0);
        rightMasterFalcon.setPosition(0.0);
    }

//    public double calcDist() {
//        double x = m_odometry.getPoseMeters().getX();
//        double y = m_odometry.getPoseMeters().getY();
//        double dist = Math.hypot(x, y);
//        return dist;
//    }

//    public double calcHeading() {
//        double x = m_odometry.getPoseMeters().getX();
//        double y = m_odometry.getPoseMeters().getY();
////        val delta = ((m_odometry.poseMeters.rotation.degrees % 360) + 360) % 360 - 180 // delta % 360 is to set the input between -360 and 360
//        double atanDegree = Math.toDegrees(Math.atan2(y, x));
//        return atanDegree;
//    }

//    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
//        double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
//        double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);
//        double leftOutput =
//                leftPIDController.calculate(leftMasterFalcon.getPosition().getValue(), speeds.leftMetersPerSecond);
//        double rightOutput =
//                rightPIDController.calculate(rightMasterFalcon.getPosition().getValue(), speeds.rightMetersPerSecond);
////        leftGroup.setVoltage(leftOutput + leftFeedforward)
////        rightGroup.setVoltage(rightOutput + rightFeedforward)
//    }

//    public void drive(double xSpeed, double rotation) {
////        setSpeeds(kinematics.toWheelSpeeds(ChassisSpeeds(xSpeed, 0.0, rotation)))
//        diffDrive.arcadeDrive(xSpeed, rotation);
//    }

//    public double[] getPositions() {
//        double[] pos = new double[2];
//        pos[0] = leftMasterFalcon.getPosition().getValue();
//        pos[1] = rightMasterFalcon.getPosition().getValue();
//        return pos;
//    }

    public double[] getVelocities() {
        double[] velocites = new double[2];
        velocites[0] = leftMasterFalcon.getVelocity().getValue();
        velocites[1] = rightMasterFalcon.getVelocity().getValue();
        return velocites;
    }

    public DifferentialDriveWheelSpeeds getDifferentialDriveWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
    }

    public ChassisSpeeds getCurrentChassisSpeeds() {
        var diffWheelSpeeds = getDifferentialDriveWheelSpeeds();
        return Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(difWheelSpeeds);
    }

//    public void set(double leftPercent, double rightPercent) {
//        leftMasterFalcon.set(leftPercent);
//        rightMasterFalcon.set(rightPercent);
//    }

//    public void set(ControlMode controlMode, double leftMagnitude, double rightMagnitude) {
//        leftMasterFalcon.set(leftMagnitude);
//        rightMasterFalcon.set(rightMagnitude);
//    }

    public void stop() {
        leftMasterFalcon.stopMotor();
        rightMasterFalcon.stopMotor();
    }

    private void updateOdometry() {
        Rotation2d rotation2d = imu.getRotation2d();
        double leftDist = leftMasterFalcon.getPosition().getValue();
        double rightDist = rightMasterFalcon.getPosition().getValue();

        m_odometry.update(rotation2d, leftDist, rightDist);
    }

    @Override
    public void periodic() {
        updateOdometry();
        m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
        debugPeriodic();
    }
}
