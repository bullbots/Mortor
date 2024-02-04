package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.DifferentialDriveDebug;
import frc.robot.util.NavX;
import frc.robot.util.PIDControllerDebug;
import frc.robot.util.SafeTalonFX;

/**
 * Initializes the drivetrain using falcon500
 */
class DrivetrainFalcon extends SubsystemBase {

    // These values are used for Autonomous
    private double ticks_per_wheel_revolution = 26112.0;
    private double ticks_per_foot = ticks_per_wheel_revolution / (Constants.WHEEL_DIAMETER_FT * Math.PI); // .8 inches is diameter of wheel in feet

    // NEED THIS FOR AUTONOMOUS
    // private val max_ticks_per_hundred_milliseconds: Double = ticks_per_foot * Constants.MAX_SPEED_LOW_GEAR / 10

    // Initializing Master Falcon Motors
    private SafeTalonFX leftMasterFalcon = new SafeTalonFX(Constants.LEFT_MASTER_PORT, true, false); // change to false for no PID?
    private SafeTalonFX rightMasterFalcon = new SafeTalonFX(Constants.RIGHT_MASTER_PORT, true, false);

    // Initializing Slave Falcon Motors
    private SafeTalonFX leftSlaveFalcon = new SafeTalonFX(Constants.LEFT_SLAVE_PORT, true, false);
    private SafeTalonFX rightSlaveFalcon = new SafeTalonFX(Constants.RIGHT_SLAVE_PORT, true, false);

//    private val leftGroup = MotorControllerGroup(leftMasterFalcon, leftSlaveFalcon)
//    private val rightGroup = MotorControllerGroup(rightMasterFalcon, rightSlaveFalcon)

//    private val kinematics = DifferentialDriveKinematics(Constants.TRACK_WIDTH)
    private DifferentialDriveDebug diffDrive = new DifferentialDriveDebug(leftMasterFalcon, rightMasterFalcon);
    private NavX imu = new NavX();

    private PIDControllerDebug leftPIDController = new PIDControllerDebug(0.02, 0.0, 0.0);
    private PIDControllerDebug rightPIDController = new PIDControllerDebug(0.02, 0.0, 0.0);

    // TODO: ks and kv values need to be determined for the robot
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1.0, 3.0);

    // WARN: In kotlin these were lateinit, don't know if that was important
    private NetworkTableEntry leftCurrent; 
    private NetworkTableEntry leftPosition;
    private NetworkTableEntry leftVelocity; 

    private NetworkTableEntry rightCurrent;
    private NetworkTableEntry rightPosition; 
    private NetworkTableEntry rightVelocity;

    private double isFullSpeed = 1.0;

    private boolean m_flippedOdometry = false;

    private int loopIdx = 0;

    private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(imu.getRotation2d(), 0.0, 0.0);

    public static Field2d m_fieldSim = new Field2d();

    public enum CoastMode {
        Coast, Brake
    }

    DrivetrainFalcon() {
        if (RobotBase.isReal()) {

            leftSlaveFalcon.follow(leftMasterFalcon);
            rightSlaveFalcon.follow(rightMasterFalcon);

            rightMasterFalcon.setInverted(true);
            rightSlaveFalcon.setInverted(InvertType.FollowMaster);
            leftMasterFalcon.setInverted(false);
            leftSlaveFalcon.setInverted(InvertType.FollowMaster);

            setCoastMode(CoastMode.Brake);

            // leftMasterFalcon.configClosedloopRamp(Constants.DRIVETRAIN_RAMP);
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
        // configureMotionMagic();

//        configureSmartDashBoard()

        resetEncoders();

//        SmartDashboard.putData("Field", m_fieldSim)
    }


    private void setCoastMode(CoastMode coastMode) {
        // Sets neutralMode to Coast or Brake depending on coastMode
        var neutralMode = NeutralMode.Coast; // By default this is Coast
        if(coastMode == CoastMode.Coast) {
            neutralMode = NeutralMode.Coast;
        } else if (coastMode == CoastMode.Brake) {
            neutralMode = NeutralMode.Brake;
        }
        rightMasterFalcon.setNeutralMode(neutralMode);
        rightSlaveFalcon.setNeutralMode(neutralMode);
        leftMasterFalcon.setNeutralMode(neutralMode);
        leftSlaveFalcon.setNeutralMode(neutralMode);
    }

    public void setOdometryDirection(boolean invert) { m_flippedOdometry = invert; }

    public double getAverageDist() {
        double leftDist = leftMasterFalcon.getSelectedSensorPosition() / ticks_per_foot;
        double rightDist = rightMasterFalcon.getSelectedSensorPosition() / ticks_per_foot;
        return (leftDist + rightDist) * 0.5;
    }

    private void updateOdometry() {
        double leftDist = leftMasterFalcon.getSelectedSensorPosition() / ticks_per_foot;
        double rightDist = rightMasterFalcon.getSelectedSensorPosition() / ticks_per_foot;

//        println("INFO: Left Dist: $leftDist, Right Dist: $rightDist")

        if(m_flippedOdometry) {
            double temporary = -leftDist;
            leftDist = -rightDist;
            rightDist = temporary;
        }

        Rotation2d rotation2d = imu.getRotation2d();

        if (m_flippedOdometry) {
            rotation2d.rotateBy(Rotation2d.fromDegrees(180.0));
        }

        m_odometry.update(rotation2d, leftDist, rightDist);

    }

    public void resetOdometry(Pose2d pose) {
        leftMasterFalcon.setSelectedSensorPosition(0.0);
        rightMasterFalcon.setSelectedSensorPosition(0.0);

        m_odometry.resetPosition(imu.getRotation2d(), 0.0, 0.0, pose);
    }

    public void resetGyro() { imu.reset(); }

    public Pose2d  getPose() { return m_odometry.getPoseMeters(); }

    public void configurePID() {

        // Set Velocity PID Constants in slot 0
        leftMasterFalcon.config_kF(0, Constants.LEFT_VELOCITY_FF);
        leftMasterFalcon.config_kP(0, Constants.LEFT_VELOCITY_P);
        leftMasterFalcon.config_kI(0, Constants.LEFT_VELOCITY_I);
        leftMasterFalcon.config_kD(0, Constants.LEFT_VELOCITY_D);

        rightMasterFalcon.config_kF(0, Constants.RIGHT_VELOCITY_FF);
        rightMasterFalcon.config_kP(0, Constants.RIGHT_VELOCITY_P);
        rightMasterFalcon.config_kI(0, Constants.RIGHT_VELOCITY_I);
        rightMasterFalcon.config_kD(0, Constants.RIGHT_VELOCITY_D);

    }

    public void configureMotionMagic() {
        leftMasterFalcon.configMotionCruiseVelocity(Constants.LEFT_MASTER_VELOCITY, Constants.kTIMEOUT_MS);
        leftMasterFalcon.configMotionAcceleration(Constants.LEFT_MASTER_ACCELERATION, Constants.kTIMEOUT_MS);

        rightMasterFalcon.configMotionCruiseVelocity(Constants.RIGHT_MASTER_VELOCITY, Constants.kTIMEOUT_MS);
        rightMasterFalcon.configMotionAcceleration(Constants.RIGHT_MASTER_ACCELERATION, Constants.kTIMEOUT_MS);
    }

//    fun configureSmartDashBoard() {
//        leftCurrent = generateEntry("Left Current", 0, 0)
//        leftPosition = generateEntry("Left Position", 2, 0)
//        leftVelocity = generateEntry("Left Velocity", 4, 0)
//        rightCurrent = generateEntry("Right Current", 0, 2)
//        rightPosition = generateEntry("Right Position", 2, 2)
//        rightVelocity = generateEntry("Right Velocity", 4, 2)
//    }

    @Override 
    public void periodic() {
//        println("DrivetrainFalcon periodic")
//        SmartDashboard.putNumber("Encoder Ticks - Left", leftMasterFalcon.selectedSensorPosition)
//        SmartDashboard.putNumber("Encoder Ticks - Right", rightMasterFalcon.selectedSensorPosition)
//        SmartDashboard.putNumber(
//            "Encoder Rate (Normalized) - Left",
//            leftMasterFalcon.selectedSensorVelocity / max_ticks_per_hundred_milliseconds
//        )
//        SmartDashboard.putNumber(
//            "Encoder Rate (Normalized) - Right",
//            rightMasterFalcon.selectedSensorVelocity / max_ticks_per_hundred_milliseconds
//        )
//
//        SmartDashboard.putNumber("NavX Angle", gyro.rotation2d.degrees)
//
//        SmartDashboard.putNumber("Right Master Current", rightMasterFalcon.statorCurrent)
//        SmartDashboard.putNumber("Right Slave Current", rightSlaveFalcon.statorCurrent)
//        SmartDashboard.putNumber("Left Master Current", leftMasterFalcon.statorCurrent)
//        SmartDashboard.putNumber("Left Slave Current", leftSlaveFalcon.statorCurrent)

        updateOdometry();



        loopIdx++;
        if (loopIdx == 10) {
            loopIdx = 0;

            if (RobotBase.isReal()) {
//                SmartDashboard.putNumber("Left Encoder", leftMasterFalcon.selectedSensorPosition)
//                SmartDashboard.putNumber("Right Encoder", rightMasterFalcon.selectedSensorPosition)
//                SmartDashboard.putNumber("Left Drive Speed", leftMasterFalcon.selectedSensorVelocity / 22000)
//                SmartDashboard.putNumber("Right Drive Speed", rightMasterFalcon.selectedSensorVelocity / 22000)
//                SmartDashboard.putNumber("Left Drive Stator Current", leftMasterFalcon.statorCurrent)
//                SmartDashboard.putNumber("Right Drive Stator Current", rightMasterFalcon.statorCurrent)
//                SmartDashboard.putNumber("Left Drive Supply Current", leftMasterFalcon.supplyCurrent)
//                SmartDashboard.putNumber("Right Drive Supply Current", rightMasterFalcon.supplyCurrent)

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

    public void arcadeDrive(double speed, double rotation, boolean squareInputs) {

        diffDrive.arcadeDrive(speed, rotation, squareInputs);
    }


    public void curvatureDrive(double speed, double rotation, boolean isQuickTurn) {
        diffDrive.curvatureDrive(speed, rotation, isQuickTurn);
//        diffDrive.arcadeDrive(speed, rotation, squareInputs=false)
    }

    /**
     * Sets the encoder values back to zero
     */
    public void resetEncoders() {
        System.out.println("Reset Encoders called");
        leftMasterFalcon.setSelectedSensorPosition(0.0);
        rightMasterFalcon.setSelectedSensorPosition(0.0);
    }

    public double calcDist() {
        double x = m_odometry.getPoseMeters().getX();
        double y = m_odometry.getPoseMeters().getY();

        double dist = Math.hypot(x, y);

        return dist;
    }
    /**
     * @return atanDegree: The heading the robot needs to face towards the goal
     */
    public double calcHeading() {
        double x = m_odometry.getPoseMeters().getX();
        double y = m_odometry.getPoseMeters().getY();
//        val delta = ((m_odometry.poseMeters.rotation.degrees % 360) + 360) % 360 - 180 // delta % 360 is to set the input between -360 and 360
        double atanDegree = Math.toDegrees(Math.atan2(y, x));

//        println("INFO: X Position: $x, Y Position: $y, Delta: $delta, Heading: $atanDegree")

        return atanDegree;
    }

    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
        double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);
        double leftOutput =
            leftPIDController.calculate(leftMasterFalcon.getSelectedSensorPosition(), speeds.leftMetersPerSecond);
        double rightOutput =
            rightPIDController.calculate(rightMasterFalcon.getSelectedSensorPosition(), speeds.rightMetersPerSecond);

//        leftGroup.setVoltage(leftOutput + leftFeedforward)
//        rightGroup.setVoltage(rightOutput + rightFeedforward)

    }

    /**
     * Controls the robot using arcade drive.
     *
     * @param xSpeed: Double / The speed for the X axis
     * @param rotation: Double / The rotation
     */
    public void drive(double xSpeed, double rotation) {
//        setSpeeds(kinematics.toWheelSpeeds(ChassisSpeeds(xSpeed, 0.0, rotation)))
        diffDrive.arcadeDrive(xSpeed, rotation);
    }

//    override fun simulationPeriodic() {
//        drivetrainSim
//    }

    /**
     * @return double array of positions [left, right]
     */
    public double[] getPositions() {
	double pos[] = new double[2];
        pos[0] = leftMasterFalcon.getSelectedSensorPosition();
	pos[1] = rightMasterFalcon.getSelectedSensorPosition();
        return pos;
    }

    /**
     * @return double array of velocities [left, right]
     */
    public double[] getVelocities() {
	double velocites[] = new double[2];
        velocites[0] = leftMasterFalcon.getSelectedSensorVelocity();
	velocites[1] = rightMasterFalcon.getSelectedSensorVelocity();
        return velocites;
    }

    /**
     * Sets the left and right motors to a percent output
     * @param leftPercent Double
     * @param rightPercent Double
     */
    public void set(double leftPercent, double rightPercent) {
        leftMasterFalcon.set(leftPercent);
        rightMasterFalcon.set(rightPercent);
    }

    public void set(ControlMode controlMode, double leftMagnitude, double rightMagnitude) {
        leftMasterFalcon.set(controlMode, leftMagnitude);
        rightMasterFalcon.set(controlMode, rightMagnitude);
    }

    /**
     * Immediately stops the drivetrain, only use in emergencies
     */
    public void stop() {
        leftMasterFalcon.stopMotor();
        rightMasterFalcon.stopMotor();
    }


    /**
     * Helper function to generate NetworkTableEntries
     */
//    private fun generateEntry(entryName: String, columnIndex: Int, rowIndex: Int): NetworkTableEntry {
//        return Shuffleboard.getTab("Drivetrain")
//            .add(entryName, 0)
//            .withSize(2, 2)
//            .withPosition(columnIndex, rowIndex)
//            .withWidget(BuiltInWidgets.kGraph)
//            .entry
//    }

    fun driveLeft(double value) { leftMasterFalcon.set(value); }

    fun driveRight(double value) { rightMasterFalcon.set(value); }
}
