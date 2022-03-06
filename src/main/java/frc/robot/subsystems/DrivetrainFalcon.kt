package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.InvertType
import com.ctre.phoenix.motorcontrol.NeutralMode
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.util.DifferentialDriveDebug
import frc.robot.util.NavX
import frc.robot.util.PIDControllerDebug
import frc.robot.util.SafeTalonFX
import kotlin.math.atan2
import kotlin.math.hypot

/**
 * Initializes the drivetrain using falcon500
 */
class DrivetrainFalcon : SubsystemBase() {

    // These values are used for Autonomous
    private val ticks_per_wheel_revolution = 26112.0
    private val ticks_per_foot = ticks_per_wheel_revolution / (0.666 * Math.PI) // .8 inches is diameter of wheel in feet

    // NEED THIS FOR AUTONOMOUS
    // private val max_ticks_per_hundred_milliseconds: Double = ticks_per_foot * Constants.MAX_SPEED_LOW_GEAR / 10

    // Initializing Master Falcon Motors
    private val leftMasterFalcon = SafeTalonFX(Constants.LEFT_MASTER_PORT, isDrivetrain=true) // change to false for no PID?
    private val rightMasterFalcon = SafeTalonFX(Constants.RIGHT_MASTER_PORT, isDrivetrain=true)

    // Initializing Slave Falcon Motors
    private val leftSlaveFalcon = SafeTalonFX(Constants.LEFT_SLAVE_PORT, isDrivetrain=true)
    private val rightSlaveFalcon = SafeTalonFX(Constants.RIGHT_SLAVE_PORT, isDrivetrain=true)

    private val leftGroup = MotorControllerGroup(leftMasterFalcon, leftSlaveFalcon)
    private val rightGroup = MotorControllerGroup(rightMasterFalcon, rightSlaveFalcon)

//    private val kinematics = DifferentialDriveKinematics(Constants.TRACK_WIDTH)
    private val diffDrive = DifferentialDriveDebug(leftMasterFalcon, rightMasterFalcon)
    private val imu = NavX()

    private val leftPIDController = PIDControllerDebug(0.02, 0.0, 0.0)
    private val rightPIDController = PIDControllerDebug(0.02, 0.0, 0.0)

    // TODO: ks and kv values need to be determined for the robot
    private val feedforward = SimpleMotorFeedforward(1.0, 3.0)

    lateinit var leftCurrent: NetworkTableEntry
    lateinit var leftPosition: NetworkTableEntry
    lateinit var leftVelocity: NetworkTableEntry

    lateinit var rightCurrent: NetworkTableEntry
    lateinit var rightPosition: NetworkTableEntry
    lateinit var rightVelocity: NetworkTableEntry

    var isFullSpeed = 1.0
//    private val firstGearSlope = 1 / shiftThreshold
//    private val secondGearSlope = ( (21000 - 9240) / (1 - shiftThreshold)) / 21000

    private var m_flippedOdometry = false

    private var loopIdx = 0

    private val m_odometry = DifferentialDriveOdometry(imu.rotation2d)

    // Any static variables or cases must go here
    companion object {
        val m_fieldSim = Field2d()
    }

    enum class CoastMode {
        Coast, Brake
    }

    init {
        if (RobotBase.isReal()) {

            leftSlaveFalcon.follow(leftMasterFalcon)
            rightSlaveFalcon.follow(rightMasterFalcon)

            rightMasterFalcon.inverted = true
            rightSlaveFalcon.setInverted(InvertType.FollowMaster)
            leftMasterFalcon.inverted = false
            leftSlaveFalcon.setInverted(InvertType.FollowMaster)

            setCoastMode(CoastMode.Brake)

            // leftMasterFalcon.configClosedloopRamp(Constants.DRIVETRAIN_RAMP);
            // rightMasterFalcon.configClosedloopRamp(Constants.DRIVETRAIN_RAMP);

            // orchestra = new Orchestra();
            // orchestra.addInstrument(leftMasterFalcon);
            // orchestra.addInstrument(rightMasterFalcon);
            // orchestra.addInstrument(leftSlaveFalcon);
            // orchestra.addInstrument(rightSlaveFalcon);

            // orchestra.loadMusic("test.chrp");
            diffDrive.setDeadband(0.05)
        }

        // diffDrive.setRightSideInverted(false);
        diffDrive.isSafetyEnabled = false

        // shifter.shiftLow();

        // configurePID();
        // configureMotionMagic();
        // configureSmartDashboard();

//        configureSmartDashBoard()

        resetEncoders()

        SmartDashboard.putData("Field", m_fieldSim)
    }


    private fun setCoastMode(coastMode: CoastMode) {
        // Sets neutralMode to Coast or Brake depending on coastMode
        var neutralMode = NeutralMode.Coast // By default this is Coast
        if(coastMode == CoastMode.Coast) {
            neutralMode = NeutralMode.Coast
        } else if (coastMode == CoastMode.Brake) {
            neutralMode = NeutralMode.Brake
        }
        rightMasterFalcon.setNeutralMode(neutralMode)
        rightSlaveFalcon.setNeutralMode(neutralMode)
        leftMasterFalcon.setNeutralMode(neutralMode)
        leftSlaveFalcon.setNeutralMode(neutralMode)
    }

    fun setOdometryDirection(invert: Boolean) { m_flippedOdometry = invert }

    private fun updateOdometry() {
        var leftDist = leftMasterFalcon.selectedSensorPosition / ticks_per_foot
        var rightDist = rightMasterFalcon.selectedSensorPosition / ticks_per_foot

//        println("INFO: Left Dist: $leftDist, Right Dist: $rightDist")

        if(m_flippedOdometry) {
            var temporary = -leftDist
            leftDist = -rightDist
            rightDist = temporary
        }

        var rotation2d = imu.rotation2d

        if (m_flippedOdometry) {
            rotation2d.rotateBy(Rotation2d.fromDegrees(180.0))
        }

        m_odometry.update(rotation2d, leftDist, rightDist)

    }

    fun resetOdometry(pose: Pose2d) {
        leftMasterFalcon.selectedSensorPosition = 0.0
        rightMasterFalcon.selectedSensorPosition = 0.0

        m_odometry.resetPosition(pose, imu.rotation2d)
    }

    fun resetGyro() { imu.reset() }

    fun resetGyro180() { imu.reset180() }

    fun getPose(): Pose2d { return m_odometry.poseMeters }

    fun configurePID() {

        // Set Velocity PID Constants in slot 0
        leftMasterFalcon.config_kF(0, Constants.LEFT_VELOCITY_FF)
        leftMasterFalcon.config_kP(0, Constants.LEFT_VELOCITY_P)
        leftMasterFalcon.config_kI(0, Constants.LEFT_VELOCITY_I)
        leftMasterFalcon.config_kD(0, Constants.LEFT_VELOCITY_D)

        rightMasterFalcon.config_kF(0, Constants.RIGHT_VELOCITY_FF)
        rightMasterFalcon.config_kP(0, Constants.RIGHT_VELOCITY_P)
        rightMasterFalcon.config_kI(0, Constants.RIGHT_VELOCITY_I)
        rightMasterFalcon.config_kD(0, Constants.RIGHT_VELOCITY_D)

    }

    fun configureMotionMagic() {
        leftMasterFalcon.configMotionCruiseVelocity(Constants.LEFT_MASTER_VELOCITY, Constants.kTIMEOUT_MS)
        leftMasterFalcon.configMotionAcceleration(Constants.LEFT_MASTER_ACCELERATION, Constants.kTIMEOUT_MS)

        rightMasterFalcon.configMotionCruiseVelocity(Constants.RIGHT_MASTER_VELOCITY, Constants.kTIMEOUT_MS)
        rightMasterFalcon.configMotionAcceleration(Constants.RIGHT_MASTER_ACCELERATION, Constants.kTIMEOUT_MS)
    }

    fun configureSmartDashBoard() {
        leftCurrent = generateEntry("Left Current", 0, 0)
        leftPosition = generateEntry("Left Position", 2, 0)
        leftVelocity = generateEntry("Left Velocity", 4, 0)
        rightCurrent = generateEntry("Right Current", 0, 2)
        rightPosition = generateEntry("Right Position", 2, 2)
        rightVelocity = generateEntry("Right Velocity", 4, 2)
    }

    override fun periodic() {
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

        updateOdometry()
        m_fieldSim

        loopIdx++
        if (loopIdx == 10) {
            loopIdx = 0

            if (RobotBase.isReal()) {
//                SmartDashboard.putNumber("Left Encoder", leftMasterFalcon.selectedSensorPosition)
//                SmartDashboard.putNumber("Right Encoder", rightMasterFalcon.selectedSensorPosition)
                SmartDashboard.putNumber("Heading", calcHeading())

//                leftCurrent.setNumber(leftMasterFalcon.statorCurrent)
                // leftPosition!!.setNumber(leftMasterFalcon.selectedSensorPosition)
                // leftVelocity!!.setNumber(leftMasterFalcon.selectedSensorVelocity)

//                rightCurrent.setNumber(rightMasterFalcon.statorCurrent)
                // rightPosition!!.setNumber(rightMasterFalcon.selectedSensorPosition)
                // rightVelocity!!.setNumber(rightMasterFalcon.selectedSensorVelocity)


            } else {
                var curLeftCurrent = 0.0

                // if (simIter.hasNext()) {
                //   curLeftCurrent = simIter.next();
                // }
                leftCurrent.setNumber(curLeftCurrent)
                leftPosition.setNumber(0.0)
                leftVelocity.setNumber(0.0)

                rightCurrent.setNumber(0.0)
                rightPosition.setNumber(0.0)
                rightVelocity.setNumber(0.0)

            }
        }
    }

    fun arcadeDrive(speed: Double, rotation: Double, squareInputs: Boolean) {

        diffDrive.arcadeDrive(speed, rotation, squareInputs)
    }


    fun curvatureDrive(speed: Double, rotation: Double, isQuickTurn: Boolean) {
        diffDrive.curvatureDrive(speed, rotation, isQuickTurn)
    }

    /**
     * Sets the encoder values back to zero
     */
    fun resetEncoders() {
        println("Reset Encoders called")
        leftMasterFalcon.selectedSensorPosition = 0.0
        rightMasterFalcon.selectedSensorPosition = 0.0
    }

    fun calcDist(): Double {
        val x = m_odometry.poseMeters.x
        val y = m_odometry.poseMeters.y

        val dist = hypot(x, y)


        return dist
    }
    /**
     * @return atanDegree: The heading the robot needs to face towards the goal
     */
    fun calcHeading(): Double {
        val x = m_odometry.poseMeters.x
        val y = m_odometry.poseMeters.y
        val delta = ((m_odometry.poseMeters.rotation.degrees % 360) + 360) % 360 - 180 // delta % 360 is to set the input between -360 and 360
        val atanDegree = Math.toDegrees(atan2(y, x))

//        println("INFO: X Position: $x, Y Position: $y, Delta: $delta, Heading: $atanDegree")

        return atanDegree
    }

    fun setSpeeds(speeds: DifferentialDriveWheelSpeeds) {
        val leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond)
        val rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond)
        val leftOutput =
            leftPIDController.calculate(leftMasterFalcon.selectedSensorPosition, speeds.leftMetersPerSecond)
        val rightOutput =
            rightPIDController.calculate(rightMasterFalcon.selectedSensorPosition, speeds.rightMetersPerSecond)

        leftGroup.setVoltage(leftOutput + leftFeedforward)
        rightGroup.setVoltage(rightOutput + rightFeedforward)

    }

    /**
     * Controls the robot using arcade drive.
     *
     * @param xSpeed: Double / The speed for the X axis
     * @param rotation: Double / The rotation
     */
    fun drive(xSpeed: Double, rotation: Double) {
//        setSpeeds(kinematics.toWheelSpeeds(ChassisSpeeds(xSpeed, 0.0, rotation)))
        diffDrive.arcadeDrive(xSpeed, rotation)
    }

//    override fun simulationPeriodic() {
//        drivetrainSim
//    }

    /**
     * @return double array of positions [left, right]
     */
    fun getPositions(): DoubleArray {
        return doubleArrayOf(leftMasterFalcon.selectedSensorPosition, rightMasterFalcon.selectedSensorPosition)
    }

    /**
     * @return double array of velocities [left, right]
     */
    fun getVelocities(): DoubleArray {
        return doubleArrayOf(leftMasterFalcon.selectedSensorVelocity, rightMasterFalcon.selectedSensorVelocity)
    }

    /**
     * Sets the left and right motors to a percent output
     * @param leftPercent Double
     * @param rightPercent Double
     */
    fun set(leftPercent: Double, rightPercent: Double) {
        leftMasterFalcon.set(leftPercent)
        rightMasterFalcon.set(rightPercent)
    }

    fun set(controlMode: ControlMode, leftMagnitude: Double, rightMagnitude: Double) {
        leftMasterFalcon.set(controlMode, leftMagnitude)
        rightMasterFalcon.set(controlMode, rightMagnitude)
    }

    /**
     * Immediately stops the drivetrain, only use in emergencies
     */
    fun stop() {
        leftMasterFalcon.stopMotor()
        rightMasterFalcon.stopMotor()
    }


    /**
     * Helper function to generate NetworkTableEntries
     */
    private fun generateEntry(entryName: String, columnIndex: Int, rowIndex: Int): NetworkTableEntry {
        return Shuffleboard.getTab("Drivetrain")
            .add(entryName, 0)
            .withSize(2, 2)
            .withPosition(columnIndex, rowIndex)
            .withWidget(BuiltInWidgets.kGraph)
            .entry
    }

    fun driveLeft(value: Double) { leftMasterFalcon.set(value) }

    fun driveRight(value: Double) { rightMasterFalcon.set(value) }
}