package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.DifferentialDriveDebug;
import frc.robot.util.NavX;
import frc.robot.util.SafeTalonFX;

class DrivetrainFalcon : SubsystemBase() {

    // These values are used for Autonomous
    private val ticks_per_wheel_revolution = 42700.0
    private val ticks_per_foot = ticks_per_wheel_revolution / (0.5 * Math.PI) // .5 is diameter of wheel in feet

    // NEED THIS FOR AUTONOMOUS
    // private val max_ticks_per_hundred_milliseconds: Double = ticks_per_foot * Constants.MAX_SPEED_LOW_GEAR / 10

    // Initializing Master Falcon Motors
    val leftMasterFalcon = SafeTalonFX(Constants.LEFT_MASTER_PORT, false) // change to false for no PID?
    val rightMasterFalcon = SafeTalonFX(Constants.RIGHT_MASTER_PORT, false)

    // Initializing Slave Falcon Motors
    private val leftSlaveFalcon = SafeTalonFX(Constants.LEFT_SLAVE_PORT, false)
    private val rightSlaveFalcon = SafeTalonFX(Constants.RIGHT_SLAVE_PORT, false)

    private val diffDrive = DifferentialDriveDebug(leftMasterFalcon, rightMasterFalcon)
    private val gyro = NavX()

    private val leftCurrent: NetworkTableEntry? = null
    private val leftPosition: NetworkTableEntry? = null
    private val leftVelocity: NetworkTableEntry? = null

    private val rightCurrent: NetworkTableEntry? = null
    private val rightPosition: NetworkTableEntry? = null
    private val rightVelocity: NetworkTableEntry? = null

    private val shiftThreshold = 0.8
    private val firstGearSlope = 1 / shiftThreshold
    private val secondGearSlope = ( (21000 - 9240) / (1 - shiftThreshold)) / 21000

    private var m_leftDist: Double? = null
    private var m_rightDist: Double? = null

    private var m_flippedOdometry = false

    private val m_odometry = DifferentialDriveOdometry(gyro.rotation2d)

    // Any static variables or cases must go here
    companion object {
        val m_fieldSim = Field2d()
    }

    enum class CoastMode {
        Coast, Brake
    }

    init {
        if (RobotBase.isReal()) {

            leftSlaveFalcon.follow(leftMasterFalcon);
            rightSlaveFalcon.follow(rightMasterFalcon);

            rightMasterFalcon.inverted = true;
            rightSlaveFalcon.setInverted(InvertType.FollowMaster);
            leftMasterFalcon.inverted = false;
            leftSlaveFalcon.setInverted(InvertType.FollowMaster);

            setCoastMode(CoastMode.Coast);

            // leftMasterFalcon.configClosedloopRamp(Constants.DRIVETRAIN_RAMP);
            // rightMasterFalcon.configClosedloopRamp(Constants.DRIVETRAIN_RAMP);

            // orchestra = new Orchestra();
            // orchestra.addInstrument(leftMasterFalcon);
            // orchestra.addInstrument(rightMasterFalcon);
            // orchestra.addInstrument(leftSlaveFalcon);
            // orchestra.addInstrument(rightSlaveFalcon);

            // orchestra.loadMusic("test.chrp");
            diffDrive.setDeadband(0.05);
        }

        // diffDrive.setRightSideInverted(false);
        diffDrive.setSafetyEnabled(false);

        // shifter.shiftLow();

        // configurePID();
        // configureMotionMagic();
        // configureSmartDashboard();

        SmartDashboard.putData("Field", m_fieldSim);
    }

    fun setCoastMode(coastMode: CoastMode) {
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

    fun updateOdometry() {
        val leftDist = leftMasterFalcon.getSelectedSensorPosition() / ticks_per_foot
        val rightDist = rightMasterFalcon.getSelectedSensorPosition() / ticks_per_foot

        var rotation2d = gyro.rotation2d

            if (m_flippedOdometry) {
                rotation2d.rotateBy(Rotation2d.fromDegrees(180.0))
            }



        m_odometry.update(rotation2d, leftDist, rightDist)

    }

    fun resetOdometry(pose: Pose2d) {
        leftMasterFalcon.setSelectedSensorPosition(0)

    }




}