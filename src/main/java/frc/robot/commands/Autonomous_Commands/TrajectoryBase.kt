package frc.robot.commands.Autonomous_Commands

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.controller.RamseteController
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.subsystems.DrivetrainFalcon
import frc.robot.util.TrajectoryManager

class TrajectoryBase(private val drivetrain: DrivetrainFalcon, private val imu: AHRS,
                     private val trajectoryName: String, private val resetGyro: Boolean = true ) : CommandBase() {

    private val timer = Timer()

    private val ramsete = RamseteController()

    private var loopIdx = 0

    private var trajectory: Trajectory? = null

    private var isInitialized: Boolean = false

    init {
        addRequirements(drivetrain)
    }

    private fun getTrajectory() {
        if (trajectory == null && TrajectoryManager.getTrajectories() != null) {
            println("INFO: getting trajectory: $trajectoryName")
            trajectory = TrajectoryManager.getTrajectories()!![trajectoryName]
        }
    }

    private fun initializeTrajectory() {
        if (!isInitialized) {
            println("INFO: initializing trajectory")
            getTrajectory()
            trajectory?.initialPose?.let {
                drivetrain.resetOdometry(it)
                isInitialized = true
                timer.reset()
                timer.start()
            }
        }
    }

    override fun initialize() {
        if (resetGyro) {
            drivetrain.resetGyro()
        }

        ramsete.setEnabled(true)
//        drivetrain.setOdometryDirection(isBackwards)
    }

    override fun execute() {
        val elapsed = timer.get()

        initializeTrajectory()

        if (!isInitialized) { return }

        val reference = trajectory!!.sample(elapsed)

        val speeds = ramsete.calculate(drivetrain.getPose(), reference)

        val ramseteSpeed = speeds.vxMetersPerSecond
        val ramseteRot = speeds.omegaRadiansPerSecond

        val normalizedRamseteSpeed = ramseteSpeed / Constants.FTPERSEC_TOPSPEED
        val normalizedRamseteRot = -ramseteRot / Constants.FTPERSEC_TOPSPEED * Constants.WHEEL_RADIUS_FT

//        val direction = if (isBackwards) -1.0 else 1.0
        drivetrain.arcadeDrive(normalizedRamseteSpeed, normalizedRamseteRot, false)

        val tPose = reference!!.poseMeters
        val tX = tPose.x
        val tY = tPose.y
        val tRotation = tPose.rotation.degrees

        val aPose = drivetrain.getPose()
        val aX = aPose.x
        val aY = aPose.y
        val aRotation = aPose.rotation.degrees

//        loopIdx++
//        if (loopIdx == 10) {
//            loopIdx = 0
//
//            SmartDashboard.putNumber("Ramsete Speed - Normalized", normalizedRamseteSpeed)
//            SmartDashboard.putNumber("Ramsete Rot - Normalized", normalizedRamseteRot)
//
//            SmartDashboard.putNumber("Pose X - Trajectory", tX)
//            SmartDashboard.putNumber("Pose Y - Trajectory", tY)
//            SmartDashboard.putNumber("Pose R - Trajectory", tRotation)
//
//            SmartDashboard.putNumber("Pose X - Actual", aX)
//            SmartDashboard.putNumber("Pose Y - Actual", aY)
//            SmartDashboard.putNumber("Pose R - Actual", aRotation)
//        }


        DrivetrainFalcon.m_fieldSim.robotPose = reference.poseMeters
    }

    override fun end(interrupted: Boolean) {
        println("INFO: trajectory end: $trajectoryName")
        drivetrain.setOdometryDirection(false)
        isInitialized = false
    }

    override fun isFinished(): Boolean {
        if (imu.isConnected && !imu.isCalibrating) {
            trajectory?.let {
                print("INFO: checking isFinished: $trajectoryName")
                return timer.get() > it.totalTimeSeconds
            }
        } else {
            CommandScheduler.getInstance().cancelAll()
            return true
        }
        return true
    }

}