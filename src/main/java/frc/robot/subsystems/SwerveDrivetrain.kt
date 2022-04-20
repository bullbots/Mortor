package frc.robot.subsystems

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.wpilibj.AnalogGyro
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants


class SwerveDrivetrain : SubsystemBase() {

    private val frontLeftLocation = Translation2d(0.381, 0.381)
    private val frontRightLocation = Translation2d(0.381, -0.381)
    private val backLeftLocation = Translation2d(-0.381, 0.381)
    private val backRightLocation = Translation2d(-0.381, -0.381)

    private val frontLeft = SwerveModule(1, 2, 0, 1, 2, 3)
    private val frontRight = SwerveModule(3, 4, 4, 5, 6, 7)
    private val backLeft = SwerveModule(5, 6, 8, 9, 10, 11)
    private val backRight = SwerveModule(7, 8, 12, 13, 14, 15)

    // TODO: Check if NavX2 is used here
    private val gyro = AnalogGyro(0)

    private val kinematics = SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation)

    private val odometry = SwerveDriveOdometry(kinematics, gyro.rotation2d)

    init {
        gyro.reset()
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed: Double / Speed of the robot in the x direction (forward)
     * @param ySpeed: Double / Speed of the robot in the y direction (sideways)
     * @param rotation: Double / Angular rate of the robot.
     * @param fieldRelative: Boolean / Whether the provided x and y speeds are relative to the field.
     */
    @SuppressWarnings("ParameterName")
    fun drive(xSpeed: Double, ySpeed: Double, rotation: Double, fieldRelative: Boolean) {
        val swerveModuleStates = kinematics.toSwerveModuleStates(
            if (fieldRelative) ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                rotation,
                gyro.rotation2d
            ) else ChassisSpeeds(xSpeed, ySpeed, rotation)
        )
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.MAX_SPEED)
        frontLeft.setDesiredState(swerveModuleStates[0])
        frontRight.setDesiredState(swerveModuleStates[1])
        backLeft.setDesiredState(swerveModuleStates[2])
        backRight.setDesiredState(swerveModuleStates[3])
    }

    fun updateOdometry() {
        odometry.update(
            gyro.rotation2d,
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        )
    }


}