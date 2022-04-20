package frc.robot.subsystems

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Encoder
import frc.robot.Constants
import frc.robot.util.SafeSparkMax
import frc.robot.util.SafeTalonFX
import kotlin.math.PI


class SwerveModule(driveFalconID: Int,
                   turningMotorID: Int,
                   driveEncoderChannelA: Int,
                   driveEncoderChannelB: Int,
                   turningEncoderChannelA: Int,
                   turningEncoderChannelB: Int
                   ) {

    val driveMotor = SafeTalonFX(driveFalconID)
    val turningMotor = SafeSparkMax(turningMotorID)

    val driveEncoder = Encoder(driveEncoderChannelA, driveEncoderChannelB)
    val turningEncoder = Encoder(turningEncoderChannelA, turningEncoderChannelB)


    // TODO: Tune PIDController
    private val drivePIDController = PIDController(1.0, 0.0, 0.0)
    private val turningPIDController = ProfiledPIDController(1.0, 0.0, 0.0,
        TrapezoidProfile.Constraints(Constants.MODULE_MAX_ANGULAR_VELOCITY, Constants.MODULE_MAX_ANGULAR_ACCELERATION))

    val driveFeedforward = SimpleMotorFeedforward(1.0,3.0)
    val turnFeedforward = SimpleMotorFeedforward(1.0, 0.5)

    init {
        driveEncoder.distancePerPulse = 2 * PI * Constants.WHEEL_RADIUS_FT / Constants.ENCODER_RESOLUTION
        turningEncoder.distancePerPulse = 2 * PI / Constants.ENCODER_RESOLUTION

        turningPIDController.enableContinuousInput(-PI, PI)

    }

    fun getState(): SwerveModuleState {
        return SwerveModuleState(driveEncoder.rate, Rotation2d(turningEncoder.get().toDouble()))
    }

    fun setDesiredState(desiredState: SwerveModuleState) {
        val state = SwerveModuleState.optimize(desiredState, Rotation2d(turningEncoder.get().toDouble()))

        val driveOutput = drivePIDController.calculate(driveEncoder.rate, state.speedMetersPerSecond)
        val driveFeedforward = driveFeedforward.calculate(state.speedMetersPerSecond)

        val turnOutput = turningPIDController.calculate(turningEncoder.get().toDouble(), state.angle.radians)
        val turnFeedforward = turnFeedforward.calculate(turningPIDController.setpoint.velocity)

        driveMotor.setVoltage(driveOutput + driveFeedforward)
        turningMotor.setVoltage(turnOutput + turnFeedforward)

    }



}