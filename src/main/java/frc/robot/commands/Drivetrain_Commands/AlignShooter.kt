package frc.robot.commands.Drivetrain_Commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.robot.subsystems.DrivetrainFalcon
import frc.robot.util.PIDControllerDebug
import java.util.function.DoubleConsumer
import java.util.function.DoubleSupplier
import kotlin.math.abs
import kotlin.math.sign

// TODO: REMOVE PID CONTROLLER
class AlignShooter(private val measurementSource: DoubleSupplier,
                   private val setpointSource: DoubleSupplier,
                   private val drivetrain: DrivetrainFalcon) :
    CommandBase() {

    private var loopIdx = 0
    private var delta = 0.0

    override fun initialize() {
        println("INFO: AlignShooter is being called")
        delta = MathUtil.inputModulus(setpointSource.asDouble - measurementSource.asDouble, -180.0, 180.0)
    }

    override fun execute() {

        delta = MathUtil.inputModulus(setpointSource.asDouble - measurementSource.asDouble, -180.0, 180.0)

        val output = if (abs(delta) > 45) {
            sign(delta) * 0.575
        } else if(abs(delta) > 30) {
            sign(delta) * 0.375
        } else if(abs(delta) > 1) {
            sign(delta) * 0.26
        } else {
            0.0
        }
        drivetrain.drive(0.0, -output)

//        // Debugging values
//        loopIdx++
//        if (loopIdx == 10) {
//            loopIdx = 0
//            println("INFO: Yaw: ${m_measurement.asDouble}, Heading: ${m_setpoint.asDouble}, PidOut: $pidOut, FF: $ff, TotalOut: ${pidOut+ff}")
//            println("INFO: Yaw: ${m_measurement.asDouble}, Heading: ${m_setpoint.asDouble}, Delta: $delta")
//            println("INFO: Yaw: ${m_measurement.asDouble}, Heading: ${m_setpoint.asDouble}, Output value: ${pidOut + ff}")
//
//        }
    }

    override fun isFinished(): Boolean {
        return abs(delta) < 1 &&
        abs(drivetrain.getVelocities()[0]) < 0.015
    }

    override fun end(interrupted: Boolean) {
        println("INFO: AlignShooter end")
        drivetrain.stop()
    }
}