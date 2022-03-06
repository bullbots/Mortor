package frc.robot.commands.Drivetrain_Commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.robot.subsystems.DrivetrainFalcon
import java.util.function.DoubleConsumer
import java.util.function.DoubleSupplier
import kotlin.math.abs
import kotlin.math.sign

class AlignShooter(controller: PIDController, measurementSource: DoubleSupplier, setpointSource: DoubleSupplier,
                   useOutput: DoubleConsumer, var drivetrain: DrivetrainFalcon) :
    PIDCommand(controller, measurementSource, setpointSource, useOutput, drivetrain) {

    private var loopIdx = 0

    override fun initialize() {
        super.initialize()
        println("INFO: AlignShooter is being called")
    }

    override fun execute() {
        val pidOut = m_controller.calculate(m_measurement.asDouble, m_setpoint.asDouble)
        val delta = MathUtil.inputModulus(m_setpoint.asDouble - m_measurement.asDouble, -180.0, 180.0)
        var ff = 0.0

        if(abs(delta) > 5 ) {
            ff = sign(delta) * 0.3
        }

//        m_useOutput.accept(0.6)
        m_useOutput.accept(pidOut + ff)

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

    override fun isFinished(): Boolean { return m_controller.atSetpoint() }

    override fun end(interrupted: Boolean) {
        println("INFO: AlignShooter end")
        drivetrain.stop()
    }
}