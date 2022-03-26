package frc.robot.commands.Drivetrain_Commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.robot.subsystems.DrivetrainFalcon
import frc.robot.util.PIDControllerDebug
import java.util.function.DoubleConsumer
import java.util.function.DoubleSupplier
import kotlin.math.abs
import kotlin.math.sign

class AlignShooter(private val debugController: PIDControllerDebug, measurementSource: DoubleSupplier, setpointSource: DoubleSupplier,
                   useOutput: DoubleConsumer, var drivetrain: DrivetrainFalcon) :
    PIDCommand(debugController, measurementSource, setpointSource, useOutput, drivetrain) {

    private var loopIdx = 0
    private var delta = 0.0

    override fun initialize() {
        super.initialize()
        debugController.p = SmartDashboard.getNumber("PID P Value", 0.0)
        debugController.i = SmartDashboard.getNumber("PID I Value", 0.0)
        debugController.d = SmartDashboard.getNumber("PID D Value", 0.0)
        println("INFO: AlignShooter is being called")
        delta = MathUtil.inputModulus(m_setpoint.asDouble - m_measurement.asDouble, -180.0, 180.0)
    }

    override fun execute() {
//        val pidOut = debugController.calculateDebug(m_measurement.asDouble, m_setpoint.asDouble, true)

        delta = MathUtil.inputModulus(m_setpoint.asDouble - m_measurement.asDouble, -180.0, 180.0)

        val output = if (abs(delta) > 45) {
            sign(delta) * 0.575
        } else if(abs(delta) > 1){
            sign(delta) * 0.26
        } else {
            0.0
        }

//        val delta = MathUtil.inputModulus(m_setpoint.asDouble - m_measurement.asDouble, -180.0, 180.0)
        var ff = 0.0



//        ff = when (delta) {
//            in 360.0..10.0 -> sign(delta) * 0.3
//            in 10.0..5.0 -> sign(delta) * 0.2
//            else -> 0.0
//        }


//        if(abs(delta) > 15 ) {
//            ff = sign(delta) * 0.3
//        }

//        m_useOutput.accept(0.6)
        m_useOutput.accept(output)

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