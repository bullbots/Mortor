package frc.robot.commands.Drivetrain_Commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.robot.subsystems.DrivetrainFalcon
import java.util.function.DoubleConsumer
import java.util.function.DoubleSupplier

class AlignShooter(controller: PIDController, measurementSource: DoubleSupplier, setpointSource: DoubleSupplier,
                   useOutput: DoubleConsumer, drivetrain: DrivetrainFalcon) :
    PIDCommand(controller, measurementSource, setpointSource, useOutput, drivetrain) {


    override fun initialize() {
        super.initialize()

        println("INFO: AlignShooter is being called")
    }

    override fun execute() {
        var pidOut = m_controller.calculate(m_measurement.asDouble, m_setpoint.asDouble)
        m_useOutput.accept(pidOut)
        println("INFO: Yaw: ${m_measurement.asDouble}, Heading: ${m_setpoint.asDouble}, Output value: $pidOut")
    }

    override fun isFinished(): Boolean { return m_controller.atSetpoint() }

    override fun end(interrupted: Boolean) {
        println("INFO: AlignShooter end")
    }
}