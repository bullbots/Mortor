package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.NeutralMode
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.util.SafeTalonFX
import java.util.stream.Collectors
import java.util.stream.IntStream

class Shooter : SubsystemBase() {

    private var shooter_spinner: SafeTalonFX? = null

    private var shooterVelocity: NetworkTableEntry? = null

    var velocityRange = IntStream.rangeClosed(-100, 100).boxed().collect(Collectors.toList())
    private var velIndex = 0

    init {
        shooter_spinner = SafeTalonFX(Constants.SHOOTER_PORT, true)

        configurePID()

        shooter_spinner?.setNeutralMode(NeutralMode.Coast)

        var inst = NetworkTableInstance.getDefault()

    }

    private fun configurePID() {
        shooter_spinner?.config_kF(0, Constants.SHOOTER_FF)
        shooter_spinner?.config_kP(0, Constants.SHOOTER_P)
        shooter_spinner?.config_kI(0, Constants.SHOOTER_I)
        shooter_spinner?.config_kD(0, Constants.SHOOTER_D)
    }

    fun stop() {
        shooter_spinner?.stopMotor()
    }

    /**
     *
     */
    fun getVelocity(): Double {
        var curVal = 0.0
        if (RobotBase.isReal()) {
            curVal = shooter_spinner?.selectedSensorVelocity!!
        } else {
            var curIndex = 0
            curIndex = velIndex
        }


    }
}