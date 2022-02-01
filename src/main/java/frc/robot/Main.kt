package frc.robot

import edu.wpi.first.wpilibj.RobotBase


/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot call.
 */
class Main {
    private fun Main() {}

    companion object {
        fun main(args: Array<String>) {
            RobotBase.startRobot { Robot() }
        }
    }

}

