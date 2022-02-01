package frc.robot

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.button.JoystickButton

class RobotContainer {

    // Any static variables need to go in the companion object
    companion object {
        val stick = Joystick(0)
        val button1 = JoystickButton(stick, 1)
        val button2 = JoystickButton(stick, 2)
        val button3 = JoystickButton(stick, 3)
        val button4 = JoystickButton(stick, 4)
        val button6 = JoystickButton(stick, 6)
        val button10 = JoystickButton(stick, 10)
    }

    // val drivetrain = DrivetrainFalcon();
}