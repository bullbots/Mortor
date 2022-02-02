package frc.robot

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.subsystems.DrivetrainFalcon
import java.util.concurrent.atomic.AtomicReference
import edu.wpi.first.wpilibj.Counter;

class RobotContainer {

    // Any static variables need to go in the companion object
    companion object {
        private val stick = Joystick(0)
        private val button1 = JoystickButton(stick, 1)
        private val button2 = JoystickButton(stick, 2)
        private val button3 = JoystickButton(stick, 3)
        private val button4 = JoystickButton(stick, 4)
        private val button6 = JoystickButton(stick, 6)
        private val button10 = JoystickButton(stick, 10)

        private val pathColor = AtomicReference<Color>(Color.UNLOADED)


    }

    private val drivetrain = DrivetrainFalcon()

    var m_chooser = SendableChooser<Command>()

    enum class Color(value: Int) {
        UNLOADED(0),
        RED(1),
        BLUE(2);

        private var value: Integer? = null
        companion object {
            private var map = HashMap<Integer, Color>()

            // In replace of static {}
            fun addColor() {
                for (color in Color.values()) {
                    map[color.value!!] = color
                }
            }

            fun valueOf(color: Integer): Color { return map[color]!! }
        }

        fun getValue(): Int { return value!!.toInt() }

    }

    private enum class Letter(value: Int) {
        UNLOADED(0),
        A(1),
        B(2);

        private var value: Integer? = null

        companion object {
            private var
            fun addLetter() {
                for (letter in Letter.values()) {
                    map[letter.value!!]
                }
            }
        }
    }









}