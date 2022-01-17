package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.robot.subsystems.DrivetrainFalcon;

public class RobotContainer {

    // User Input Joystick Controller
    private static Joystick stick = new Joystick(0);

    private static JoystickButton button1 = new JoystickButton(stick, 1);
    private static JoystickButton button2 = new JoystickButton(stick, 2);
    private static JoystickButton button3 = new JoystickButton(stick, 3);
    private static JoystickButton button4 = new JoystickButton(stick, 4);
    private static JoystickButton button6 = new JoystickButton(stick, 6);
    private static JoystickButton button10 = new JoystickButton(stick, 10);

    // Subsystems
    //private final DrivetrainFalcon drivetrainFalcon = new DrivetrainFalcon();
}
