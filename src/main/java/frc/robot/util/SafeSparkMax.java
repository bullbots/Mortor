package frc.robot.util;

import com.revrobotics.CANSparkMax;


/**
 * Wrapper used to prevent NEOs from burning out or being damaged
 * @param deviceNumber Int
 * @param motorType MotorType: By default this value is kBrushless
 */
public class SafeSparkMax extends CANSparkMax {

    private int stallLimit = 40;
    private int freeLimit = 40;


    public SafeSparkMax(int deviceNumber, MotorType motorType) {
	super(deviceNumber, motorType);
    // MotorType.kBrushless this was the default motor type
        restoreFactoryDefaults();
        clearFaults();
//        setSmartCurrentLimit(stallLimit, freeLimit)
        burnFlash();
    }
}
