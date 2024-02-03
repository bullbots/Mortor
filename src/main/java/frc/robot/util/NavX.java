package frc.robot.util;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Used to find the angle of the robot in its current position
 * to keep orientated
 */
public class NavX extends AHRS {

    private double angleDelta = 0.0;


    @Override 
    public double getAngle() {
        double angle = super.getAngle() - angleDelta;
        angle = MathUtil.inputModulus(angle, -180.0, 180.0);

        return angle;
    }

//    override fun reset() { angleDelta = super.getAngle() }

}
