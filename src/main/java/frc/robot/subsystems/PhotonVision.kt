package frc.robot.subsystems

import edu.wpi.first.networktables.NetworkTableInstance
import org.photonvision.PhotonCamera

class PhotonVision {
    private val camera = PhotonCamera("HD_Pro_Webcam_C920")

    fun getYaw(): Double { return camera.latestResult.bestTarget.yaw }

    fun getPitch(): Double { return camera.latestResult.bestTarget.pitch }


}