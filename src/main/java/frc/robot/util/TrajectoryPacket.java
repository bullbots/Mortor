package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.*;
import java.util.ArrayList;

import java.lang.Math;


public class TrajectoryPacket {

    public static double lastX = 0.0;
    public static double lastY = 0.0;
    public static double startAngle = 0.0;
    public static double endAngle = 0.0;
    public static boolean reversed = false;
    public double firstX = 0.0;
    public double firstY = 0.0;
    public ArrayList<Translation2d> pathRead;

    public static TrajectoryPacket generateTrajectoryPacket(File file) {

        var pathRead = new ArrayList<Translation2d>();
        var angleList = new ArrayList<Double>();

        BufferedReader br = null;

        try {
            br = new BufferedReader(new FileReader(file));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        var reversed = false;

        try {
            String line;
            while ((line = br.readLine()) != null) {
                var sections = line.split(",");

                if (sections[0] == "X") {
                    continue;
                }

                var x = Double.parseDouble(sections[0]);
                var y = Double.parseDouble(sections[1]) + 27;

                pathRead.add(new Translation2d(x, y));

                var tangentX = Double.parseDouble(sections[2]);
                var tangentY = Double.parseDouble(sections[3]);

                var angle = Math.atan2(tangentY, tangentX);

                angleList.add(Math.toDegrees(angle));

                reversed = Boolean.parseBoolean(sections[5]);
            }

        } catch (IOException error) {
            // System.out.println("Ignore this error:");
            error.printStackTrace();
        }

        var trajectoryPacket = new TrajectoryPacket();

        trajectoryPacket.firstX = pathRead.get(0).getX();
        trajectoryPacket.firstY = pathRead.get(0).getY();
        lastX = pathRead.get(pathRead.size() - 1).getX();
        lastY = pathRead.get(pathRead.size() - 1).getY();

        pathRead.remove(0);
        pathRead.remove(pathRead.size() - 1);

        trajectoryPacket.pathRead = pathRead;

        startAngle = angleList.get(0);
        endAngle = angleList.get(angleList.size() - 1);

        TrajectoryPacket.reversed = reversed;

        return trajectoryPacket;
    }
}
