package org.firstinspires.ftc.teamcode.Auto.newAuto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// BLUE alliance. Locations are Pedro field coordinates in INCHES: (0,0) is a field corner, the field is 144 x 144.
// All the shooting and path code lives in PedroPathingTestOneBase.
@Autonomous(name = "PedroPathingTestOneBlue")
@Config
public class PedroPathingTestOneBlue extends PedroPathingTestOneBase {

    public static double HEADING_DEG = 90.0;   // placeholder for location(1): 90 = facing up the field. Robot keeps this heading the whole auto.

    public static double START_X = 72.0;       // placeholder for location(1)
    public static double START_Y = 8.0;        // placeholder for location(1)

    public static double STRAFE_X = 48.0;      // placeholder for location(2)
    public static double STRAFE_Y = 8.0;       // placeholder for location(2)

    public static double FINAL_X = 48.0;       // placeholder for location(3)
    public static double FINAL_Y = 40.0;       // placeholder for location(3)

    @Override
    protected Pose getStartPose() {
        return new Pose(START_X, START_Y, Math.toRadians(HEADING_DEG)); // Pedro wants radians, so we convert
    }

    @Override
    protected Pose getStrafePose() {
        return new Pose(STRAFE_X, STRAFE_Y, Math.toRadians(HEADING_DEG));
    }

    @Override
    protected Pose getFinalPose() {
        return new Pose(FINAL_X, FINAL_Y, Math.toRadians(HEADING_DEG));
    }
}
