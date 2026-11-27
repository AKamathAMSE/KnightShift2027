package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

// This file tells Pedro about YOUR robot: its weight, its drive motors, and its dead wheels.
// Every line marked "placeholder" must be replaced with a real number from your robot.
// If an import is red, click it and press Alt+Enter: Android Studio will offer the right one.
public class Constants {

    // ---- Robot weight and tuning numbers ----
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5.0); // placeholder: robot weight in KILOGRAMS (weigh it on a scale)
    // The tuners in the Tuning OpMode will give you more numbers to add here later
    // (forwardZeroPowerAcceleration, lateralZeroPowerAcceleration, PID values).

    // ---- When a path counts as "finished" (leave these defaults for now) ----
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 3, 1);

    // ---- Drive motors (names and directions copied from your ShootStrafeAuto) ----
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1) // 1 = allowed to use full motor power
            .leftFrontMotorName("LF") // must match the Robot Configuration exactly
            .leftRearMotorName("LB") // must match the Robot Configuration exactly
            .rightFrontMotorName("RF") // must match the Robot Configuration exactly
            .rightRearMotorName("RB") // must match the Robot Configuration exactly
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD) // copied from ShootStrafeAuto, verify with the Tuning OpMode
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE) // copied from ShootStrafeAuto, verify with the Tuning OpMode
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD) // copied from ShootStrafeAuto, verify with the Tuning OpMode
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE); // copied from ShootStrafeAuto, verify with the Tuning OpMode

    // ---- Dead wheel odometry (2 wheels + Control Hub IMU) ----
    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("LF") // placeholder: name of the motor PORT the FORWARD dead wheel's encoder cable plugs into (ports 0 or 3 are fastest)
            .strafeEncoder_HardwareMapName("RF") // placeholder: name of the motor PORT the SIDEWAYS dead wheel's encoder cable plugs into (ports 0 or 3 are fastest)
            .forwardEncoderDirection(Encoder.FORWARD) // placeholder direction: push robot forward, if x goes DOWN in the Localization Test, change to Encoder.REVERSE
            .strafeEncoderDirection(Encoder.FORWARD) // placeholder direction: push robot left, if y goes DOWN in the Localization Test, change to Encoder.REVERSE
            .forwardTicksToInches(Math.PI * (50.8 / 25.4) / 8192.0) // inches per tick = wheel circumference (pi x 2 in) / 8192 ticks per rev (REV Through Bore)
            .strafeTicksToInches(Math.PI * (50.8 / 25.4) / 8192.0) // same math for the sideways wheel
            .forwardY(0.0) // placeholder: how many inches the FORWARD wheel sits LEFT (+) or RIGHT (-) of the robot's center. Measure it.
            .strafeX(0.0) // placeholder: how many inches the SIDEWAYS wheel sits in FRONT (+) or BEHIND (-) the robot's center. Measure it.
            .IMU_HardwareMapName("imu") // the Control Hub's built-in IMU. In the Robot Configuration it must be on I2C port 0, named "imu"
            .IMU_Orientation(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP, // same as your other files: REV logo face up
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)); // same as your other files: USB ports toward the back

    // This method builds the "follower", the object that drives the robot along paths.
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(localizerConstants)
                .build();
    }
}
