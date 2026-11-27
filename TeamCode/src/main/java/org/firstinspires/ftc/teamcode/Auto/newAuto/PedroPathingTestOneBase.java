package org.firstinspires.ftc.teamcode.Auto.newAuto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// This is the SHARED code. It is NOT an OpMode you run (it has no @Autonomous).
// PedroPathingTestOneBlue and PedroPathingTestOneRed both "extend" it, which means they reuse
// everything here and only supply their own three locations.
//
// What the auto does:
//   1. Start at location 1, rev the shooter 4 s, then shoot (gate open + intake) 4 s.
//   2. Strafe to location 2 (heading stays locked, so the robot slides without turning).
//   3. Drive forward to location 3 and hold there.
@Config
public abstract class PedroPathingTestOneBase extends LinearOpMode {

    // Each alliance class fills these in with its own locations.
    protected abstract Pose getStartPose();   // location 1
    protected abstract Pose getStrafePose();  // location 2
    protected abstract Pose getFinalPose();   // location 3

    public static double PATH_TIMEOUT_S = 6.0; // give up on a path after this long (stuck robot, bad odometry)

    // ---- Shooter + feed (same values and meaning as ShootStrafeAuto) ----
    public static double SHOOTER_VELOCITY = 1760.0; // ticks per second, not RPM
    public static double SHOOTER_REV_SECONDS = 4.0; // spin-up time before the gate opens
    public static double FEED_SECONDS = 4.0;        // gate open + intake running
    public static double GATE_HOME_POSITION = 0.0;  // gate closed
    public static double GATE_CLOCKWISE_POSITION = 0.5; // gate open
    public static double INTAKE_POWER = 1.0;
    public static double shooterP = 0.001;
    public static double shooterI = 0.0;
    public static double shooterD = 0.0;
    public static double shooterF = 0.0;

    private Follower follower;
    private DcMotorEx shooterl, shooterr, intakel, intaker;
    private Servo gate;

    @Override
    public void runOpMode() {

        // ---- Hardware (names must match the Robot Configuration exactly) ----
        shooterl = hardwareMap.get(DcMotorEx.class, "Shooterl");
        shooterr = hardwareMap.get(DcMotorEx.class, "Shooterr");
        intakel = hardwareMap.get(DcMotorEx.class, "Intakel");
        intaker = hardwareMap.get(DcMotorEx.class, "Intaker");
        gate = hardwareMap.get(Servo.class, "Gate");

        shooterl.setDirection(DcMotor.Direction.FORWARD);
        shooterr.setDirection(DcMotor.Direction.REVERSE);
        intakel.setDirection(DcMotor.Direction.FORWARD);
        intaker.setDirection(DcMotor.Direction.REVERSE);
        gate.setDirection(Servo.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{shooterl, shooterr, intakel, intaker}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        gate.setPosition(GATE_HOME_POSITION);

        // ---- Pedro setup ----
        Pose startPose = getStartPose();
        Pose strafePose = getStrafePose();
        Pose finalPose = getFinalPose();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose); // tells odometry "the robot is HERE right now"

        // Constant heading = the robot never turns, so a sideways move is a true strafe.
        PathChain strafePath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, strafePose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        PathChain forwardPath = follower.pathBuilder()
                .addPath(new BezierLine(strafePose, finalPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        telemetry.addLine("Ready. Alliance: " + getClass().getSimpleName());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ---- The actual auto ----
        shootAndFeed();                                 // rev 4 s, shoot 4 s
        driveTo(strafePath, false, "strafe to location 2");
        driveTo(forwardPath, true, "forward to location 3");

        // Keep holding the final spot until the 30 seconds run out.
        while (opModeIsActive()) {
            follower.update();
            telemetry.addLine("Done, holding position");
            telemetry.addData("x / y / heading(deg)", "%.1f / %.1f / %.1f",
                    follower.getPose().getX(), follower.getPose().getY(),
                    Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }

    // Start a path, then keep calling follower.update() until the robot arrives (or the timeout hits).
    // follower.update() MUST be called every loop or the robot will not move.
    private void driveTo(PathChain path, boolean holdEnd, String label) {
        follower.followPath(path, holdEnd);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && follower.isBusy() && timer.seconds() < PATH_TIMEOUT_S) {
            follower.update();
            telemetry.addData("Leg", label);
            telemetry.addData("x / y / heading(deg)", "%.1f / %.1f / %.1f",
                    follower.getPose().getX(), follower.getPose().getY(),
                    Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
        if (follower.isBusy()) follower.breakFollowing(); // timed out: stop following
    }

    // Phase 1: shooter revs alone. Phase 2: gate opens, intake runs, shooter keeps spinning.
    private void shootAndFeed() {

        BasicPIDController pidL = new BasicPIDController(shooterP, shooterI, shooterD);
        BasicPIDController pidR = new BasicPIDController(shooterP, shooterI, shooterD);
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < SHOOTER_REV_SECONDS) {
            runShooter(pidL, pidR, SHOOTER_VELOCITY);
            telemetry.addData("Phase", "revving shooter");
            telemetry.addData("Shooter L/R vel", "%.0f / %.0f", shooterl.getVelocity(), shooterr.getVelocity());
            telemetry.update();
        }

        gate.setPosition(GATE_CLOCKWISE_POSITION);
        intakel.setPower(INTAKE_POWER);
        intaker.setPower(INTAKE_POWER);
        timer.reset();

        while (opModeIsActive() && timer.seconds() < FEED_SECONDS) {
            runShooter(pidL, pidR, SHOOTER_VELOCITY);
            telemetry.addData("Phase", "feeding (gate open, intake on)");
            telemetry.addData("Shooter L/R vel", "%.0f / %.0f", shooterl.getVelocity(), shooterr.getVelocity());
            telemetry.update();
        }

        shooterl.setPower(0);
        shooterr.setPower(0);
        intakel.setPower(0);
        intaker.setPower(0);
        gate.setPosition(GATE_HOME_POSITION);
    }

    private void runShooter(BasicPIDController pidL, BasicPIDController pidR, double target) {
        pidL.setPID(shooterP, shooterI, shooterD);
        pidR.setPID(shooterP, shooterI, shooterD);
        shooterl.setPower(pidL.update(target, shooterl.getVelocity()) + (target * shooterF));
        shooterr.setPower(pidR.update(target, shooterr.getVelocity()) + (target * shooterF));
    }

    // Same custom PID controller as ShootStrafeAuto / KnightShiftTeleop.
    public static class BasicPIDController {
        private double Kp, Ki, Kd;
        private double lastError = 0;
        private double integral = 0;
        private ElapsedTime timer = new ElapsedTime();

        public BasicPIDController(double Kp, double Ki, double Kd) {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
        }

        public void setPID(double Kp, double Ki, double Kd) {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
        }

        public double update(double reference, double state) {
            double error = reference - state;
            double dt = timer.seconds();
            timer.reset();
            integral += error * dt;
            double derivative = (dt != 0) ? (error - lastError) / dt : 0.0;
            lastError = error;
            return (Kp * error) + (Ki * integral) + (Kd * derivative);
        }
    }
}
