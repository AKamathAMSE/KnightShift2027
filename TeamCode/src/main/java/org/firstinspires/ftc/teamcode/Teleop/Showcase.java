package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Showcase")
public class Showcase extends LinearOpMode {

    private DcMotorEx LB, LF, RB, RF;
    private Servo Gate;
    private DcMotorEx Shooterl, Shooterr;
    private DcMotorEx Intakel, Intaker;
    private CRServo Arm;

    double lbPower, lfPower, rbPower, rfPower;
    double ilPower, irPower;

    // Shooter RPM & encoder conversion (update to match shooter motor/gearing)
    public static double DEFAULT_SHOOTER_RPM = 1760.0;
    public static double UNJAM_SHOOTER_RPM = -1200.0;
    public static double TICKS_PER_MOTOR_REV = 28.0; // e.g., goBILDA 5202/5203/5204 yellow jacket base
    public static double SHOOTER_GEAR_RATIO = 1.0;   // motor revs per 1 shooter wheel rev
    public static double REV_BEFORE_GATE_SECONDS = 0.35;
    public static double AUTO_SEQUENCE_REV_SECONDS = 2.5;

    // Intake & feed powers
    public static double INTAKE_TOGGLE_POWER = 1.0;
    public static double FEED_INTAKE_POWER = 0.55; // medium power/RPM while feeding
    public static double UNJAM_INTAKE_POWER = -0.8;

    // Arm variables (flower arm toggle)
    public static double DEGREES_PER_SECOND = 90.0;
    public static double ARM_POWER = 0.6;
    public static double ARM_DOWN_ANGLE = 90.0;
    public static double ARM_UP_ANGLE = 0.0;

    private ElapsedTime armTimer = new ElapsedTime();
    private boolean isArmMoving = false;
    private boolean isArmDown = false;
    private double armTargetTime = 0.0;
    private double currentEstimatedAngle = 0.0;
    private double targetAngle = 0.0;

    // Gate servo positions (lift gate by X degrees to clear into shooting chamber)
    public static double GATE_HOME_POSITION = 0.0;
    public static double GATE_LIFT_DEGREES = 45.0;
    public static double SERVO_MAX_DEGREES = 270.0;
    public static double GATE_OPEN_POSITION =
            Math.min(1.0, GATE_HOME_POSITION + (GATE_LIFT_DEGREES / SERVO_MAX_DEGREES));

    // Shooter PID coefficients
    public static double shooterP = 0.001;
    public static double shooterI = 0.0;
    public static double shooterD = 0.0;
    public static double shooterF = 0.0;

    // Toggle & timer states
    private boolean intakeToggledOn = false;
    private boolean prevRbTogglePressed = false;
    private boolean prevLbTogglePressed = false;

    private boolean isFeedingHold = false;
    private boolean isAutoSequenceActive = false;
    private boolean gateIsOpen = false;

    private ElapsedTime revDelayTimer = new ElapsedTime();
    private boolean revDelayStarted = false;
    private ElapsedTime autoSequenceTimer = new ElapsedTime();

    private double targetShooterRpm = DEFAULT_SHOOTER_RPM;

    @Override
    public void runOpMode() {

        ElapsedTime runtime = new ElapsedTime();

        // Hardware mapping
        LB = hardwareMap.get(DcMotorEx.class, "LB");
        LF = hardwareMap.get(DcMotorEx.class, "LF");
        RB = hardwareMap.get(DcMotorEx.class, "RB");
        RF = hardwareMap.get(DcMotorEx.class, "RF");

        Arm = hardwareMap.get(CRServo.class, "Arm");
        Gate = hardwareMap.get(Servo.class, "Gate");

        Intakel = hardwareMap.get(DcMotorEx.class, "Intakel");
        Intaker = hardwareMap.get(DcMotorEx.class, "Intaker");

        Shooterr = hardwareMap.get(DcMotorEx.class, "Shooterr");
        Shooterl = hardwareMap.get(DcMotorEx.class, "Shooterl");

        // Drive motor directions (left reversed, right forward for mirrored mounting)
        LB.setDirection(DcMotor.Direction.REVERSE);
        LF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.FORWARD);

        Intakel.setDirection(DcMotor.Direction.FORWARD);
        Intaker.setDirection(DcMotor.Direction.REVERSE);

        Shooterl.setDirection(DcMotor.Direction.FORWARD);
        Shooterr.setDirection(DcMotor.Direction.REVERSE);

        Arm.setDirection(CRServo.Direction.FORWARD);
        Gate.setDirection(Servo.Direction.FORWARD);

        // Reset encoders
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Intakel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intaker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Shooterl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shooterr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set run modes
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Intakel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intaker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Shooterl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooterr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Gate.setPosition(GATE_HOME_POSITION);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        BasicPIDController shooterPID =
                new BasicPIDController(shooterP, shooterI, shooterD);

        while (opModeIsActive()) {

            shooterPID.setPID(shooterP, shooterI, shooterD);

            updateDrive();
            updateArmToggle();
            updateMechanismActions();
            shooterMethod(shooterPID);
            applyHardwareOutputs();

            // Telemetry
            telemetry.addData("Run Time", runtime);
            telemetry.addData("Intake Toggled", intakeToggledOn);
            telemetry.addData("Arm Down", isArmDown);
            telemetry.addData("Shooter Target RPM", targetShooterRpm);
            telemetry.addData("Shooter Left TPS", Shooterl.getVelocity());
            telemetry.addData("Shooter Right TPS", Shooterr.getVelocity());
            telemetry.addData("Gate Open", gateIsOpen);
            telemetry.addData("Auto Seq Active", isAutoSequenceActive);
            telemetry.update();
        }
    }

    // Intake is front, shooter is back: Left stick = move, Right stick = rotate
    private void updateDrive() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        lbPower = forward - strafe + turn;
        lfPower = forward + strafe + turn;
        rbPower = forward + strafe - turn;
        rfPower = forward - strafe - turn;

        double max = Math.max(
                Math.max(Math.abs(lfPower), Math.abs(rfPower)),
                Math.max(Math.abs(lbPower), Math.abs(rbPower))
        );

        if (max > 1.0) {
            lbPower /= max;
            lfPower /= max;
            rbPower /= max;
            rfPower /= max;
        }
    }

    // LB toggles flower arm down / up on button press
    private void updateArmToggle() {
        boolean lbPressed = gamepad1.left_bumper;

        if (lbPressed && !prevLbTogglePressed) {
            isArmDown = !isArmDown;
            double requestedAngle = isArmDown ? ARM_DOWN_ANGLE : ARM_UP_ANGLE;

            if (requestedAngle != currentEstimatedAngle) {
                targetAngle = requestedAngle;
                double angleDifference = targetAngle - currentEstimatedAngle;
                double direction = Math.signum(angleDifference);

                Arm.setPower(ARM_POWER * direction);
                armTargetTime = Math.abs(angleDifference) / DEGREES_PER_SECOND;
                armTimer.reset();
                isArmMoving = true;
            }
        }
        prevLbTogglePressed = lbPressed;

        if (isArmMoving && armTimer.seconds() >= armTargetTime) {
            Arm.setPower(0.0);
            currentEstimatedAngle = targetAngle;
            isArmMoving = false;
        }
    }

    // Handles RB toggle/feed, LB rev hold, B unjam hold, A align+aim, X full sequence
    private void updateMechanismActions() {
        boolean rbDown = gamepad1.right_bumper;
        boolean lbDown = gamepad1.left_bumper;
        boolean bDown = gamepad1.b;

        // Toggle intake on/off when RB is tapped
        if (rbDown && !prevRbTogglePressed) {
            intakeToggledOn = !intakeToggledOn;
        }
        prevRbTogglePressed = rbDown;

        // Start full auto sequence on X press
        if (gamepad1.x && !isAutoSequenceActive) {
            isAutoSequenceActive = true;
            autoSequenceTimer.reset();
            revDelayStarted = false;
        } else if (!gamepad1.x) {
            isAutoSequenceActive = false;
        }

        // Default baseline each loop
        targetShooterRpm = DEFAULT_SHOOTER_RPM;
        ilPower = intakeToggledOn ? INTAKE_TOGGLE_POWER : 0.0;
        irPower = intakeToggledOn ? INTAKE_TOGGLE_POWER : 0.0;
        gateIsOpen = false;
        isFeedingHold = false;

        // 1. Highest priority: B (while held) -> Unjam (reverse intake & shooter)
        if (bDown) {
            isAutoSequenceActive = false;
            revDelayStarted = false;
            targetShooterRpm = UNJAM_SHOOTER_RPM;
            ilPower = UNJAM_INTAKE_POWER;
            irPower = UNJAM_INTAKE_POWER;
            gateIsOpen = false;
            return;
        }

        // 2. X: Auto align + auto aim RPM adjust + rev for 2.5s + feed (with delayed gate)
        if (isAutoSequenceActive) {
            autoAlignStep();
            targetShooterRpm = getAutoAimTargetRpm();

            if (!revDelayStarted) {
                revDelayTimer.reset();
                revDelayStarted = true;
            }

            if (autoSequenceTimer.seconds() >= AUTO_SEQUENCE_REV_SECONDS) {
                // After 2.5s rev, feed into chamber
                ilPower = FEED_INTAKE_POWER;
                irPower = FEED_INTAKE_POWER;
                gateIsOpen = (revDelayTimer.seconds() >= REV_BEFORE_GATE_SECONDS);
            } else {
                // Revving shooter before feeding; open gate after brief rev delay
                gateIsOpen = (revDelayTimer.seconds() >= REV_BEFORE_GATE_SECONDS);
            }
            return;
        }

        // 3. A: Auto align + auto aim RPM adjust
        if (gamepad1.a) {
            autoAlignStep();
            targetShooterRpm = getAutoAimTargetRpm();
        }

        // 4. LB (while held): Rev shooter (gate opens after brief rev delay)
        if (lbDown) {
            if (!revDelayStarted) {
                revDelayTimer.reset();
                revDelayStarted = true;
            }
            gateIsOpen = (revDelayTimer.seconds() >= REV_BEFORE_GATE_SECONDS);
        }

        // 5. RB (while held): Feed at medium power + lift gate (after brief rev delay)
        if (rbDown) {
            isFeedingHold = true;
            if (!revDelayStarted) {
                revDelayTimer.reset();
                revDelayStarted = true;
            }
            ilPower = FEED_INTAKE_POWER;
            irPower = FEED_INTAKE_POWER;
            gateIsOpen = (revDelayTimer.seconds() >= REV_BEFORE_GATE_SECONDS);
        }

        // Reset rev delay timer once neither LB, RB, nor X is active
        if (!lbDown && !rbDown && !isAutoSequenceActive) {
            revDelayStarted = false;
            gateIsOpen = false;
        }
    }

    private void autoAlignStep() {
        // Hook vision/odometry heading alignment adjustment here when ready
    }

    private double getAutoAimTargetRpm() {
        // Hook distance-based RPM calculation here when ready; defaults to 1760 RPM
        return DEFAULT_SHOOTER_RPM;
    }

    private double rpmToTicksPerSecond(double rpm) {
        return (rpm / 60.0) * TICKS_PER_MOTOR_REV * SHOOTER_GEAR_RATIO;
    }

    // Shooter PID control
    private void shooterMethod(BasicPIDController pid) {
        double targetVelTps = rpmToTicksPerSecond(targetShooterRpm);
        double currentVelL = Shooterl.getVelocity();
        double currentVelR = Shooterr.getVelocity();

        double powerL = pid.update(targetVelTps, currentVelL)
                + (targetVelTps * shooterF);
        double powerR = pid.update(targetVelTps, currentVelR)
                + (targetVelTps * shooterF);

        Shooterl.setPower(powerL);
        Shooterr.setPower(powerR);
    }

    private void applyHardwareOutputs() {
        LB.setPower(lbPower);
        LF.setPower(lfPower);
        RB.setPower(rbPower);
        RF.setPower(rfPower);

        Intakel.setPower(ilPower);
        Intaker.setPower(irPower);

        Gate.setPosition(gateIsOpen ? GATE_OPEN_POSITION : GATE_HOME_POSITION);
    }

    // Custom PID controller
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

            double derivative =
                    (dt != 0) ? (error - lastError) / dt : 0.0;

            lastError = error;

            return (Kp * error)
                    + (Ki * integral)
                    + (Kd * derivative);
        }
    }
}
