package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.CONSTANTS.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "BLUE - Close",group="Robot")
@Disabled
public class AutoBlueClose extends LinearOpMode {
    GoBildaPinpointDriver odo;
    private Follower follower;
    private final Pose startPose = new Pose(17, 124, Math.toRadians(330));
    private final Pose scorePose = new Pose(59, 94, Math.toRadians(185));
    private final Pose scorePose2 = new Pose(56, 94, Math.toRadians(185));
    private final Pose pickup1Pose = new Pose(49.5, 89, Math.toRadians(186));
    private final Pose pickup2Pose = new Pose(50, 69, Math.toRadians(185));
    private final Pose pickup3Pose = new Pose(49, 28, Math.toRadians(180));
    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    //    NON PEDRO
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fL = null;
    private DcMotor bL = null;
    private DcMotor fR = null;
    private DcMotor bR = null;
    private DcMotorEx fwl = null;
    private DcMotorEx fwr = null;
    private DcMotor intake1 = null;
    private Servo flick1 = null;
    private Servo flick2 = null;
    private Servo flick3 = null;
    private CRServo turret = null;
    private Limelight3A limelight;
    private ColorSensor cs1a;
    private ColorSensor cs1b;
    private ColorSensor cs2a;
    private ColorSensor cs2b;
    private ColorSensor cs3a;
    private ColorSensor cs3b;
    private int servoIndex = 0;  // start at fir
    // st position
    private String[] slotColors = {"Empty", "Empty", "Empty"};
    private String[] pattern = {"purple", "purple", "green"};
    private boolean sweepingForward = true;
    private boolean intakeReady = true;
    private boolean needPattern = true;
    private boolean outtaking = false;
    private boolean wackSet = false;
    private double count = 0;
    private boolean intakeDone = false;
    //        AXON ENCODER TRACKING
    private AnalogInput axonEncoder;
    private static final double MAX_VOLTAGE = 3.3; // Check your specific hub, usually 3.3V
    private static final double GEAR_RATIO = 5.0;  // 5:1 Reduction
    private double lastVoltage = 0;
    private int rotationCount = 0;
    private double turretPower = 0.95;
    private double targetTagID = 20;
    private double lastDirection = 1;
    double lastError = 0;
    // Safety Limits (Degrees)
    private static final double MAX_TURRET_ANGLE = 180;
    private static final double MIN_TURRET_ANGLE = -180;

    //    FLYWHEEL
    PIDController pid = new PIDController(0.0115, 0.0, 0.0);
    final double MAX_MOTOR_RPM = 6000;      // GoBILDA 6000 RPM
    final double TICKS_PER_REV = 28;        // Encoder CPR
    final double MAX_VELOCITY = (MAX_MOTOR_RPM / 60.0) * TICKS_PER_REV; // ticks/sec
    ElapsedTime timer = new ElapsedTime();
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose2))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose2.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose2))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose2.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(140, 0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        fL = hardwareMap.get(DcMotorEx.class, "fL");
        bL = hardwareMap.get(DcMotorEx.class, "bL");
        fR = hardwareMap.get(DcMotorEx.class, "fR");
        bR = hardwareMap.get(DcMotorEx.class, "bR");
        fwl = hardwareMap.get(DcMotorEx.class, "fwl");
        fwr = hardwareMap.get(DcMotorEx.class, "fwr");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        flick1 = hardwareMap.get(Servo.class, "flick1");
        flick2 = hardwareMap.get(Servo.class, "flick2");
        flick3 = hardwareMap.get(Servo.class, "flick3");
        turret = hardwareMap.get(CRServo.class, "turret");
        axonEncoder = hardwareMap.get(AnalogInput.class, "encoder");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        cs1a = hardwareMap.get(ColorSensor.class, "cs1a");
        cs1b = hardwareMap.get(ColorSensor.class, "cs1b");
        cs2a = hardwareMap.get(ColorSensor.class, "cs2a");
        cs2b = hardwareMap.get(ColorSensor.class, "cs2b");
        cs3a = hardwareMap.get(ColorSensor.class, "cs3a");
        cs3b = hardwareMap.get(ColorSensor.class, "cs3b");

        fL.setDirection(DcMotor.Direction.FORWARD);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bL.setDirection(DcMotor.Direction.FORWARD);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fR.setDirection(DcMotor.Direction.REVERSE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bR.setDirection(DcMotor.Direction.REVERSE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fwl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fwr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fwl.setDirection(DcMotor.Direction.REVERSE);
        fwr.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{fL, fR, bL, bR}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        follower = Constants.createFollower(hardwareMap);
        buildPaths();

        follower.setStartingPose(startPose);

        waitForStart();
        pid.setSetpoint(autoCloseFwSpeed);
        new Thread(()->{
            while (opModeIsActive()){
                fwOn();
            }
        }).start();

        while (opModeIsActive() && needPattern){
            checkPattern();
            new Thread(()->{
                sleep(500);
                needPattern = false;
            }).start();
        }

        new Thread(()->{
            while (opModeIsActive()){
                moveTurret();
            }
        }).start();

        // --------- STEP 1: SCORE PRELOAD ----------
        follower.followPath(scorePreload);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        outtake();
        // --------- STEP 2: GRAB PICKUP 1 ----------
        follower.followPath(grabPickup1, true);
        while (opModeIsActive() && follower.isBusy()) {
            fwOn();
            follower.update();

        }
        sleep(150);
        headingCorrect(pickup1Pose.getHeading());
        intakeMacro();
        // --------- STEP 3: SCORE PICKUP 1 ----------
        follower.followPath(scorePickup1, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        headingCorrect2(scorePose2.getHeading());
        sleep(100);
        outtake();
        // --------- STEP 4: GRAB PICKUP 2 ----------
        follower.followPath(grabPickup2, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        headingCorrect(pickup1Pose.getHeading());
        intakeMacro();

        // --------- STEP 5: SCORE PICKUP 2 ----------
//        follower.followPath(scorePickup2, true);
//        while (opModeIsActive() && follower.isBusy()) {
//            follower.update();
//        }
//        headingCorrect(scorePose2.getHeading());
//        outtake();
//
//
//        // --------- STEP 6: GRAB PICKUP 3 ----------
//        follower.followPath(grabPickup3, true);
//        while (opModeIsActive() && follower.isBusy()) {
//            follower.update();
//        }
//        intakeMacro();
//
//        // --------- STEP 7: SCORE PICKUP 3 ----------
//        follower.followPath(scorePickup3, true);
//        while (opModeIsActive() && follower.isBusy()) {
//            follower.update();
//        }
//        outtake();
    }

    private void outtake() {
        if (!outtaking) {
            outtaking = true;
            checkColor();
            List<Integer> servoSequence = new ArrayList<>();
            boolean[] used = new boolean[slotColors.length];
            for (String targetColor : pattern) {
                for (int i = 0; i < slotColors.length; i++) {
                    if (!used[i] && slotColors[i].equalsIgnoreCase(targetColor)) {
                        servoSequence.add(i);  // add servo position corresponding to that slot
                        used[i] = true;                // mark slot as used
                        break;                         // move on to next pattern color
                    }
                }
            }
            for (int i = 0; i < slotColors.length; i++) {
                if (!used[i]) {
                    servoSequence.add(i);
                    used[i] = true;
                }
            }

            for (int i = 0; i < servoSequence.size(); i++) {
                if (outtaking) {
                    if (servoSequence.get(i)==0){
                        flick1.setPosition(flicksUp[0]);
                    } else if (servoSequence.get(i)==1){
                        flick2.setPosition(flicksUp[1]);
                    } else if (servoSequence.get(i)==2){
                        flick3.setPosition(flicksUp[2]);
                    }

                    sleep(500);
                    flick1.setPosition(flicksDown[0]);
                    flick2.setPosition(flicksDown[1]);
                    flick3.setPosition(flicksDown[2]);
                    sleep(500);
                }
            }
            slotColors[0] = "Empty";
            slotColors[1] = "Empty";
            slotColors[2] = "Empty";
            outtaking = false;
        }
    }

    private void checkPattern() {
        limelight.pipelineSwitch(0);
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            int tagId = 0;
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                tagId = fr.getFiducialId();
            }
            if (tagId == 21) {
                pattern[0] = "green";
                pattern[1] = "purple";
                pattern[2] = "purple";
                needPattern = false;
            } else if (tagId == 22) {
                pattern[0] = "purple";
                pattern[1] = "green";
                pattern[2] = "purple";
                needPattern = false;
            } else if (tagId == 23) {
                pattern[0] = "purple";
                pattern[1] = "purple";
                pattern[2] = "green";
                needPattern = false;
            }
            telemetry.addData("AprilTag ID", tagId);
            telemetry.addData("Pattern 0", pattern[0]);
            telemetry.addData("Pattern 1", pattern[1]);
            telemetry.addData("Pattern 2", pattern[2]);
        } else {
            telemetry.addLine("No valid AprilTag detected");
        }
    }

    private void moveTurret() {
        double currentVoltage = axonEncoder.getVoltage();
        double currentServoAngle = (currentVoltage / MAX_VOLTAGE) * 360.0;
        double threshold = MAX_VOLTAGE / 2.0;
        if (currentVoltage - lastVoltage < -threshold) {
            rotationCount++;
        } else if (currentVoltage - lastVoltage > threshold) {
            rotationCount--;
        }
        lastVoltage = currentVoltage;
        double totalServoDegrees = (rotationCount * 360.0) + currentServoAngle;
        double turretAngle = totalServoDegrees / GEAR_RATIO;

        limelight.pipelineSwitch(1);
        double outputPower = 0;
        double tx = 0;
        boolean locked = false;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if (tag.getFiducialId() == targetTagID) {
                    tx = tag.getTargetXDegrees();
                    lastDirection = Math.signum(tx)*1;
                    locked = true;
                    break;
                }
            }
        }
        if (Math.abs(turretAngle) > MAX_TURRET_ANGLE || !locked){
            if (turretAngle >= MAX_TURRET_ANGLE) {
                lastDirection = -1;
            } else if (turretAngle <= MIN_TURRET_ANGLE) {
                lastDirection = 1;
            }
            turretPower = 0.99*lastDirection;
            turret.setPower(turretPower);
        }
        else if (locked) {
            double error = tx;
            double dt = timer.seconds();
            if (dt == 0) dt = 0.001; // Safety

            double errorRate = (error - lastError) / dt;
            errorRate = Range.clip(errorRate, -100, 100);

            double pTerm = turretkP * error;
            double dTerm = turretkD * errorRate;
            double fTerm = 0;
            if (Math.abs(error) > 3.0) {  // degrees
                fTerm = Math.signum(error) * turretkF;
            }
            outputPower = pTerm + dTerm + fTerm;
            lastError = error;
            timer.reset();
            if (Math.abs(error) > 1) {
                turret.setPower(Range.clip(outputPower, -0.75, 0.75));
            } else {
                turret.setPower(0);
            }
        }
    }

    private void intakeMacro(){
        intakeDone = false;
        new Thread(()->{
            while (!intakeDone){
                intake();
            }
        }).start();
        driveRelativeX(-20);
        new Thread(()->{
            sleep(1000);
            intakeDone = true;
        }).start();
    }

    private void intake() {
        intake1.setDirection(DcMotor.Direction.REVERSE);
        intake1.setPower(intakeSpeed);
    }

    private void checkColor() {
        double tolerance = 0.06;
        purpleBall = normalizeColor(purpleBall);
        greenBall = normalizeColor(greenBall);

        double[] rgb1a = normalizeColor(new double[]{cs1a.red(), cs1a.green(), cs1a.blue()});
        double[] rgb1b = normalizeColor(new double[]{cs1b.red(), cs1b.green(), cs1b.blue()});
        double[] rgb2a = normalizeColor(new double[]{cs2a.red(), cs2a.green(), cs2a.blue()});
        double[] rgb2b = normalizeColor(new double[]{cs2b.red(), cs2b.green(), cs2b.blue()});
        double[] rgb3a = normalizeColor(new double[]{cs3a.red(), cs3a.green(), cs3a.blue()});
        double[] rgb3b = normalizeColor(new double[]{cs3b.red(), cs3b.green(), cs3b.blue()});

        // Check Slot 1
        double pdist = colorDistance(rgb1a, purpleBall);
        double gdist = colorDistance(rgb1a, greenBall);
        double pdist2 = colorDistance(rgb1b, purpleBall);
        double gdist2 = colorDistance(rgb1b, greenBall);
        if ((pdist < gdist && pdist < tolerance) || (pdist2 < gdist2 && pdist2 < tolerance))
            slotColors[0] = "Purple";
        else if (gdist < pdist && gdist < tolerance || (gdist2 < pdist2 && gdist2 < tolerance))
            slotColors[0] = "Green";

        // Check Slot 2
        pdist = colorDistance(rgb2a, purpleBall);
        gdist = colorDistance(rgb2a, greenBall);
        pdist2 = colorDistance(rgb2b, purpleBall);
        gdist2 = colorDistance(rgb2b, greenBall);
        if ((pdist < gdist && pdist < tolerance) || (pdist2 < gdist2 && pdist2 < tolerance))
            slotColors[1] = "Purple";
        else if (gdist < pdist && gdist < tolerance || (gdist2 < pdist2 && gdist2 < tolerance))
            slotColors[1] = "Green";

        // Check Slot 3
        pdist = colorDistance(rgb3a, purpleBall);
        gdist = colorDistance(rgb3a, greenBall);
        pdist2 = colorDistance(rgb3b, purpleBall);
        gdist2 = colorDistance(rgb3b, greenBall);
        if ((pdist < gdist && pdist < tolerance) || (pdist2 < gdist2 && pdist2 < tolerance))
            slotColors[2] = "Purple";
        else if (gdist < pdist && gdist < tolerance || (gdist2 < pdist2 && gdist2 < tolerance))
            slotColors[2] = "Green";
    }

    private double[] normalizeColor(double[] rgb) {
        double sum = rgb[0] + rgb[1] + rgb[2];
        return new double[]{rgb[0] / sum, rgb[1] / sum, rgb[2] / sum};
    }

    private double colorDistance(double[] c1, double[] c2) {
        return Math.sqrt(Math.pow(c1[0] - c2[0], 2) + Math.pow(c1[1] - c2[1], 2) + Math.pow(c1[2] - c2[2], 2));
    }

    // Robust heading correction (drop into your class). targetHeadingRad is radians.
    private void headingCorrect(double targetHeadingRad) {
        final double kP = 1.3;
        final double yawTolRad = Math.toRadians(1.0);  // stop within ~1 degree
        final double maxPower = 0.35;    // max wheel power used for rotation (tune)
        final long timeoutMs = 5000;     // safety timeout in ms
        long start = System.currentTimeMillis();

        while (opModeIsActive()) {
            if (System.currentTimeMillis() - start > timeoutMs) {
                telemetry.addData("headingCorrect", "timeout");
                break;
            }

            // update odometry
            odo.update();
            Pose2D pos = odo.getPosition();
            double currYaw = pos.getHeading(AngleUnit.RADIANS);

            // shortest angular error: target - current normalized to [-pi, pi]
            double yawErr = Math.atan2(Math.sin(targetHeadingRad - currYaw),
                    Math.cos(targetHeadingRad - currYaw));

            // stop condition
            if (Math.abs(yawErr) <= yawTolRad) {
                break;
            }

            // P controller -> wheel power (sign convention: positive yawErr => rotate positive)
            double power = -kP * yawErr;
            // clip and ensure minimum deadband
            power = Math.max(-maxPower, Math.min(maxPower, power));

            // If power is very small, push it to a minimum to overcome static friction
            double minPower = 0.1;
            if (Math.abs(power) > 0 && Math.abs(power) < minPower) {
                power = Math.copySign(minPower, power);
            }

            telemetry.addData("target", Math.toDegrees(targetHeadingRad));
            telemetry.addData("yaw", Math.toDegrees(currYaw));
            telemetry.addData("error", Math.toDegrees(yawErr));
            telemetry.update();

            fL.setPower(power);
            fR.setPower(-power);
            bL.setPower(power);
            bR.setPower(-power);

            sleep(12);
        }

        // stop motors and clear flag
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
        telemetry.addLine("headingCorrect done");
        telemetry.addLine("AT is the best! - Jonah");
        telemetry.update();
    }

    private void headingCorrect2(double targetHeadingRad) {
        final double kP = 1.3;
        final double yawTolRad = Math.toRadians(1.0);  // stop within ~1 degree
        final double maxPower = 0.2;    // max wheel power used for rotation (tune)
        final long timeoutMs = 5000;     // safety timeout in ms
        long start = System.currentTimeMillis();

        while (opModeIsActive()) {
            if (System.currentTimeMillis() - start > timeoutMs) {
                telemetry.addData("headingCorrect", "timeout");
                break;
            }

            // update odometry
            odo.update();
            Pose2D pos = odo.getPosition();
            double currYaw = pos.getHeading(AngleUnit.RADIANS);

            // shortest angular error: target - current normalized to [-pi, pi]
            double yawErr = Math.atan2(Math.sin(targetHeadingRad - currYaw),
                    Math.cos(targetHeadingRad - currYaw));

            // stop condition
            if (Math.abs(yawErr) <= yawTolRad) {
                break;
            }

            // P controller -> wheel power (sign convention: positive yawErr => rotate positive)
            double power = -kP * yawErr;
            // clip and ensure minimum deadband
            power = Math.max(-maxPower, Math.min(maxPower, power));

            // If power is very small, push it to a minimum to overcome static friction
            double minPower = 0.1;
            if (Math.abs(power) > 0 && Math.abs(power) < minPower) {
                power = Math.copySign(minPower, power);
            }

            telemetry.addData("target", Math.toDegrees(targetHeadingRad));
            telemetry.addData("yaw", Math.toDegrees(currYaw));
            telemetry.addData("error", Math.toDegrees(yawErr));
            telemetry.update();

            fL.setPower(power);
            fR.setPower(-power);
            bL.setPower(power);
            bR.setPower(-power);

            sleep(12);
        }

        // stop motors and clear flag
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
        telemetry.addLine("headingCorrect done");
        telemetry.addLine("AT is the best! - Jonah");
        telemetry.update();
    }

    public void driveRelativeX(double inches) {
        // Update odometry to get starting pose
        odo.update();
        double startX = odo.getPosition().getX(DistanceUnit.INCH);   // inches
        double targetX = startX + inches;
        // Loop until we reach target or timeout
        while (opModeIsActive()) {
            odo.update();
            double currentX = odo.getPosition().getX(DistanceUnit.INCH);
            double error = targetX - currentX;

            // Stop when close enough
            if (Math.abs(error) < 0.2) {
                break;
            }

            // Scale power as you approach the target (smooth stop)
            double power = -0.1*Math.signum(error);   // apply sign
            // Mecanum pure strafe
            fL.setPower(power);
            fR.setPower(power);
            bL.setPower(power);
            bR.setPower(power);
        }

        // Stop the robot
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }




    private void fwOn(){
        double leftVelocity  = fwl.getVelocity();
        double rightVelocity = fwr.getVelocity();
        double avgVelocity = (leftVelocity + rightVelocity) / 2.0;

        double pidOutput = pid.update(avgVelocity);
        // Feedforward based on max motor velocity
        double feedforward = autoCloseFwSpeed / MAX_VELOCITY;
        pidOutput += feedforward;

        // Clip to [0,1]
        pidOutput = Math.max(0, Math.min(pidOutput, 1));
        fwl.setPower(pidOutput);
        fwr.setPower(pidOutput);
//        odo.update();
//        Pose2D pos = odo.getPosition();
//
//        telemetry.addData("slot 0", slotColors[0]);
//        telemetry.addData("slot 1", slotColors[1]);
//        telemetry.addData("slot 2", slotColors[2]);
//        telemetry.update();
    }

    private void fwOff(){
        fwl.setVelocity(0);
        fwr.setVelocity(0);
    }
}
