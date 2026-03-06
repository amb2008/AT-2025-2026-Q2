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
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.CONSTANTS.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.List;
//@Disabled

@Autonomous(name = "RED - Wall",group="Robot")
public class AutoRedWall extends LinearOpMode {
    GoBildaPinpointDriver odo;
    private Follower follower;
    private final Pose startPose = new Pose(87, 7, Math.toRadians(0));
    private final Pose scorePose = new Pose(87, 7, Math.toRadians(0));
    private final Pose scorePose2 = new Pose(91, 12, Math.toRadians(357));
    private final Pose pickup1Pose = new Pose(90, 30, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(127, 12, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(110, 10, Math.toRadians(0));
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
    private DcMotor intake2 = null;
    private Servo flick1 = null;
    private Servo flick2 = null;
    private Servo flick3 = null;
    private Servo grant = null;
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
    private boolean sweep = true;
    private boolean needPattern = true;
    private boolean outtaking = false;
    private boolean wackSet = false;
    private double count = 0;
    private boolean intakeDone = false;

    boolean shooting = false;
    //        AXON ENCODER TRACKING
    private AnalogInput axonEncoder;
    private static final double MAX_VOLTAGE = 3.3; // Check your specific hub, usually 3.3V
    private static final double GEAR_RATIO = 5.0;  // 5:1 Reduction
    private double lastVoltage = 0;
    private int rotationCount = 0;
    private double turretPower = 0.95;
    private double targetTagID = 24;
    private double lastDirection = -1; //move to the left at start
    double targetVelocity = 840;
    double lastError = 0;
    // Safety Limits (Degrees)
    private static final double MAX_TURRET_ANGLE = 170;
    private static final double MIN_TURRET_ANGLE = -170;

    //    FLYWHEEL
    PIDController pid = new PIDController(0.041, 0.0, 0.0);
    private IMU imu;
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
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose2))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose2.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose2))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose2.getHeading())
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
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        flick1 = hardwareMap.get(Servo.class, "flick1");
        flick2 = hardwareMap.get(Servo.class, "flick2");
        flick3 = hardwareMap.get(Servo.class, "flick3");
        grant = hardwareMap.get(Servo.class, "grant");
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

        //        FLYWHEEL
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        fL.setDirection(DcMotor.Direction.FORWARD);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bL.setDirection(DcMotor.Direction.FORWARD);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fR.setDirection(DcMotor.Direction.REVERSE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bR.setDirection(DcMotor.Direction.REVERSE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fwl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fwr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        grant.setPosition(0.02);
        flick1.setPosition(flicksDown[0]);
        flick2.setPosition(flicksDown[1]);
        flick3.setPosition(flicksDown[2]);
        new Thread(()->{
            sleep(3500);
            needPattern = false;
        }).start();
        new Thread(()->{
            while (opModeIsActive()){
                fwOn();
            }
        }).start();

        new Thread(()->{
            while (opModeIsActive()){
                if (sweep){
                    moveTurret();
                } else {
                    turret.setPower(0);
                }
            }
        }).start();

//         --------- STEP 1: SCORE PRELOAD ----------
        sleep(1500);
        outtake();
        sleep(500);
        outtake();
        sweep = false;
        // --------- STEP 4: GRAB PICKUP 2 ----------
        follower.followPath(grabPickup2, true);
        while (opModeIsActive() && follower.isBusy()) {
            sweep = false;
            follower.update();
            telemetry.addLine("Following path");
            telemetry.update();
        }
        telemetry.addLine("Path finished");
        telemetry.update();
        intakeMacroFar();
//
        // --------- STEP 5: SCORE PICKUP 2 ----------
        follower.followPath(scorePickup2, true);
        while (opModeIsActive() && follower.isBusy()) {
            sweep = false;
            reverseIntake();
            follower.update();
            telemetry.addLine("Following path");
            telemetry.update();
        }
        intake1.setPower(0);
        intake2.setPower(0);
        telemetry.addLine("Path finished");
        telemetry.update();
        sweep = true;
        outtake();
        sleep(500);
        outtake();
        sweep = false;
//
        // --------- STEP 6: GRAB PICKUP 3 ----------
        follower.followPath(grabPickup3, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addLine("Following path");
            telemetry.update();
        }
        telemetry.addLine("Path finished");
        telemetry.update();
        intakeMacroFar();
//
//        // --------- STEP 7: SCORE PICKUP 3 ----------
//        follower.followPath(scorePickup3, true);
//        while (opModeIsActive() && follower.isBusy()) {
//            follower.update();
//            telemetry.addLine("Following path");
//            telemetry.update();
//        }
//        telemetry.addLine("Path finished");
//        telemetry.update();
//        outtake();
    }

    private void outtake() {
        if (!outtaking) {
            if (limelight.getLatestResult() == null) {
                while (limelight.getLatestResult() == null) {

                }
                sleep(500);
            }
            outtaking = true;
            shooting = true;
            double counter = 0;
            for (String targetColor : pattern) {
                counter += 1;
                if (counter == 2){
                    targetVelocity = 820;
                } else {
                    targetVelocity = 840;
                }
                boolean launched = false;
                checkColor();
                for (int i = 0; i < slotColors.length; i++) {
                    if (slotColors[i].equalsIgnoreCase(targetColor)) {
                        if (i == 0) {
                            flick1.setPosition(flicksUp[0]);
                        } else if (i == 1) {
                            flick2.setPosition(flicksUp[1]);
                        } else if (i == 2) {
                            flick3.setPosition(flicksUp[2]);
                        }
                        launched = true;
                        break;
                    }
                }
                if (!launched) {
                    for (int i = 0; i < slotColors.length; i++) {
                        if (slotColors[i].equalsIgnoreCase("green") || slotColors[i].equalsIgnoreCase("purple")) {
                            if (i == 0) {
                                flick1.setPosition(flicksUp[0]);
                            } else if (i == 1) {
                                flick2.setPosition(flicksUp[1]);
                            } else if (i == 2) {
                                flick3.setPosition(flicksUp[2]);
                            }
                            launched = true;
                            break;
                        }
                    }
                }
                if (launched) {

                    sleep(300);
                    flick1.setPosition(flicksDown[0]);
                    flick2.setPosition(flicksDown[1]);
                    flick3.setPosition(flicksDown[2]);
                    for (int i = 0; i < slotColors.length; i++) {
                        if (slotColors[i].equalsIgnoreCase("green") || slotColors[i].equalsIgnoreCase("purple")) {
                            if (i == 0) {
                                flick1.setPosition(flicksUp[0]);
                            } else if (i == 1) {
                                flick2.setPosition(flicksUp[1]);
                            } else if (i == 2) {
                                flick3.setPosition(flicksUp[2]);
                            }
                            break;

                        }

                    }
                    flick1.setPosition(flicksDown[0]);
                    flick2.setPosition(flicksDown[1]);
                    flick3.setPosition(flicksDown[2]);
                    if (counter < 3) {
                        sleep(300);
                    }
                }
            }
            outtaking = false;
            shooting = false;
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
            double error = tx-2;
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
//        if (needPattern){
//            checkPattern();
//        }
    }

    private void intakeMacroClose(){
        intakeDone = false;
        new Thread(()->{
            while (!intakeDone){
                intake();
            }
            intake1.setPower(0);
            intake2.setPower(0);
        }).start();
        new Thread(()->{
            sleep(3500);
            intakeDone = true;
        }).start();
        driveRelativeX(43);
    }
    private void intakeMacroFar(){
        intakeDone = false;
        new Thread(()->{
            while (!intakeDone){
                intake();
            }
            intake1.setPower(0);
            intake2.setPower(0);
        }).start();
        new Thread(()->{
            sleep(2700);
            intakeDone = true;
        }).start();
        driveRelativeX(1);
        driveRelativeY(-5);
    }

    private void intake() {
        intake1.setDirection(DcMotor.Direction.FORWARD);
        intake1.setPower(intakeSpeed);
        intake2.setDirection(DcMotor.Direction.REVERSE);
        intake2.setPower(intakeSpeed);
    }

    private void reverseIntake() {
        intake1.setDirection(DcMotor.Direction.REVERSE);
        intake1.setPower(intakeSpeed);
        intake2.setDirection(DcMotor.Direction.FORWARD);
        intake2.setPower(intakeSpeed);
    }

    private void checkColor() {
        double tolerance = 0.07;
        purpleBall = normalizeColor(purpleBall);
        greenBall = normalizeColor(greenBall);
        slotColors[0] = "Empty";
        slotColors[1] = "Empty";
        slotColors[2] = "Empty";

        double[] rgb1a = normalizeColor(new double[]{cs1a.red(), cs1a.green(), cs1a.blue()});
        double[] rgb1b = normalizeColor(new double[]{cs1b.red(), cs1b.green(), cs1b.blue()});
        double[] rgb2a = normalizeColor(new double[]{cs2a.red(), cs2a.green(), cs2a.blue()});
        double[] rgb2b = normalizeColor(new double[]{cs2b.red(), cs2b.green(), cs2b.blue()});
        double[] rgb3a = normalizeColor(new double[]{cs3a.red(), cs3a.green(), cs3a.blue()});
        double[] rgb3b = normalizeColor(new double[]{cs3b.red(), cs3b.green(), cs3b.blue()});
//        telemetry.addData("Red", cs1a.red());
//        telemetry.addData("Green", cs1a.green());
//        telemetry.addData("Blue", cs1a.blue());

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

    public void driveRelativeX(double inches) {
        odo.update();
        double startX = odo.getPosition().getX(DistanceUnit.INCH);   // inches
        double targetX = startX + inches;
        boolean movingPositive = inches > 0;
        // Loop until we reach target or timeout
        while (opModeIsActive() && !intakeDone) {
            odo.update();
            double currentX = odo.getPosition().getX(DistanceUnit.INCH);
            double error = targetX - currentX;

            // Stop when close enough
            telemetry.addData("Current X", currentX);
            telemetry.addData("Target X", targetX);
            if (movingPositive) {
                if (currentX >= targetX) break;
            } else {
                if (currentX <= targetX) break;
            }
            double power = 0.28 * Math.signum(error);   // apply sign
            // Mecanum pure strafe
            fL.setPower(power);
            fR.setPower(power);
            bL.setPower(power);
            bR.setPower(power);
        }
    }

    public void driveRelativeY(double inches) {
        odo.update();
        double startY = odo.getPosition().getY(DistanceUnit.INCH);
        double targetY = startY + inches;

        // Determine direction so we can stop correctly
        boolean movingPositive = inches > 0;

        while (opModeIsActive() && !intakeDone) {
            odo.update();
            double currentY = odo.getPosition().getY(DistanceUnit.INCH);
            double error = targetY - currentY;

            telemetry.addData("Current Y", currentY);
            telemetry.addData("Target Y", targetY);
            telemetry.update();

            // Exit condition: check if we've reached or passed the target
            if (movingPositive) {
                if (currentY >= targetY) break;
            } else {
                if (currentY <= targetY) break;
            }

            double power = -0.4 * Math.signum(error); // Strafing usually requires more power to overcome friction

            // Mecanum Strafe Pattern
            // Left Front and Right Back go one way; Right Front and Left Back go the other.
            fL.setPower(power);
            fR.setPower(-power);
            bL.setPower(-power);
            bR.setPower(power);
        }
    }

    private void fwOn(){
        //limelight stuff
        double leftVelocity = fwl.getVelocity();
        double rightVelocity = fwr.getVelocity();
        telemetry.addData("Left velocity", leftVelocity);
        telemetry.addData("Right velocity", rightVelocity);
        double avgVelocity = (leftVelocity + rightVelocity) / 2.0;

        pid.setSetpoint(targetVelocity);
        double pidOutput = pid.update(avgVelocity);
        // Feedforward based on max motor velocity
        double feedforward = targetVelocity / MAX_VELOCITY;
        pidOutput += feedforward;
        pidOutput = Math.max(0, Math.min(pidOutput, 1));

        fwl.setPower(pidOutput);
        fwr.setPower(pidOutput);
    }

    private void fwOff(){
        fwl.setVelocity(0);
        fwr.setVelocity(0);
    }
}
