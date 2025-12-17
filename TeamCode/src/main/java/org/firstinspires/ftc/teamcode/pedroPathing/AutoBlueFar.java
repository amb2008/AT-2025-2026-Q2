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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.CONSTANTS.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "BLUE - Far",group="Robot")
public class AutoBlueFar extends LinearOpMode {
    GoBildaPinpointDriver odo;
    private Follower follower;
    private final Pose startPose = new Pose(57, 9, Math.toRadians(90));
    private final Pose scorePose = new Pose(56, 14, Math.toRadians(20));
    private final Pose scorePose2 = new Pose(56, 22, Math.toRadians(20));
    private final Pose pickup1Pose = new Pose(46, 32, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(44, 56, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(46, 84, Math.toRadians(180));
    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    //    NON PEDRO
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx fL = null;
    private DcMotorEx bL = null;
    private DcMotorEx fR = null;
    private DcMotorEx bR = null;
    private DcMotorEx fwl = null;
    private DcMotorEx fwr = null;
    private DcMotor intake1 = null;
    private Servo sorting1 = null;
    private Servo sorting2 = null;
    private Servo limelightmount = null;
    private Limelight3A limelight;
    private double llServoPos = 0.3;
    private ColorSensor colorSensor;
    private int servoIndex = 0;  // start at first position
    private String[] slotColors = {"Purple", "Purple", "Green"};
    private String[] pattern = {"purple", "purple", "green"};
    private String lastBallColor = "Unknown";
    private boolean sweepingForward = true;
    private double heading = 0.0;
    private double axial = 0.0;
    private double lateral = 0.0;
    private double yaw = 0.0;
    private boolean intakeReady = true;
    private boolean firstIntake = false;
    private boolean needPattern = true;
    private boolean intakeDone = false;
    private double lastPos = suzani[servoIndex];
    private double count = 0;

    //    FLYWHEEL
    PIDController pid = new PIDController(0.0115, 0.0, 0.0);
    final double MAX_MOTOR_RPM = 6000;      // GoBILDA 6000 RPM
    final double TICKS_PER_REV = 28;        // Encoder CPR
    final double MAX_VELOCITY = (MAX_MOTOR_RPM / 60.0) * TICKS_PER_REV; // ticks/sec
    final double headingConstraint = Math.toRadians(0.5);
    ElapsedTime llTimer = new ElapsedTime();
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        scorePreload.setHeadingConstraint(headingConstraint);

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose2))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .setHeadingConstraint(headingConstraint)
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose2))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
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
        fL = hardwareMap.get(DcMotorEx.class, "fL");
        bL = hardwareMap.get(DcMotorEx.class, "bL");
        fR = hardwareMap.get(DcMotorEx.class, "fR");
        bR = hardwareMap.get(DcMotorEx.class, "bR");
        fwl = hardwareMap.get(DcMotorEx.class, "fwl");
        fwr = hardwareMap.get(DcMotorEx.class, "fwr");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        sorting1 = hardwareMap.get(Servo.class, "sorting1");
        sorting2 = hardwareMap.get(Servo.class, "sorting2");
        limelightmount = hardwareMap.get(Servo.class, "limelightmount");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-175.0, 60, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        fL.setDirection(DcMotor.Direction.FORWARD);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bL.setDirection(DcMotor.Direction.FORWARD);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fR.setDirection(DcMotor.Direction.REVERSE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bR.setDirection(DcMotor.Direction.REVERSE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fwl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fwr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fwl.setDirection(DcMotor.Direction.FORWARD);
        fwr.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                fL.getCurrentPosition(),
                bR.getCurrentPosition());
        telemetry.update();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();

        follower.setStartingPose(startPose);

        waitForStart();
        pid.setSetpoint(autoFwSpeed2);
        new Thread(()->{
            while (opModeIsActive()){
                fwOn();
            }
        }).start();

        while (opModeIsActive() && needPattern){
            checkPattern();
            susanNext();
            new Thread(()->{
                sleep(500);
                needPattern = false;
            }).start();
        }

        // --------- STEP 1: SCORE PRELOAD ----------
        follower.followPath(scorePreload);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        headingCorrect(scorePose.getHeading());
        susanNext();
        sleep(1500);
        outtake();
        count += 1;
        // --------- STEP 2: GRAB PICKUP 1 ----------
        follower.followPath(grabPickup1, true);
        while (opModeIsActive() && follower.isBusy()) {
            fwOn();
            follower.update();
        }
        headingCorrect(pickup1Pose.getHeading());
        intakeMacro();
        // --------- STEP 3: SCORE PICKUP 1 ----------
        follower.followPath(scorePickup1, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        headingCorrect(scorePose2.getHeading());
        sleep(200);
        outtake();
        count += 1;

        // --------- STEP 4: GRAB PICKUP 2 ----------
        follower.followPath(grabPickup2, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        headingCorrect(pickup1Pose.getHeading());
        intakeMacro();

        // --------- STEP 5: SCORE PICKUP 2 ----------
        follower.followPath(scorePickup2, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        headingCorrect(scorePose2.getHeading());
        outtake();


        // --------- STEP 6: GRAB PICKUP 3 ----------
        follower.followPath(grabPickup3, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        intakeMacro();

        // --------- STEP 7: SCORE PICKUP 3 ----------
        follower.followPath(scorePickup3, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        outtake();
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

    private void outtake() {
        sorting2.setPosition(wackDown);
        List<Double> servoSequence = sequence();
        // --- Perform outtake motion ---
        for (double servoPos : servoSequence) {

            telemetry.addData("slot 0", slotColors[0]);
            telemetry.addData("slot 1", slotColors[1]);
            telemetry.addData("slot 2", slotColors[2]);
            telemetry.update();

            sorting1.setPosition(servoPos);

            if (opModeIsActive()) {
                if (Math.abs(lastPos - servoPos) > 0.4) {
                    sleep(1900);
                } else if (lastPos != servoPos) {
                    sleep(800);
                }

                sorting2.setPosition(wackUp);
                sleep(400);
                sorting2.setPosition(wackDown);
                sleep(200);

                lastPos = servoPos;
            }
        }

        // Reset
        servoIndex = 0;
        slotColors[0] = "Empty";
        slotColors[1] = "Empty";
        slotColors[2] = "Empty";
    }

    private void generatePermutations(List<Integer> arr, int k, List<List<Integer>> out) {
        if (k == arr.size()) {
            out.add(new ArrayList<>(arr));
            return;
        }
        for (int i = k; i < arr.size(); i++) {
            int temp = arr.get(k);
            arr.set(k, arr.get(i));
            arr.set(i, temp);

            generatePermutations(arr, k + 1, out);

            temp = arr.get(k);
            arr.set(k, arr.get(i));
            arr.set(i, temp);
        }
    }

    private List<Double> sequence(){
        List<Integer> slotIndices = new ArrayList<>();
        for (int i = 0; i < slotColors.length; i++) {
//            if (!slotColors[i].equalsIgnoreCase("Empty"))
            slotIndices.add(i);
        }

        // Convert pattern into list for assignment
        List<String> patternList = new ArrayList<>();
        for (String p : pattern) patternList.add(p);

        // --- Generate all permutations of slot indices ---
        List<List<Integer>> permutations = new ArrayList<>();
        generatePermutations(slotIndices, 0, permutations);

        double bestCost = Double.MAX_VALUE;
        List<Integer> bestPerm = slotIndices;  // fallback if nothing matches well

        // --- Evaluate each permutation ---
        for (List<Integer> perm : permutations) {
            double cost = 0;
            double currentPos = lastPos;
            boolean valid = true;

            for (int i = 0; i < patternList.size(); i++) {
                String targetColor = patternList.get(i);
                int slotIndex = 0;
                if (perm.size() > 0){
                    slotIndex = perm.get(i % perm.size());  // safe wrap if mismatch
                } else {
                    break;
                }

                if (!slotColors[slotIndex].equalsIgnoreCase(targetColor)) {
                    valid = false;
                    break;
                }

                double nextPos = suzano[slotIndex];
                if (i>0){
                    cost += Math.abs(nextPos - currentPos);
                }
                currentPos = nextPos;
            }

            if (valid && cost < bestCost) {
                bestCost = cost;
                bestPerm = new ArrayList<>(perm);
            }
        }

        // --- Build servo sequence from winning permutation ---
        List<Double> servoSequence = new ArrayList<>();
        if (bestCost>5){
            boolean[] used = new boolean[slotColors.length];
            for (String targetColor : pattern) {
                for (int i = 0; i < slotColors.length; i++) {
                    if (!used[i] && slotColors[i].equalsIgnoreCase(targetColor)) {
                        servoSequence.add(suzano[i]);  // add servo position corresponding to that slot
                        used[i] = true;                // mark slot as used
                        break;                         // move on to next pattern color
                    }
                }
            }
            for (int i = 0; i < slotColors.length; i++) {
                if (!used[i]) {
                    servoSequence.add(suzano[i]);
                    used[i] = true;
                }
            }
        } else {
            for (int idx : bestPerm) {
                servoSequence.add(suzano[idx]);
            }
        }

        if (count > 2){
            servoSequence.removeIf(pos -> pos == suzano[2]);
        }

        return servoSequence;
    }

    private void susanNext(){
        List<Double> servoSequence = sequence();
        double servoPos = servoSequence.get(0);
        sorting1.setPosition(servoPos);
        lastPos = servoPos;
    }

    private void intakeMacro(){
        lateral=0;
        yaw=0;
        servoIndex = 0;
        intakeDone = false;
        new Thread(()->{
            while (!intakeDone){
                intake();
                telemetry.addLine("INTAKING");
            }
            if (count == 1){
                slotColors[0] = "Green";
                slotColors[1] = "Purple";
                slotColors[2] = "Purple";
            } else if (count == 2){
                slotColors[0] = "Purple";
                slotColors[1] = "Green";
                slotColors[2] = "Purple";
            }
        }).start();

        driveRelativeX(-3);
        if (count < 2){
            sleep(100);
            driveRelativeX(-1);
            sleep(200);
            driveRelativeX(-18);
            sleep(200);
        } else {
            driveRelativeX(-8);
            sleep(100);
        }
        new Thread(()->{
            sleep(500);
            susanNext();
        }).start();
        new Thread(()->{
            sleep(1000);
            intakeDone = true;
        }).start();
    }

    private void intake() {
        String ballColor = checkColor();  // Get current detected color
        sorting1.setPosition(suzani[servoIndex]);
        lastPos = suzani[servoIndex];
        telemetry.addData("Intake ready", intakeReady);

        intake1.setDirection(DcMotor.Direction.REVERSE);
        if (ballColor.equals("Unknown")) {
            intake1.setPower(intakeSpeed);
            lastBallColor = "Unknown";
        } else if (intakeReady) {
            intake1.setPower(0);
            intakeReady = false;
//            ballColor = checkColor();
            lastBallColor = ballColor;
            // Store color in current slot
            slotColors[servoIndex] = ballColor;

            new Thread(() -> {
                sleep(10);
                if (servoIndex < 2) {
                    servoIndex++;
                    sorting1.setPosition(suzani[servoIndex]);
                } else {
                    intakeDone = true;
                }
                lastPos = suzani[servoIndex];
                sleep(1000);
                intakeReady = true;
            }).start();
        }
    }

    private String checkColor() {
        double red = colorSensor.red();
        double green = colorSensor.green();
        double blue = colorSensor.blue();
        purpleBall = normalizeColor(purpleBall);
        greenBall = normalizeColor(greenBall);

        telemetry.addData("Red", red);
        telemetry.addData("Green", green);
        telemetry.addData("Blue", blue);

        // Normalize input
        double sum = red + green + blue;
        if (sum > 0) {
            red /= sum;
            green /= sum;
            blue /= sum;
        }
        // Compare with *normalized* reference colors
        double purpleDistance = colorDistance(new double[]{red, green, blue}, purpleBall);
        double greenDistance = colorDistance(new double[]{red, green, blue}, greenBall);

        String detected = "Unknown";
        double tolerance = 0.06;  // tuned for normalized distances

        if (purpleDistance < greenDistance && purpleDistance < tolerance)
            detected = "Purple";
        else if (greenDistance < purpleDistance && greenDistance < tolerance)
            detected = "Green";

        telemetry.addData("Detected", detected);
        return detected;
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
        final double yawTolRad = Math.toRadians(2.0);  // stop within ~1 degree
        final double maxPower = 0.4;    // max wheel power used for rotation (tune)
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
            double power = -0.12*Math.signum(error);   // apply sign
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
        double feedforward = autoFwSpeed2 / MAX_VELOCITY;
        pidOutput += feedforward;

        // Clip to [0,1]
        pidOutput = Math.max(0, Math.min(pidOutput, 1));
        fwl.setPower(pidOutput);
        fwr.setPower(pidOutput);
        odo.update();
        Pose2D pos = odo.getPosition();
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
