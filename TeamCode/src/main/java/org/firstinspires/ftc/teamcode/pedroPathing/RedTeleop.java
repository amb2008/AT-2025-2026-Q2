/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static org.firstinspires.ftc.teamcode.CONSTANTS.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Red Tele-Op", group="Linear OpMode")
public class RedTeleop extends LinearOpMode {
    GoBildaPinpointDriver odo;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fL = null;
    private DcMotor bL = null;
    private DcMotor fR = null;
    private DcMotor bR = null;
    private DcMotorEx fwl = null;
    private DcMotorEx fwr = null;
    private DcMotor intake1 = null;
    private Servo sorting1 = null;
    private Servo sorting2 = null;
    private Servo limelightmount = null;
    private Limelight3A limelight;
    private double llServoPos = 0.3;
    double filtX = 0, filtY = 0, filtYaw = 0;
    boolean firstFrame = true;
    private ColorSensor colorSensor;
    private int servoIndex = 0;  // start at first position
    private String[] slotColors = {"Empty", "Empty", "Empty"};
    private String[] pattern = {"purple", "purple", "green"};
    private String lastBallColor = "Unknown";
    private boolean sweepingForward = true;
    private boolean intakeReady = true;
    private boolean needPattern = true;
    private boolean outtaking = false;
    private boolean wackSet = false;
    private boolean driverLock = false;

    //    manual booleans
    private boolean aWasPressed = false;
    private boolean yWasPressed = false;
    private boolean xWasPressed = false;
    private double lastPos = suzani[servoIndex];;
    ElapsedTime llTimer = new ElapsedTime();

    @Override//
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-175.0, 60, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

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
        limelight.start();
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

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

        fwl.setDirection(DcMotor.Direction.FORWARD);
        fwr.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{fL, fR, bL, bR}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        fwl.setVelocity(0);
        fwr.setVelocity(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            fwl.setVelocity(fwSpeed);
            fwr.setVelocity(fwSpeed);
            if (!wackSet){
                wackSet = true;
                sorting1.setPosition(suzano[0]);
                lastPos = suzano[0];
                sorting2.setPosition(wackDown);
            }
            if (needPattern) {
                limelight.pipelineSwitch(0);
                checkPattern();
                driveMecanum();
                limelightmount.setPosition(SERVO_CENTER_POS);
            } else {
                limelight.pipelineSwitch(1);
                LLResult result = limelight.getLatestResult();
                adjustLl(result);
                logBotPose(result);
                if (gamepad1.a && result != null && result.isValid()) {
                    driveToOrigin(redX, redY, redYaw);
                } else if (gamepad1.b){
                    driveToOrigin(redInX, redInY, redInYaw);
                } else {
                    driveMecanum();
                }
                printCurrentRobotPose();
            }

            if (gamepad1.left_trigger > 0.1){
                slotColors[0] = "Empty";
                slotColors[1] = "Empty";
                slotColors[2] = "Empty";
            }

            if (gamepad1.y){
                odo.resetPosAndIMU();
            }
            if (gamepad2.x && !xWasPressed) {
                xWasPressed = true;
                new Thread(()->{
                    outtake(pattern);
                }).start();
            }
            if (!gamepad2.x) xWasPressed = false;
            if (gamepad2.y) {
                new Thread(()->{
                    String[] outPattern = {"purple", "purple", "green"};
                    outtake(outPattern);
                }).start();
            }
            if (gamepad2.b) {
                new Thread(()->{
                    String[] outPattern = {"purple", "green", "purple"};
                    outtake(outPattern);
                }).start();
            }
            if (gamepad2.a) {
                new Thread(()->{
                    String[] outPattern = {"green", "purple", "purple"};
                    outtake(outPattern);
                }).start();
            }


//            GAMEPAD 2
            if (gamepad2.left_trigger > 0.1) {
                intake();
            } else if (gamepad2.right_trigger > 0.1) {
                intake1.setDirection(DcMotor.Direction.FORWARD);
                intake1.setPower(intakeSpeed);
            }
            else {
                intake1.setPower(0);
            }
            if (gamepad2.left_bumper) {
                servoIndex = 0;
                sorting1.setPosition(suzani[servoIndex]);
                lastPos = suzani[servoIndex];
            }
            if (gamepad2.right_bumper && !aWasPressed) {
                slotColors[0] = "Purple";
                slotColors[1] = "Purple";
                slotColors[2] = "Green";
                aWasPressed = true;
                servoIndex++;
                if (servoIndex > 2) servoIndex = 0;
                sorting1.setPosition(suzani[servoIndex]);
                lastPos = suzani[servoIndex];
            }
            if (!gamepad2.right_bumper) aWasPressed = false;

            if (gamepad2.dpad_right && !yWasPressed) {
                yWasPressed = true;
                servoIndex++;
                if (servoIndex > 2) servoIndex = 0;
                sorting1.setPosition(suzano[servoIndex]);
                lastPos = suzano[servoIndex];
            }
            if (!gamepad2.dpad_right) yWasPressed = false;

            if (gamepad2.dpad_left){
                outtaking = false;
            }
// --- Wack Up/Down ---
            if (gamepad2.dpad_up) {
                sorting2.setPosition(wackUp);
                new Thread(()->{
                    sleep(400);
                    sorting2.setPosition(wackDown);
                }).start();
            } else if (gamepad2.dpad_down){
                sorting2.setPosition(wackDown);
            }

            odo.update();
            Pose2D pos = odo.getPosition();

            double currX = pos.getX(DistanceUnit.INCH);
            double currY = pos.getY(DistanceUnit.INCH);
            telemetry.addData("Slot 1", slotColors[0]);
            telemetry.addData("Slot 2", slotColors[1]);
            telemetry.addData("Slot 3", slotColors[2]);
            telemetry.addData("Servo Index", servoIndex);
            telemetry.addData("Flywheel Left", "%.2f ticks/sec", fwl.getVelocity());
            telemetry.addData("Flywheel Right", "%.2f ticks/sec", fwr.getVelocity());
            telemetry.addData("Pattern 0", pattern[0]);
            telemetry.addData("Pattern 1", pattern[1]);
            telemetry.addData("Pattern 2", pattern[2]);
            telemetry.addData("currx", currX);
            telemetry.addData("curry", currY);
            telemetry.update();
        }
    }
    private void driveMecanum() {
        double axial = gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double yaw = -gamepad1.right_stick_x;

        if (gamepad1.dpad_up) {
            axial = -dpadSpeed;
        }
        if (gamepad1.dpad_down) {
            axial = dpadSpeed;
        }
        if (gamepad1.dpad_right) {
            lateral = -dpadSpeed;
        }
        if (gamepad1.dpad_left) {
            lateral = dpadSpeed;
        }

        if (gamepad1.left_bumper){
            yaw = 0.3;
        }
        if (gamepad1.right_bumper){
            yaw = -0.3;
        }

//        DRIVER LOCK
        if (driverLock) {
            odo.update();
            double robotYaw = odo.getPosition().getHeading(AngleUnit.RADIANS);

            double originalAxial = axial;     // forward/back
            double originalLateral = lateral; // left/right

            axial   =  originalAxial * Math.cos(robotYaw) - originalLateral * Math.sin(robotYaw);
            lateral =  originalAxial * Math.sin(robotYaw) + originalLateral * Math.cos(robotYaw);
        }

        double frontLeftPower = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower = axial - lateral + yaw;
        double backRightPower = axial + lateral - yaw;

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));
        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        fL.setPower(frontLeftPower);
        fR.setPower(frontRightPower);
        bL.setPower(backLeftPower);
        bR.setPower(backRightPower);
    }

    private void adjustLl(LLResult result) {
        limelightmount.setPosition(llServoPos);
        if (llTimer.milliseconds() < 10) return;
        llTimer.reset();

        if (result != null && result.isValid()) {
            double tx = result.getTx(); // horizontal offset (deg)
            double tolerance = 2;
            if (Math.abs(tx) < tolerance){
                double kP = 0.0012;          // proportional gain (tune this)
                double step = kP * tx;
                llServoPos += step;
                llServoPos = Range.clip(llServoPos, llServoMin, llServoMax);
                limelightmount.setPosition(llServoPos);
                sleep(5);
                telemetry.addData("tx", tx);
            }
        } else {
            if (sweepingForward) {
                llServoPos += searchSpeed;
                if (llServoPos >= llServoMax) {
                    llServoPos = llServoMax;
                    sweepingForward = false;
                }
            } else {
                llServoPos -= searchSpeed;
                if (llServoPos <= llServoMin) {
                    llServoPos = llServoMin;
                    sweepingForward = true;
                }
            }

            limelightmount.setPosition(llServoPos);
            telemetry.addLine("No target detected — searching...");
        }
    }

    private void logBotPose(LLResult result) {
        odo.update();
        Pose2D pos = odo.getPosition();

        telemetry.addData("Yaw", pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Has Result", (result != null));
        telemetry.addData("Result Valid", (result != null && result.isValid()));
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            telemetry.addData("X (m)", botpose.getPosition().x);
            telemetry.addData("Y (m)", botpose.getPosition().y);
        } else {
            telemetry.addLine("No Limelight pose");
        }
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
//            if (lastBallColor.equals("Unknown")) {
            intakeReady = false;
            ballColor = checkColor();
            lastBallColor = ballColor;
            // Store color in current slot
            slotColors[servoIndex] = ballColor;
            new Thread(() -> {
                sleep(10);
                if (servoIndex < 2) {
                    servoIndex++;
                    sorting1.setPosition(suzani[servoIndex]);
                }
                lastPos = suzani[servoIndex];
                sleep(1100);
                intakeReady = true;
            }).start();
//            }
        }

    }

    private void outtake(String[] outPattern) {
        if (!outtaking) {
            outtaking = true;
            sorting2.setPosition(wackDown);
            List<Double> servoSequence = new ArrayList<>();
            boolean[] used = new boolean[slotColors.length];
            for (String targetColor : outPattern) {
                for (int i = 0; i < slotColors.length; i++) {
                    if (!used[i] && slotColors[i].equalsIgnoreCase(targetColor)) {
                        servoSequence.add(suzano[i]);  // add servo position corresponding to that slot
                        used[i] = true;                // mark slot as used
                        break;                         // move on to next pattern color
                    }
                }
            }
            //        also add unused slots in case color was mismatched
            for (int i = 0; i < slotColors.length; i++) {
                if (!used[i]) {
                    servoSequence.add(suzano[i]);
                    used[i] = true;
                }
            }

            for (int i = 0; i < servoSequence.size(); i++) {
                if (outtaking){
                    double servoPos = servoSequence.get(i);
                    sorting1.setPosition(servoPos);
                    if (Math.abs(lastPos - servoPos) > 0.4) {
                        sleep(2000);
                    } else {
                        sleep(1200);
                    }
                    if (outtaking){
                        sorting2.setPosition(wackUp);
                        sleep(400);
                    }
                    sorting2.setPosition(wackDown);
                    sleep(140);
                    lastPos = servoPos;
                }
            }
            servoIndex = 0;
            slotColors[0] = "Empty";
            slotColors[1] = "Empty";
            slotColors[2] = "Empty";
            sorting2.setPosition(wackDown);
            sleep(140);
            sorting1.setPosition(suzani[servoIndex]);
            lastPos = suzani[servoIndex];
            outtaking = false;
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

    //    AUTOMATIC DRIVING CODE
    private double smooth(double prev, double curr, double alpha) {
        return prev * (1 - alpha) + curr * alpha;
    }

    private void driveToOrigin(double targX, double targY, double targYaw) {
        // Update odometry
        odo.update();
        Pose2D pos = odo.getPosition();
        double currYawOdo = pos.getHeading(AngleUnit.RADIANS);

        // ---- Limelight ----
// ---- Limelight ----
//        LLResult result = limelight.getLatestResult();
//        if (!result.isValid() || result.getBotposeTagCount() == 0) {
//            telemetry.addLine("Bad LL frame, ignoring");
//            return;
//        }
//
//        Pose3D botpose = result.getBotpose();
//        double camX  = botpose.getPosition().x;
//        double camY  = botpose.getPosition().y;
//        // now call correction (camYaw is in radians)
//        Pose2D corrected = correctForCameraOffset(
//                camX,
//                camY,
//                llServoPos,
//                currYawOdo
//        );
//        // corrected robot pose from LL
//        double currX = corrected.getX(DistanceUnit.METER);
//        double currY = corrected.getY(DistanceUnit.METER);
        double currYaw = currYawOdo;

        // ---- First frame filter init ----
//        if (firstFrame) {
//            filtX = currX;
//            filtY = currY;
//            filtYaw = currYaw;
//            firstFrame = false;
//        }

        // ---- LPF ----
        double alpha = 0.25;
//        filtX = smooth(filtX, currX, alpha);
//        filtY = smooth(filtY, currY, alpha);
        filtYaw = smooth(filtYaw, currYaw, alpha);

        // ---- Errors ----
//        double fieldErrorX = targX - filtX;
//        double fieldErrorY = targY - filtY;
//        double distance = Math.hypot(fieldErrorX, fieldErrorY);

        double yawError = targYaw - filtYaw;
        yawError = Math.atan2(Math.sin(yawError), Math.cos(yawError));

        // ---- Gains ----
        double kP_drive = 1.0;
        double kP_turn = 0.7;

        // ---- Convert field → robot ----
//        double robotErrorX =  fieldErrorX * Math.cos(-filtYaw)
//                - fieldErrorY * Math.sin(-filtYaw);
//        double robotErrorY =  fieldErrorX * Math.sin(-filtYaw)
//                + fieldErrorY * Math.cos(-filtYaw);
//
//        double targetAxial   = robotErrorX * kP_drive;
//        double targetLateral = -robotErrorY * kP_drive;
        double targetYaw = yawError * kP_turn;
        double targetAxial = 0;
        double targetLateral = 0;

        // ---- Slow-down zone ----
//        double slowdownDist = 1;
//        if (distance < slowdownDist) {
//            double scale = Math.max(distance / slowdownDist, 0.5);
//            targetAxial   *= scale;
//            targetLateral *= scale;
//            targetYaw     *= scale;
//        }

        // ---- Overcome friction ----
        double minPower = 0.07;
        if (Math.abs(targetAxial) < minPower && Math.abs(targetAxial) > 0)
            targetAxial = Math.copySign(minPower, targetAxial);

        if (Math.abs(targetLateral) < minPower && Math.abs(targetLateral) > 0)
            targetLateral = Math.copySign(minPower, targetLateral);

        if (Math.abs(targetYaw) < minPower && Math.abs(targetYaw) > 0)
            targetYaw = Math.copySign(minPower, targetYaw);

        // ---- Mecanum mixing ----
        double fl = targetAxial + targetLateral + targetYaw;
        double fr = targetAxial - targetLateral - targetYaw;
        double bl = targetAxial - targetLateral + targetYaw;
        double br = targetAxial + targetLateral - targetYaw;

        // ---- Normalize ----
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        // ---- Stop if at target ----
        double posTol = 0.05;        // meters

        if (Math.abs(yawError) < yawTol) {
            fL.setPower(0);
            fR.setPower(0);
            bL.setPower(0);
            bR.setPower(0);
            telemetry.addLine("Reached target");
            return;
        }

        // ---- Apply ----
        fL.setPower(fl);
        fR.setPower(fr);
        bL.setPower(bl);
        bR.setPower(br);

        telemetry.addData("Yaw Error", Math.toDegrees(yawError));
    }
    private Pose2D correctForCameraOffset(
            double camX,
            double camY,
            double servoPos,
            double robotYaw         // from odometry
    ) {
        double servoAngle = servoPosToAngle(servoPos);

        // Camera world yaw = robot yaw + servo rotation
        double cameraWorldYaw = robotYaw + servoAngle;

        // Camera offset from robot center:
        double dx = CAMERA_OFFSET_X;
        double dy = CAMERA_OFFSET_Y;

        // Rotate offset into world space
        double fieldDX = dx * Math.cos(cameraWorldYaw) - dy * Math.sin(cameraWorldYaw);
        double fieldDY = dx * Math.sin(cameraWorldYaw) + dy * Math.cos(cameraWorldYaw);

        // Convert camera position -> robot center
        double robotX = camX - fieldDX;
        double robotY = camY - fieldDY;

        return new Pose2D(
                DistanceUnit.METER,
                robotX,
                robotY,
                AngleUnit.RADIANS,
                robotYaw     // keep robot yaw consistent
        );
    }

    private double servoPosToAngle(double pos) {
        double spanPos = SERVO_MAX_POS - SERVO_MIN_POS;
        double offsetPos = pos-SERVO_CENTER_POS;

        // Convert offset in 0-1 range to angle
        double normalized = offsetPos / spanPos;

        return normalized * SERVO_TOTAL_ANGLE;
    }

    private void printCurrentRobotPose() {
        // Update odometry
        odo.update();
        Pose2D pos = odo.getPosition();
        double robotYaw = pos.getHeading(AngleUnit.RADIANS);

        // Get limelight frame
        LLResult result = limelight.getLatestResult();

        if (!result.isValid() || result.getBotposeTagCount() == 0) {
            telemetry.addLine("LL: No valid tag");
            telemetry.addData("Odo X", pos.getX(DistanceUnit.METER));
            telemetry.addData("Odo Y", pos.getY(DistanceUnit.METER));
            telemetry.addData("Odo Yaw", Math.toDegrees(robotYaw));
            return;
        }

        Pose3D botpose = result.getBotpose();
        double camX  = botpose.getPosition().x;
        double camY  = botpose.getPosition().y;
        double camYaw = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

        // Correct camera offset (servo angle included)
        Pose2D corrected = correctForCameraOffset(
                camX,
                camY,
                llServoPos,
                robotYaw
        );

        double trueX = corrected.getX(DistanceUnit.METER);
        double trueY = corrected.getY(DistanceUnit.METER);

        telemetry.addLine("Robot Position:");
        telemetry.addData("X (m)", trueX);
        telemetry.addData("Y (m)", trueY);
        telemetry.addData("Yaw (deg)", Math.toDegrees(robotYaw));
    }

}
