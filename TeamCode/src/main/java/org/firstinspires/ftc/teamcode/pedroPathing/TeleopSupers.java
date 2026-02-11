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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static org.firstinspires.ftc.teamcode.CONSTANTS.*;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
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

@TeleOp(name="Tele-Op", group="Linear OpMode")
//@Disabled
public class TeleopSupers extends LinearOpMode {
    GoBildaPinpointDriver odo;
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
    private boolean sweepingForward = true;
    private boolean intakeReady = true;
    private boolean needPattern = true;
    private boolean outtaking = false;
    private boolean wackSet = false;
    private boolean driverLock = false;
    private boolean up = false;
    double integralSum = 0;
    double lastError = 0;

    //    manual booleans
    private boolean aWasPressed = false;
    private boolean yWasPressed = false;
    private boolean xWasPressed = false;
    private double lastPos = suzani[servoIndex];
    private double fwCurrSpeed = fwFarSpeed;
    private double turretPower = 0.95;
    private double targetTagID = 20;
    private double lastDirection = 1;

    //        AXON ENCODER TRACKING
    private AnalogInput axonEncoder;
    private static final double MAX_VOLTAGE = 3.3; // Check your specific hub, usually 3.3V
    private static final double GEAR_RATIO = 5.0;  // 5:1 Reduction
    private double lastVoltage = 0;
    private int rotationCount = 0;
    // Safety Limits (Degrees)
    private static final double MAX_TURRET_ANGLE = 180;
    private static final double MIN_TURRET_ANGLE = -180;
    ElapsedTime timer = new ElapsedTime();

    @Override//
    public void runOpMode() {
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

        fwl.setDirection(DcMotor.Direction.FORWARD);
        fwr.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{fL, fR, bL, bR}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            new Thread(()->{
//            fwl.setVelocity(fwCurrSpeed);
//            fwr.setVelocity(fwCurrSpeed);
                moveTurret();
            }).start();
            if (!wackSet){
                wackSet = true;
                flick1.setPosition(flicksDown[0]);
                flick2.setPosition(flicksDown[1]);
                flick3.setPosition(flicksDown[2]);
            }
            limelight.pipelineSwitch(1);
            driveMecanum();

            if (gamepad1.left_trigger >0.1){
                fwCurrSpeed = fwNearSpeed;
            } else if (gamepad1.right_trigger >0.1){
                fwCurrSpeed = fwFarSpeed;
            }
            if (gamepad1.y){
                slotColors[0] = "Empty";
                slotColors[1] = "Empty";
                slotColors[2] = "Empty";
            }

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

            if (gamepad2.dpad_left){
                outtaking = false;
            }

            if (gamepad1.a && !aWasPressed){
                aWasPressed = true;
                if (!up){
                    flick1.setPosition(flicksDown[0]);
                    up=true;
                } else {
                    up = false;
                    flick1.setPosition(flicksUp[0]);
                }
            }

            if (gamepad1.b && !aWasPressed){
                aWasPressed = true;
                if (!up){
                    flick2.setPosition(flicksDown[1]);
                    up=true;
                } else {
                    up = false;
                    flick2.setPosition(flicksUp[1]);
                }
            }

            if (gamepad1.x && !aWasPressed){
                aWasPressed = true;
                if (!up){
                    flick3.setPosition(flicksDown[2]);
                    up=true;
                } else {
                    up = false;
                    flick3.setPosition(flicksUp[2]);
                }
            }

            if (!gamepad1.a && !gamepad1.b && !gamepad1.x){
                aWasPressed = false;
            }
            telemetry.addData("Pressed", aWasPressed);
            telemetry.addData("Up", up);

            checkColor();
            telemetry.addData("Slot 1", slotColors[0]);
            telemetry.addData("Slot 2", slotColors[1]);
            telemetry.addData("Slot 3", slotColors[2]);
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
            integralSum = 0;
            if (turretAngle >= MAX_TURRET_ANGLE) {
                lastDirection = -1;
            } else if (turretAngle <= MIN_TURRET_ANGLE) {
                lastDirection = 1;
            }
            turretPower = 0.99*lastDirection;
            turret.setPower(turretPower);
            telemetry.addLine("SWEEPING");
        }
        else if (locked) {
            double error = tx;
            double dt = timer.seconds();
            if (dt == 0) dt = 0.001; // Safety

            if (Math.abs(error) < 15.0) { // Only accumulate I if relatively close
                integralSum += (error * dt);
            } else {
                integralSum = 0; // Reset if far away
            }
            double errorRate = (error - lastError) / dt;
            errorRate = Range.clip(errorRate, -100, 100);

            double pTerm = turretkP * error;
            double iTerm = turretkI * integralSum;
            double dTerm = turretkD * errorRate;
            double fTerm = 0;
            if (Math.abs(error) > 3.0) {  // degrees
                fTerm = Math.signum(error) * turretkF;
            }
            outputPower = pTerm + iTerm + dTerm + fTerm;
            lastError = error;
            timer.reset();
            if (Math.abs(error) > 1) {
                turret.setPower(Range.clip(outputPower, -0.75, 0.75));
            } else {
                turret.setPower(0);
            }
            telemetry.addLine("LOCKED");
        }
        telemetry.addData("Angle", turretAngle);
        telemetry.addData("Rotation count", rotationCount);
    }

    private void intake() {
        if (intakeReady) {
            intake1.setDirection(DcMotor.Direction.REVERSE);
            intake1.setPower(intakeSpeed);
        }
    }

    private void outtake(String[] outPattern) {
        if (!outtaking) {
            outtaking = true;
            checkColor();
            List<Integer> servoSequence = new ArrayList<>();
            boolean[] used = new boolean[slotColors.length];
            for (String targetColor : outPattern) {
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
}
