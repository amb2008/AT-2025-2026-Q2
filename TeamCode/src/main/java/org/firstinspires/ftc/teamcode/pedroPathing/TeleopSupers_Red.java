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
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static org.firstinspires.ftc.teamcode.CONSTANTS.*;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
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

@TeleOp(name="Tele-Op - RED", group="Linear OpMode")
//@Disabled
public class TeleopSupers_Red extends LinearOpMode {
    GoBildaPinpointDriver odo;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime colorTimer = new ElapsedTime();
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
    private boolean sweepingForward = true;
    private boolean intakeReady = true;
    private boolean needPattern = true;
    private boolean outtaking = false;
    private boolean retry = true;
    private boolean wackSet = false;
    private boolean driverLock = false;
    private boolean far = false;
    private boolean up = false;
    private boolean sweep = true;
    double integralSum = 0;
    double lastError = 0;


    //    manual booleans
    private boolean aWasPressed = false;
    private boolean trigPressed = false;
    private boolean yWasPressed = false;
    private boolean xWasPressed = false;
    private double lastPos = suzani[servoIndex];
    private double fwCurrSpeed = fwFarSpeed;
//    TURRET
    private double turretPower = 0.95;
    private double targetTagID = 24;
    private double lastDirection = 1;
    private double lastTurretAngle = 0;
    private double stuckTimer = 0;
    private double counter = 0;
    private double fwkp = 0.041;
    private static final double STUCK_ANGLE_THRESHOLD = 1;   // degrees
    private static final double STUCK_TIME_THRESHOLD = 0.2;    // seconds

    //        AXON ENCODER TRACKING
    private AnalogInput axonEncoder;
    private static final double MAX_VOLTAGE = 3.3; // Check your specific hub, usually 3.3V
    private static final double GEAR_RATIO = 5.0;  // 5:1 Reduction
    private double lastVoltage = 0;
    private int rotationCount = 0;
    // Safety Limits (Degrees)
    private static final double MAX_TURRET_ANGLE = 90;
    private static final double MIN_TURRET_ANGLE = -90;
    //    FLYWHEEL
    private IMU imu;
    PIDController pid = new PIDController(fwkp, 0.0, 0.0);
    final double MAX_MOTOR_RPM = 6000;      // GoBILDA 6000 RPM
    final double TICKS_PER_REV = 28;        // Encoder CPR
    final double MAX_VELOCITY = (MAX_MOTOR_RPM / 60.0) * TICKS_PER_REV; // ticks/sec
    double targetVelocity = 500;

    boolean fwoff = false;
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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addLine("AT is the best!");
            if (!fwoff){
                fwOn();
            } else {
                fwr.setPower(-0.3);
                fwl.setPower(-0.3);
            }
            if (sweep){
                moveTurret();
            }
            if (!wackSet){
                wackSet = true;
                outtaking = false;
                grant.setPosition(0.02);
                flick1.setPosition(flicksDown[0]);
                flick2.setPosition(flicksDown[1]);
                flick3.setPosition(flicksDown[2]);
            }
            limelight.pipelineSwitch(1);
            driveMecanum();

            if (gamepad1.left_trigger >0.1 && !trigPressed){
                trigPressed = true;
                lastDirection = -lastDirection;
            } else if (gamepad1.right_trigger >0.1 && !trigPressed){
                trigPressed = true;
                lastDirection = -lastDirection;
            }
            if (gamepad1.left_trigger <0.1 && gamepad1.right_trigger <0.1 ){
                trigPressed = false;
            }
            if (gamepad2.dpad_down) {
                fwOff();
            } else if (gamepad2.dpad_up) {
                fwCurrSpeed = fwNearSpeed;
                fwoff = false;
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

            if (gamepad2.x) {
                new Thread(()->{
                    dumbOuttake();
                }).start();
            }

//            GAMEPAD 2
            if (gamepad2.left_trigger > 0.1) {
                intake();
            } else if (gamepad2.right_trigger > 0.1) {
                intake1.setDirection(DcMotor.Direction.FORWARD);
                intake1.setPower(intakeSpeed);
                intake2.setDirection(DcMotor.Direction.REVERSE);
                intake2.setPower(intakeSpeed);
            }
            else {
                intake1.setPower(0);
                intake2.setPower(0);
            }

            if (gamepad2.left_bumper && gamepad2.right_bumper){
                sweep = true;
                rotationCount = 0;
            } else if (gamepad2.left_bumper){
                turret.setPower(-0.99);
                sweep = false;
            } else if (gamepad2.right_bumper){
                turret.setPower(0.99);
                sweep = false;
            } else if (!sweep){
                turret.setPower(0);
            }

            if (gamepad2.dpad_left){
                outtaking = false;
                retry = false;
                new Thread(()->{
                    sleep(1000);
                    retry = true;
                }).start();
            }


            if (gamepad1.a && !aWasPressed){
                aWasPressed = true;
                new Thread(()->{
                    flick1.setPosition(flicksUp[0]);
                    sleep(500);
                    flick1.setPosition(flicksDown[0]);
                    flick2.setPosition(flicksDown[1]);
                    flick3.setPosition(flicksDown[2]);
                }).start();
            }

            if (gamepad1.b && !aWasPressed){
                aWasPressed = true;
                new Thread(()->{
                    flick3.setPosition(flicksUp[2]);
                    sleep(500);
                    flick1.setPosition(flicksDown[0]);
                    flick2.setPosition(flicksDown[1]);
                    flick3.setPosition(flicksDown[2]);
                }).start();
            }

            if (gamepad1.y && !aWasPressed){
                aWasPressed = true;
                new Thread(()->{
                    flick2.setPosition(flicksUp[1]);
                    sleep(500);
                    flick1.setPosition(flicksDown[0]);
                    flick2.setPosition(flicksDown[1]);
                    flick3.setPosition(flicksDown[2]);
                }).start();
            }

            if (gamepad1.x && !aWasPressed){
                aWasPressed = true;
                new Thread(()-> {
                    grant.setPosition(0.5);
                    sleep(300);
                    grant.setPosition(0.02);
                }).start();
            }

            if (!gamepad1.a && !gamepad1.b && !gamepad1.x && !gamepad1.y && !gamepad2.right_bumper && !gamepad2.left_bumper){
                aWasPressed = false;
            }


            if (colorTimer.seconds() >= 0.5) {
                checkColor();
                colorTimer.reset(); // Restart the clock for the next 0.5s
            }

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
        yaw = Range.clip(yaw, -0.8, 0.8);

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
                    lastDirection = Math.signum(tx+1)*1;
                    if (far){
                        lastDirection = Math.signum(tx+4.5)*1;
                    }
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
            // ---------------- STUCK DETECTION ----------------
//            double angleChange = Math.abs(turretAngle - lastTurretAngle);
//
//            if (angleChange < STUCK_ANGLE_THRESHOLD) {
//                stuckTimer += timer.seconds();
//            } else {
//                stuckTimer = 0;
//            }
//
//            if (stuckTimer > STUCK_TIME_THRESHOLD) {
//                lastDirection *= -1;   // reverse direction
//                stuckTimer = 0;        // reset timer
//            }
//            lastTurretAngle = turretAngle;
//            timer.reset();
        }
        else if (locked) {
            double error = tx+1;
            if (far){
                error += 3.5;
            }
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
    }

    private void intake() {
        if (intakeReady) {
            intake1.setDirection(DcMotor.Direction.REVERSE);
            intake1.setPower(intakeSpeed);
            intake2.setDirection(DcMotor.Direction.FORWARD);
            intake2.setPower(intakeSpeed);
        }
    }

    private void outtake(String[] outPattern) {
        if (!outtaking) {
            outtaking = true;
            counter = 0;
            for (String targetColor : outPattern) {
                if (outtaking){
                    counter += 1;
                    boolean launched = false;
                    checkColor();
                    for (int i = 0; i < slotColors.length; i++){
                        if (slotColors[i].equalsIgnoreCase(targetColor)){
                            if (i==0){
                                flick1.setPosition(flicksUp[0]);
                            } else if (i==1){
                                flick2.setPosition(flicksUp[1]);
                            } else if (i==2){
                                flick3.setPosition(flicksUp[2]);
                            }
                            launched = true;
                            break;
                        }
                    }
                    if (!launched){
                        for (int i = 0; i < slotColors.length; i++){
                            if (slotColors[i].equalsIgnoreCase("green") || slotColors[i].equalsIgnoreCase("purple")){
                                if (i==0){
                                    flick1.setPosition(flicksUp[0]);
                                } else if (i==1){
                                    flick2.setPosition(flicksUp[1]);
                                } else if (i==2){
                                    flick3.setPosition(flicksUp[2]);
                                }
                                launched = true;
                                break;
                            }
                        }
                    }
                    if (launched){
                        sleep(400);
                        flick1.setPosition(flicksDown[0]);
                        flick2.setPosition(flicksDown[1]);
                        flick3.setPosition(flicksDown[2]);
                    }

                    checkColor();
                    if (counter<3){
                        sleep(300);
                    }
                    if (counter == 1){
                        sleep(200);
                        checkColor();
                        if (!slotColors[0].equalsIgnoreCase("Empty") && !slotColors[1].equalsIgnoreCase("Empty") && !slotColors[2].equalsIgnoreCase("Empty")){
                            outtaking = false;
                            if (retry) {
                                outtake(outPattern);
                            }
                            break;
                        }
                    }
                }
            }
            counter = 0;
            outtaking = false;
        }
    }

    private void dumbOuttake() {
        if (!outtaking) {
            outtaking = true;
            counter = 1;
            flick1.setPosition(flicksUp[0]);
            sleep(400);
            flick1.setPosition(flicksDown[0]);
            flick2.setPosition(flicksDown[1]);
            flick3.setPosition(flicksDown[2]);
            sleep(400);
            counter += 1;
            flick2.setPosition(flicksUp[1]);
            sleep(400);
            flick1.setPosition(flicksDown[0]);
            flick2.setPosition(flicksDown[1]);
            flick3.setPosition(flicksDown[2]);
            sleep(400);
            flick3.setPosition(flicksUp[2]);
            sleep(400);
            flick1.setPosition(flicksDown[0]);
            flick2.setPosition(flicksDown[1]);
            flick3.setPosition(flicksDown[2]);
            counter = 0;
            outtaking = false;
        }
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
        telemetry.addData("Red", cs1a.red());
        telemetry.addData("Green", cs1a.green());
        telemetry.addData("Blue", cs1a.blue());

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
    private void fwOff() {
        fwl.setPower(0);
        fwr.setPower(0);
        fwoff = true;
    }

    private void fwOn(){
        //limelight stuff
        limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()){
            Pose3D botpose = llResult.getBotpose();
            double camX  = -botpose.getPosition().x;
            double camY  = botpose.getPosition().y;
            double distance = Math.sqrt(Math.pow((mtRedX-camX), 2) + Math.pow((mtRedY-camY),2));
            targetVelocity = 49.17058*Math.pow((distance), 2)-26.44751*distance+465.26609;
            targetVelocity = targetVelocity * 1.05; //tweak if shooting to short or far
            if (counter == 0){
                targetVelocity += 10;
            }
            targetVelocity = Math.round((double) targetVelocity/ 20) * 20; //Ensure a multiple of 20 to simplify PID
            if (targetVelocity > 700){
                targetVelocity = 840;
                far = true;
            } else {
                far = false;
            }
            telemetry.addData("Distance", distance);
            telemetry.addData("Target velocity", targetVelocity);
        } else {
            telemetry.addLine("No result");
        }

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

    private double[] normalizeColor(double[] rgb) {
        double sum = rgb[0] + rgb[1] + rgb[2];
        return new double[]{rgb[0] / sum, rgb[1] / sum, rgb[2] / sum};
    }

    private double colorDistance(double[] c1, double[] c2) {
        return Math.sqrt(Math.pow(c1[0] - c2[0], 2) + Math.pow(c1[1] - c2[1], 2) + Math.pow(c1[2] - c2[2], 2));
    }
}
