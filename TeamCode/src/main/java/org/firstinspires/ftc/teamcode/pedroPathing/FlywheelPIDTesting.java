package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "Flywheel Tester", group = "Debug")
//@Disabled
public class FlywheelPIDTesting extends LinearOpMode {

    private DcMotorEx flywheelLeft;
    private DcMotorEx flywheelRight;
    private Limelight3A limelight;
    private double distance;

    private IMU imu;

    // Single PID controller for both motors
    PIDController pid = new PIDController(0.0115, 0.0, 0.0);

    // Target velocity (ticks/sec)
    double targetVelocity = 1100;
    double velocityIncrement = 20; // ticks/sec per D-pad press

    // PID tuning increments
    double stepP = 0.0005;
    double stepI = 0.00005;
    double stepD = 0.0005;

    // Debounce variables
    boolean yPrev, aPrev, xPrev, bPrev, upPrev, downPrev;
    boolean leftPrev, rightPrev, bumperLeftPrev, bumperRightPrev;

    // Motor enable flag
    boolean motorsEnabled = true;

    // Motor specifications
    final double MAX_MOTOR_RPM = 6000;      // GoBILDA 6000 RPM
    final double TICKS_PER_REV = 28;        // Encoder CPR
    final double MAX_VELOCITY = (MAX_MOTOR_RPM / 60.0) * TICKS_PER_REV; // ticks/sec

    @Override
    public void runOpMode() {

        flywheelLeft = hardwareMap.get(DcMotorEx.class, "fwl");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "fwr");

        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelLeft.setDirection(DcMotor.Direction.REVERSE);
        flywheelRight.setDirection(DcMotor.Direction.FORWARD);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();

        telemetry.addLine("Dual Flywheel PID Tuner (GoBILDA 6000 RPM) Ready");
        telemetry.update();

        imu = hardwareMap.get(IMU.class, "imu");

        // Define how the hub is mounted on your robot
        // Adjust these to match your actual build!
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Initialize the IMU with these settings
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        waitForStart();

        pid.setSetpoint(targetVelocity);

        while (opModeIsActive()) {



            // ----------------------------
            // PID CONSTANT ADJUSTMENT
            // ----------------------------
            if (gamepad1.y && !yPrev) pid.setKp(pid.getKp() + stepP);
            if (gamepad1.a && !aPrev) pid.setKp(pid.getKp() - stepP);

            if (gamepad1.x && !xPrev) pid.setKi(pid.getKi() + stepI);
            if (gamepad1.b && !bPrev) pid.setKi(pid.getKi() - stepI);

            if (gamepad1.dpad_up && !upPrev) pid.setKd(pid.getKd() + stepD);
            if (gamepad1.dpad_down && !downPrev) pid.setKd(pid.getKd() - stepD);

            // ----------------------------
            // TARGET VELOCITY CONTROL
            // ----------------------------
            if (gamepad1.dpad_left && !leftPrev) {
                targetVelocity += velocityIncrement;
                if (targetVelocity > MAX_VELOCITY) targetVelocity = MAX_VELOCITY;
                pid.setSetpoint(targetVelocity);
            }
            if (gamepad1.dpad_right && !rightPrev) {
                targetVelocity -= velocityIncrement;
                if (targetVelocity < 0) targetVelocity = 0;
                pid.setSetpoint(targetVelocity);
            }

            // ----------------------------
            // MOTOR ENABLE / DISABLE
            // ----------------------------
            if (gamepad1.left_bumper && !bumperLeftPrev) motorsEnabled = true;
            if (gamepad1.right_bumper && !bumperRightPrev) motorsEnabled = false;

            // ----------------------------
            // Update debounce states
            // ----------------------------
            yPrev = gamepad1.y;
            aPrev = gamepad1.a;
            xPrev = gamepad1.x;
            bPrev = gamepad1.b;
            upPrev = gamepad1.dpad_up;
            downPrev = gamepad1.dpad_down;
            leftPrev = gamepad1.dpad_left;
            rightPrev = gamepad1.dpad_right;
            bumperLeftPrev = gamepad1.left_bumper;
            bumperRightPrev = gamepad1.right_bumper;

            // ----------------------------
            // FLYWHEEL CONTROL (AVERAGE VELOCITY)
            // ----------------------------
            double leftVelocity = flywheelLeft.getVelocity();
            double rightVelocity = flywheelRight.getVelocity();
            double avgVelocity = (leftVelocity + rightVelocity) / 2.0;

            double pidOutput = pid.update(avgVelocity);

            // Feedforward based on max motor velocity
            double feedforward = targetVelocity / MAX_VELOCITY;
            pidOutput += feedforward;

            // Clip to [0,1]
            pidOutput = Math.max(0, Math.min(pidOutput, 1));

            // Apply to motors if enabled
            if (motorsEnabled) {
                flywheelLeft.setPower(pidOutput);
                flywheelRight.setPower(pidOutput);
            } else {
                flywheelLeft.setPower(0);
                flywheelRight.setPower(0);
            }

            //limelight stuff
            limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

            LLResult llResult = limelight.getLatestResult();

            if (llResult != null && llResult.isValid()){
                Pose3D botpose = llResult.getBotpose();
                double camX  = -botpose.getPosition().x;
                double camY  = botpose.getPosition().y;
                distance = getDistance(camX, camY);
                telemetry.addData("distance", distance);
                telemetry.addData("Cam X", camX);
                telemetry.addData("Cam Y", camY);
                telemetry.addData("Target X", llResult.getTx());
                telemetry.addData("Target Area", llResult.getTa());
                telemetry.addData("Botpose", botpose.toString());
            } else {
                telemetry.addLine("No result");
            }

            // ----------------------------
            // TELEMETRY
            // ----------------------------
            telemetry.addLine("=== PID CONSTANTS ===");
            telemetry.addData("Kp", pid.getKp());
            telemetry.addData("Ki", pid.getKi());
            telemetry.addData("Kd", pid.getKd());

            telemetry.addLine("\n=== FLYWHEEL VELOCITY ===");
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Left Velocity", leftVelocity);
            telemetry.addData("Right Velocity", rightVelocity);
            telemetry.addData("Average Velocity", avgVelocity);
            telemetry.addData("PID Output", pidOutput);
            telemetry.addData("Motors Enabled", motorsEnabled);

            telemetry.addLine("\n=== Controls ===");
            telemetry.addLine("P: Y ↑ | A ↓");
            telemetry.addLine("I: X ↑ | B ↓");
            telemetry.addLine("D: Up ↑ | Down ↓");
            telemetry.addLine("Velocity: Left ↑ | Right ↓");
            telemetry.addLine("Motor On: Left Bumper | Off: Right Bumper");

            telemetry.update();
        }
    }

public double getDistance(double camX, double camY){
//      double scale = 38665.95;
//      double distance = (scale/ta);
//      return distance;
    double mtBlueX = -1.482;
    double mtBlueY = -1.413;
    double mtRedX = 1.482;
    double mtRedY = 1.413;
    double distance = Math.sqrt(Math.pow((mtRedX-camX), 2) + Math.pow((mtRedY-camY),2));
    return distance;
}
}

