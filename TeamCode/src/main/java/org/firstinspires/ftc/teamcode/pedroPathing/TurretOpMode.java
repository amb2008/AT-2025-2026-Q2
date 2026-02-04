package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *  How to Tune!
 *
 *  If the simple kP version wasn't enough, follow this order to tune this advanced version:
 *
 *  1. Find kF first: With the robot on, increase kF until the turret just barely starts to move
 *  when there is a tiny error, the back it off slightly. This "primes' the servo to overcome friction.
 *
 *  2. Set kP: Increase kP until the turret moves quickly to the target but starts to bounce (oscillate).
 *
 *  3. Add kD: Slowly add kD (start with very small numbers like 0.001). This acts like "brakes."
 *  It will see the turret approaching the cneter and counter-act the kP to make it settle softly
 */

@TeleOp(name = "Turret PIDF", group = "TeleOp")
public class TurretOpMode extends LinearOpMode {

    private Limelight3A limelight;
    private CRServo turretServo;
    private ElapsedTime timer = new ElapsedTime();
    private boolean dpadPressed = false;

    int targetTagID = 20; // 20 for Blue, 24 for Red
    // PIDF COEFICIENTS
    double kP = 0.04;
    double kI = 0.0;   // Rarely needed for turrets (that said Louisa and I did make adjustments to kI for the Limelight 4)
    double kD = 0.002; // Prevents "slamming" into position
    double kF = 0.05;  // Minimum power to overcome 5:1 gear friction

    // Tracking Variables
    double lastError = 0;
    double integralSum = 0;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretServo = hardwareMap.get(CRServo.class, "turret");

        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(1);
        limelight.start();
        timer.reset();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            double stickInput = -gamepad1.right_stick_x;
            double outputPower = 0;

            boolean targetLocked = false;
            double targetTx = 0;

            if (gamepad1.right_bumper && !dpadPressed){
                dpadPressed = true;
                kP += 0.001;
            } else if (gamepad1.left_bumper && !dpadPressed){
                dpadPressed = true;
                kP -= 0.001;
            }

            if (gamepad1.a && !dpadPressed){
                dpadPressed = true;
                kI += 0.001;
            } else if (gamepad1.b && !dpadPressed){
                dpadPressed = true;
                kI -= 0.001;
            }

            if (gamepad1.dpad_up && !dpadPressed){
                dpadPressed = true;
                kD += 0.001;
            } else if (gamepad1.dpad_down && !dpadPressed){
                dpadPressed = true;
                kD -= 0.001;
            }

            if (gamepad1.dpad_right && !dpadPressed){
                dpadPressed = true;
                kF += 0.001;
            } else if (gamepad1.dpad_left && !dpadPressed){
                dpadPressed = true;
                kF -= 0.001;
            }


            if (!gamepad1.left_bumper && !gamepad1.right_bumper && !gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.a && !gamepad1.b){
                dpadPressed = false;
            }

            telemetry.addLine("Bump Right = kP+");
            telemetry.addLine("Bump Left = kP-");
            telemetry.addLine("Dpad Up = kD+");
            telemetry.addLine("Dpad Down = kD-");
            telemetry.addLine("Dpad Right = kF+");
            telemetry.addLine("Dpad Left = kF-");
            telemetry.addLine("A = kI+");
            telemetry.addLine("B = kI-");
//            kF = 0.074

            // 1. FILTER FOR SPECIFIC DECODE TAG
            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    if (tag.getFiducialId() == targetTagID) {
                        targetTx = tag.getTargetXDegrees();
                        targetLocked = true;
                        break;
                    }
                }
            }

            // 2. PRIORITY LOGIC
            if (Math.abs(stickInput) > 0.1) {
                outputPower = stickInput; // Manual Control
            }
            else if (targetLocked) {
                // PIDF Calculation
                double error = targetTx;
                double dt = timer.seconds();
                double pTerm = kP * error;
                double iTerm = kI;
                double dTerm = kD * ((error - lastError) / dt);
                double fTerm = Math.signum(error) * kF;

                outputPower = pTerm + dTerm + fTerm;

                lastError = error;
                timer.reset();
            }

            // 3. APPLY & PROTECT
            // Limited to 0.80 power to protect the 5:1 gears from violent snaps
            if (Math.abs(targetTx) > 0.5){
                turretServo.setPower(Range.clip(outputPower, -0.80, 0.80));
            } else {
                turretServo.setPower(0);
            }

            telemetry.addData("Target", targetLocked ? "LOCKED" : "SEARCHING");
            telemetry.addData("TX", targetTx);
            telemetry.addData("kF", kF);
            telemetry.addData("kP", kP);
            telemetry.addData("kD", kD);
            telemetry.addData("kI", kP);
            telemetry.update();
        }
    }
}