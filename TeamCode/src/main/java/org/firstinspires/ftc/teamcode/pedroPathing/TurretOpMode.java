package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "PIDF TUNER", group = "Tuning")
public class TurretOpMode extends LinearOpMode {

    // Hardware
    private Limelight3A limelight;
    private CRServo turretServo;
    private ElapsedTime timer = new ElapsedTime();

    // --- PIDF VARIABLES (Modifiable) ---
    // We make these non-final so we can change them live
    double kP = 0.045;
    double kI = 0.00;   // <-- You can now tune this
    double kD = 0.002;
    double kF = 0.06;

    // Tuning State
    String[] options = {"P", "I", "D", "F"};
    int selectionIndex = 0; // 0=P, 1=I, 2=D, 3=F
    boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;

    // PID State
    double integralSum = 0;
    double lastError = 0;

    // Config
    int targetTagID = 20; // Default to Blue

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretServo = hardwareMap.get(CRServo.class, "turret");

        limelight.start();
        limelight.pipelineSwitch(1);

        telemetry.addData("Instructions", "D-Pad UP/DOWN: Select Term");
        telemetry.addData("Instructions", "D-Pad LEFT/RIGHT: Adjust Value");
        telemetry.addData("Instructions", "Hold X to force Integral Reset");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            // --- 1. GAMEPAD TUNING LOGIC ---
            handleTuningInput();

            // --- 2. VISION & PID CALCULATION ---
            double outputPower = 0;
            double tx = 0;
            boolean locked = false;

            if (result != null && result.isValid()) {
                // For tuning, we will track ANY tag for simplicity,
                // or you can stick to specific IDs.
                // Here we use the primary target for ease of testing.
                tx = result.getTx();
                locked = true;

                double error = tx;
                double dt = timer.seconds();
                if (dt == 0) dt = 0.001; // Safety

                // P
                double pTerm = kP * error;

                // I (With windup protection)
                if (Math.abs(error) < 15.0) { // Only accumulate I if relatively close
                    integralSum += (error * dt);
                } else {
                    integralSum = 0; // Reset if far away
                }
                // Manual Reset via Controller
                if (gamepad1.x) integralSum = 0;

                double iTerm = kI * integralSum;

                // D
                double dTerm = kD * ((error - lastError) / dt);

                // F
                double fTerm = Math.signum(error) * kF;

                outputPower = pTerm + iTerm + dTerm + fTerm;

                lastError = error;
                timer.reset();
            } else {
                // If we lose target, reset Integral so it doesn't spin wildly upon relock
                integralSum = 0;
            }

            // --- 3. APPLY POWER ---
            // Allow manual override to reset turret if it spins too far
            if (Math.abs(gamepad1.right_stick_x) > 0.1) {
                outputPower = -gamepad1.right_stick_x;
                integralSum = 0;
            }

            if (Math.abs(tx) > 1.0) {
                turretServo.setPower(Range.clip(outputPower, -0.75, 0.75));
            } else {
                turretServo.setPower(0);
            }
            // --- 4. TELEMETRY DASHBOARD ---
            telemetry.addData("SELECTED", ">> %s <<", options[selectionIndex]);
            telemetry.addData("VALUES", "P:%.4f  I:%.4f  D:%.4f  F:%.4f", kP, kI, kD, kF);
            telemetry.addData("-----------------", "-");
            telemetry.addData("Target", locked ? "LOCKED" : "Searching");
            telemetry.addData("Error (tx)", "%.2f", tx);
            telemetry.addData("Integral Sum", "%.2f", integralSum);
            telemetry.addData("Motor Power", "%.2f", outputPower);
            telemetry.update();
        }
    }

    // Handles the button presses with "Debouncing" so values don't change too fast
    private void handleTuningInput() {
        // SELECT VARIABLE (Up/Down)
        if (gamepad1.dpad_up && !lastUp) {
            selectionIndex--;
            if (selectionIndex < 0) selectionIndex = 3;
        }
        if (gamepad1.dpad_down && !lastDown) {
            selectionIndex++;
            if (selectionIndex > 3) selectionIndex = 0;
        }

        // ADJUST VALUE (Left/Right)
        // Multipliers determine how fast you change the number
        double adjustment = 0;
        if (gamepad1.dpad_right && !lastRight) adjustment = 1.0;
        if (gamepad1.dpad_left && !lastLeft) adjustment = -1.0;

        if (adjustment != 0) {
            switch (selectionIndex) {
                case 0: kP += adjustment * 0.001; break; // Step size for P
                case 1: kI += adjustment * 0.001; break; // Step size for I
                case 2: kD += adjustment * 0.0005; break; // Step size for D
                case 3: kF += adjustment * 0.005; break; // Step size for F
            }
        }

        // Update button states
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastRight = gamepad1.dpad_right;
        lastLeft = gamepad1.dpad_left;
    }
}