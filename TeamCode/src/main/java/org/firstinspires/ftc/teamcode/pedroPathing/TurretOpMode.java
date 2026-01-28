package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Hybrid Turret Control", group = "TeleOp")
public class TurretOpMode extends LinearOpMode {
    private Limelight3A limelight;
    private CRServo turretServo;
    private boolean dpadPressed = false;

    // --- TUNING ---
    // In CR mode, kP is the "Speed Multiplier" for tracking.
    // Start low (0.01) and increase until it's snappy but not oscillating.
    double kP = 0.04; //Example that you may need to tune by extremely minor adjustments
    /**
     * Tuning the "Deadzone"
     * Because the 5:1 gear reduction is so precies, the Limelight might see the target at 0.1 degrees and try
     * to move. In a continuous setup, this can cause a "micro-oscillation" where th turret shakes
     * back and forth.
     *  - If it shakes, increase visionDeadzone to 1.0
     *  - If it's too slow to react, increase kP
     */
    double visionDeadzone = 0.5; // Degrees of error to ignore (prevents jitter)

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretServo = hardwareMap.get(CRServo.class, "turret");

        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(1);
        limelight.start();

        telemetry.addData("Status", "CR Turret Initialized");
        telemetry.update();

        waitForStart();

        /**
         * Priority Breakdown for Hybrid Mode
         * 1. Manual Override (High Priority): As soon as the driver moves the right stick beyond the 10% deadzone (feel free to change),
         * the code enters the first block and ignores everything else. Essentially, the driver can "wrestle" control away from the Limelight.
         *
         * 2. Vision Tracking (Medium Priority): If the joystick is centered (no manual input), the code moves to the else if. If the Limelight sees
         * a valid target, it takes over and steers the Axon/turret.
         *
         * 3. Idle/Stop (Low Priority): If the stick is still AND the Limelight sees nothing, the motor power is set to 0 - You may want to change the logic
         * for this state
         */

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            double stickInput = -gamepad1.right_stick_x;
            double motorPower = 0;

            // 1. PRIORITY: Manual Override (Right Stick)
            if (gamepad1.right_bumper && !dpadPressed){
                dpadPressed = true;
                kP += 0.001;
            } else if (gamepad1.left_bumper && !dpadPressed){
                dpadPressed = true;
                kP -= 0.001;
            } else {
                dpadPressed = false;
            }
            if (Math.abs(stickInput) > 0.1) {
                motorPower = stickInput;
                telemetry.addData("Mode", "Manual Control");
            }
            // 2. SECONDARY: Limelight Tracking
            else if (result != null && result.isValid()) {
                double tx = result.getTx();

                if (Math.abs(tx) > visionDeadzone) {
                    // tx is degrees away from center.
                    // We multiply by kP to get a power between -1 and 1.
                    motorPower = tx * kP;
                    telemetry.addData("Mode", "Vision Tracking");
                } else {
                    motorPower = 0; // Target is centered
                    telemetry.addData("Mode", "LOCKED");
                }
            }
            // 3. IDLE: If no input and no target, stop.
            else {
                motorPower = 0;
                telemetry.addData("Mode", "Searching/Idle");
            }

            // Apply power (limited to prevent the 5:1 reduction from being too violent) - tweak as you see fit
            turretServo.setPower(Range.clip(motorPower, -0.8, 0.8));

            telemetry.addData("kP", kP);
            // Feedback
            telemetry.addData("Motor Power", motorPower);
            if(result != null) telemetry.addData("Target Offset (tx)", result.getTx());
            telemetry.update();
        }

        limelight.stop();
    }
}