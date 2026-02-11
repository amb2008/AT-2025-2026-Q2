package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Before running this code, double-check your Driver Station Configuration
 *  - Servos: Name your servo axon (set type to "Continuous Rotation Servo")
 *  - Analog Input: Name your analog device encoder (connected to the port your wired).
 *
 *
 *  WHAT TO LOOK FOR WHEN TESTING
 *  1. Voltage Check:
 *   - SUCCESS: As the servo spins, the voltage should smoothly climb from 0V to 3.3V
 *              and then snap back down to 0V (or vice versa) as it completes a full rotation
 *   - FAILURE (Floating): If the voltage hovers around 0.0V or random low numbers and doesn't
 *              change when the servo moves, the signal wire likely isn't making contact with Pin 3/4
 *
 *   2. The "Wrap Around":
 *   - An Axon is an Absolute Encoder, there is a specific physical point where the value
 *     jumps from 359 degrees (3.3V) back to 0 degrees (0V). Watch for this crossover point
 *     IT IS CRUCIAL FOR PROGRAMMING PID LATER
 */

@TeleOp(name = "Axon Encoder Test", group = "Test")
public class AxonEncoderTest extends OpMode {

    // Define hardware variables
    private CRServo turret;
    private AnalogInput axonEncoder;
    private static final double MAX_VOLTAGE = 3.3; // Check your specific hub, usually 3.3V
    private static final double GEAR_RATIO = 5.0;  // 5:1 Reduction
    private double lastVoltage = 0;
    private int rotationCount = 0;
    // Safety Limits (Degrees)
    private static final double MAX_TURRET_ANGLE = 180;
    private static final double MIN_TURRET_ANGLE = -180;


    @Override
    public void init() {
        // Initialize the hardware variables using the names from your config
        turret = hardwareMap.get(CRServo.class, "turret");
        axonEncoder = hardwareMap.get(AnalogInput.class, "encoder");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // CONTROL: Use Gamepad Triggers to spin the servo
        // Right Trigger spins forward, Left Trigger spins backward
        double power = gamepad1.right_trigger - gamepad1.left_trigger;
        turret.setPower(power);
//
//        // SENSING: Read the analog voltage
//        // The Axon absolute encoder outputs 0V to 3.3V
//        double voltage = axonEncoder.getVoltage();
//
//        // MATH: Convert voltage to degrees (0 - 360)
//        // 3.3V is the max range of the encoder
//        double degrees = (voltage / 3.3) * 360;
//

        double angle = getTurretAngle();
        // FEEDBACK: Show data on the Driver Station screen
    }

    public double getTurretAngle() {
        double currentVoltage = axonEncoder.getVoltage();

        // Calculate the raw angle of the axonServo itself (0-360)
        double currentServoAngle = (currentVoltage / MAX_VOLTAGE) * 360.0;

        // Detect Wraparound
        // Threshold: If jump is > 50% of range (1.65V), it's a wrap
        double threshold = MAX_VOLTAGE / 2.0;

        if (currentVoltage - lastVoltage < -threshold) {
            // Jumped from High to Low (CW rotation completed)
            rotationCount++;
        } else if (currentVoltage - lastVoltage > threshold) {
            // Jumped from Low to High (CCW rotation completed)
            rotationCount--;
        }

        lastVoltage = currentVoltage;

        // Math: Total Servo Degrees / Gear Ratio
        double totalServoDegrees = (rotationCount * 360.0) + currentServoAngle;
        double turretAngle = totalServoDegrees / GEAR_RATIO;

        telemetry.addData("Angle", turretAngle);
        telemetry.addData("Total Servo Degrees", totalServoDegrees);
        telemetry.addData("Encoder Voltage", "%.3f V", currentVoltage);
        telemetry.addData("Last Voltage", "%.1f deg", lastVoltage);
        telemetry.addData("Rotation count", rotationCount);

        telemetry.update();

        return turretAngle;
    }

    public void setPower(double power) {
        double currentAngle = getTurretAngle();

        // SOFT LIMITS: Prevent cable snapping
        if (power > 0 && currentAngle >= MAX_TURRET_ANGLE) {
            turret.setPower(0); // Stop if trying to go past +180
        } else if (power < 0 && currentAngle <= MIN_TURRET_ANGLE) {
            turret.setPower(0); // Stop if trying to go past -180
        } else {
            turret.setPower(power);
        }
    }
}