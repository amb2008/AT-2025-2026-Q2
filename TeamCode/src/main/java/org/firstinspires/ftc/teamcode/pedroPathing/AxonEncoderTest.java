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
    private CRServo axonServo;
    private AnalogInput axonEncoder;

    @Override
    public void init() {
        // Initialize the hardware variables using the names from your config
        axonServo = hardwareMap.get(CRServo.class, "axon");
        axonEncoder = hardwareMap.get(AnalogInput.class, "encoder");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // CONTROL: Use Gamepad Triggers to spin the servo
        // Right Trigger spins forward, Left Trigger spins backward
        double power = gamepad1.right_trigger - gamepad1.left_trigger;
        axonServo.setPower(power);

        // SENSING: Read the analog voltage
        // The Axon absolute encoder outputs 0V to 3.3V
        double voltage = axonEncoder.getVoltage();

        // MATH: Convert voltage to degrees (0 - 360)
        // 3.3V is the max range of the encoder
        double degrees = (voltage / 3.3) * 360;

        // FEEDBACK: Show data on the Driver Station screen
        telemetry.addData("Servo Power", "%.2f", power);
        telemetry.addData("Encoder Voltage", "%.3f V", voltage);
        telemetry.addData("Estimated Degrees", "%.1f deg", degrees);

        // This helps you see the "wrap around" point
        telemetry.addData("Raw Max Voltage", "%.1f", axonEncoder.getMaxVoltage());

        telemetry.update();
    }
}