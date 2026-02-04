package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test", group = "Tests")
@Disabled
public class ServoTest extends LinearOpMode {

    // Assume a servo named "test_servo" is configured in the robot configuration
    private Servo testServo;
    private double servoPosition = 0.5; // Start at the middle position

    @Override
    public void runOpMode() {
        // Initialize the servo
        testServo = hardwareMap.get(Servo.class, "test_servo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            // Use gamepad 1's right and left bumpers to control the servo
            if (gamepad1.right_bumper) {
                servoPosition += 0.01;
                sleep(100);
            } else if (gamepad1.left_bumper) {
                servoPosition -= 0.01;
                sleep(100);
            }
            if (gamepad1.a){
                testServo.setPosition(0.02);
                servoPosition = 0.02;
            } else if (gamepad1.y) {
                testServo.setPosition(0.27);
                servoPosition = 0.27;

            }

            // Clamp the servo position to be between 0.0 and 1.0
            servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

            // Set the servo position
            testServo.setPosition(servoPosition);

            // Add telemetry to display the current servo position
            telemetry.addData("Servo Position", servoPosition);
            telemetry.update();
        }
    }
}
