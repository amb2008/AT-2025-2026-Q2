package org.firstinspires.ftc.teamcode.pedroPathing;
import static org.firstinspires.ftc.teamcode.CONSTANTS.flicksDown;
import static org.firstinspires.ftc.teamcode.CONSTANTS.flicksUp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Objects;

@TeleOp(name = "Grant Popick Flicker Tuning", group = "Debug")
//@Disabled
public class GrantFlickerTesting extends LinearOpMode {
    private Servo flick1 = null;
    private Servo flick2 = null;
    private Servo flick3 = null;
    private boolean wackSet = false;
    private boolean aWasPressed = false;
    private long sleep1 = 500;
    private long sleep2 = 500;
    private DcMotorEx fwl = null;
    private DcMotorEx fwr = null;


    @Override
    public void runOpMode() {
        flick1 = hardwareMap.get(Servo.class, "flick1");
        flick2 = hardwareMap.get(Servo.class, "flick2");
        flick3 = hardwareMap.get(Servo.class, "flick3");
        fwl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fwr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fwl = hardwareMap.get(DcMotorEx.class, "fwl");
        fwr = hardwareMap.get(DcMotorEx.class, "fwr");
        fwl.setDirection(DcMotor.Direction.REVERSE);
        fwr.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            if (!wackSet){
                wackSet = true;
                flick1.setPosition(flicksDown[0]);
                flick2.setPosition(flicksDown[1]);
                flick3.setPosition(flicksDown[2]);
            }

            fwl.setPower(0.4);
            fwr.setPower(0.4);

            if (gamepad1.y && !aWasPressed) {
                aWasPressed = true;
                new Thread(()->{
                    Servo[] outPattern = {flick1, flick2, flick3};
                    outtake(outPattern);
                }).start();
            }
            if (gamepad1.b && !aWasPressed) {
                aWasPressed = true;
                new Thread(()->{
                    Servo[] outPattern = {flick2, flick1, flick3};
                    outtake(outPattern);
                }).start();
            }
            if (gamepad1.a && !aWasPressed) {
                aWasPressed = true;
                new Thread(()->{
                    Servo[] outPattern = {flick3, flick2, flick1};
                    outtake(outPattern);
                }).start();
            }

            if (gamepad1.right_trigger > 0.1 && !aWasPressed){
                aWasPressed = true;
                sleep1 += 20;
            }

            if (gamepad1.left_trigger > 0.1 && !aWasPressed){
                aWasPressed = true;
                sleep1 -= 20;
            }

            if (gamepad1.right_bumper && !aWasPressed){
                aWasPressed = true;
                sleep2 += 20;
            }

            if (gamepad1.left_bumper && !aWasPressed){
                aWasPressed = true;
                sleep2 -= 20;
            }


            if (!gamepad1.a && !gamepad1.b && !gamepad1.x && !(gamepad1.right_trigger > 0.1) && !(gamepad1.left_trigger > 0.1) && !gamepad1.left_bumper && !gamepad1.right_bumper){
                aWasPressed = false;
            }


            telemetry.addLine("\n=== Controls ===");
            telemetry.addLine("Gamepad 1 ABY: All three flickers in different orders");
            telemetry.addLine("Gamepad 1 Triggers adjust sleep 1 (sleep between flick set up and flick set down)");
            telemetry.addLine("Gamepad 1 Bumpers adjust sleep 2 (sleep between flick set down and next flick set up)");
            telemetry.addData("Sleep 1", sleep1);
            telemetry.addData("Sleep 2", sleep2);
            telemetry.update();
        }
    }

    private void outtake(Servo[] outPattern) {
        double counter = 0;
        for (Servo targetServo : outPattern) {
            if (counter>0){
                sleep(sleep2);
            }
            counter += 1;
            if (targetServo == flick1){
                targetServo.setPosition(flicksUp[0]);
            } else if (targetServo == flick2){
                targetServo.setPosition(flicksUp[1]);
            } else if (targetServo == flick3){
                targetServo.setPosition(flicksUp[2]);
            }
            sleep(sleep1);
            flick1.setPosition(flicksDown[0]);
            flick2.setPosition(flicksDown[1]);
            flick3.setPosition(flicksDown[2]);
        }
    }
}

