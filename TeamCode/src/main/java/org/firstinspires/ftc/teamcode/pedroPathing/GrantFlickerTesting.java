package org.firstinspires.ftc.teamcode.pedroPathing;
import static org.firstinspires.ftc.teamcode.CONSTANTS.flicksDown;
import static org.firstinspires.ftc.teamcode.CONSTANTS.flicksUp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Grant Popick Flicker Tuning", group = "Debug")
//@Disabled
public class GrantFlickerTesting extends LinearOpMode {
    private Servo flick1 = null;
    private Servo flick2 = null;
    private Servo flick3 = null;
    private boolean wackSet = false;
    private boolean aWasPressed = false;
    private boolean up = false;

    @Override
    public void runOpMode() {
        flick1 = hardwareMap.get(Servo.class, "flick1");
        flick2 = hardwareMap.get(Servo.class, "flick2");
        flick3 = hardwareMap.get(Servo.class, "flick3");

        waitForStart();

        while (opModeIsActive()) {
            if (!wackSet){
                wackSet = true;
                flick1.setPosition(flicksDown[0]);
                flick2.setPosition(flicksDown[1]);
                flick3.setPosition(flicksDown[2]);
            }

            if (gamepad2.a && !aWasPressed){
                aWasPressed = true;
                if (!up){
                    flick1.setPosition(flicksDown[0]);
                    up=true;
                } else {
                    up = false;
                    flick1.setPosition(flicksUp[0]);
                }
            }

            if (gamepad2.b && !aWasPressed){
                aWasPressed = true;
                if (!up){
                    flick2.setPosition(flicksDown[1]);
                    up=true;
                } else {
                    up = false;
                    flick2.setPosition(flicksUp[1]);
                }
            }

            if (gamepad2.x && !aWasPressed){
                aWasPressed = true;
                if (!up){
                    flick3.setPosition(flicksDown[2]);
                    up=true;
                } else {
                    up = false;
                    flick3.setPosition(flicksUp[2]);
                }
            }
            if (!gamepad2.a && !gamepad2.b && !gamepad2.x){
                aWasPressed = false;
            }


            telemetry.addLine("\n=== Controls ===");
            telemetry.addLine("Gamepad 1 AXY: Flickers");
            telemetry.update();
        }
    }
}

