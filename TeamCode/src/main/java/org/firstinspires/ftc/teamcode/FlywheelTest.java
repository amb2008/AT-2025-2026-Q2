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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static org.firstinspires.ftc.teamcode.CONSTANTS.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.CONSTANTS.*;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name="FlywheelTest", group="Linear OpMode")
@Disabled
public class FlywheelTest extends LinearOpMode {
    private DcMotorEx fwl = null;
    private DcMotorEx fwr = null;

    double fwlSpeed = 0;
    double fwrSpeed = 0;
    private Servo sorting1 = null;
    private Servo sorting2 = null;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        fwl = hardwareMap.get(DcMotorEx.class, "fwl");
        fwr = hardwareMap.get(DcMotorEx.class, "fwr");

        fwl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fwr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fwl.setDirection(DcMotor.Direction.FORWARD);
        fwr.setDirection(DcMotor.Direction.REVERSE);

        sorting1 = hardwareMap.get(Servo.class, "sorting1");
        sorting2 = hardwareMap.get(Servo.class, "sorting2");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        // Choose a max velocity (depends on motor/encoder)
        double maxVelocity = 60000; // adjust to your motor's spec
        double step = 50;          // ticks/sec change per nudge
        double susanPos = 0;

        while (opModeIsActive()) {
            // Adjust left flywheel speed
            if (gamepad1.left_stick_y < -0.5) {  // push up
                fwlSpeed = Math.min(fwlSpeed + step, maxVelocity);
            } else if (gamepad1.left_stick_y > 0.5) {  // push down
                fwlSpeed = Math.max(fwlSpeed - step, 0);
            }

            // Adjust right flywheel speed
            if (gamepad1.right_stick_y < -0.5) {  // push up
                fwrSpeed = Math.min(fwrSpeed + step, maxVelocity);
            } else if (gamepad1.right_stick_y > 0.5) {  // push down
                fwrSpeed = Math.max(fwrSpeed - step, 0);
            }

            // Apply velocities
            fwl.setPower(fwlSpeed);
            fwr.setPower(fwrSpeed);

            double kP = 0.0006;
            double kF = 0.0001;
            double currVelo = (fwl.getVelocity() + fwr.getVelocity()) / 2.0;
            double error = fwSpeed - currVelo;
            double output = Math.max(kP * error + kF * fwSpeed, 0.3);
            fwl.setPower(output);
            fwr.setPower(output);

            // Telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("susan pos", susanPos);
            telemetry.addData("Flywheel Left Speed", "%.2f ticks/sec", fwlSpeed);
            telemetry.addData("Flywheel Right Speed", "%.2f ticks/sec", fwrSpeed);
            telemetry.addData("Flywheel Left (actual)", "%.2f ticks/sec", fwl.getVelocity());
            telemetry.addData("Flywheel Right (actual)", "%.2f ticks/sec", fwr.getVelocity());
            telemetry.update();

            sleep(150); // prevent huge jumps by slowing adjustment rate
        }
    }
}

