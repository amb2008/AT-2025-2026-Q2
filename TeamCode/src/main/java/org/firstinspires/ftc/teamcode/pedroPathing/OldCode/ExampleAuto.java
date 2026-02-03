package org.firstinspires.ftc.teamcode.pedroPathing.OldCode; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Example Auto IF", group = "Examples")
@Disabled
public class ExampleAuto extends LinearOpMode {

    private Follower follower;

    private final Pose startPose = new Pose(38, 138, Math.toRadians(270));
    private final Pose scorePose = new Pose(60, 85, Math.toRadians(325));
    private final Pose pickup1Pose = new Pose(40, 84, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(40, 60, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(40, 36, Math.toRadians(180));

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        buildPaths();

        follower.setStartingPose(startPose);

        waitForStart();

        // --------- STEP 1: SCORE PRELOAD ----------
        follower.followPath(scorePreload);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        sleep(1000);


        // --------- STEP 2: GRAB PICKUP 1 ----------
        follower.followPath(grabPickup1, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        sleep(1000);


        // --------- STEP 3: SCORE PICKUP 1 ----------
        follower.followPath(scorePickup1, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        sleep(1000);


        // --------- STEP 4: GRAB PICKUP 2 ----------
        follower.followPath(grabPickup2, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        sleep(1000);


        // --------- STEP 5: SCORE PICKUP 2 ----------
        follower.followPath(scorePickup2, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        sleep(1000);


        // --------- STEP 6: GRAB PICKUP 3 ----------
        follower.followPath(grabPickup3, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        sleep(1000);


        // --------- STEP 7: SCORE PICKUP 3 ----------
        follower.followPath(scorePickup3, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }

        // Done
    }
}
