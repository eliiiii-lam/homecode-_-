package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.mechs;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "ActionTester")
public class ActionTester extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, -63.25, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        mechs Mechs = new mechs(hardwareMap);

        Actions.runBlocking(new ParallelAction(
///////////////////////////////////////////////////////on init


                ///////////////////////////////////////////////////////on init
        ));
        waitForStart();
        Actions.runBlocking(new ParallelAction(
               Mechs.armDown()
        ));
        Actions.runBlocking(
                new SleepAction(3)
        );
    }

}