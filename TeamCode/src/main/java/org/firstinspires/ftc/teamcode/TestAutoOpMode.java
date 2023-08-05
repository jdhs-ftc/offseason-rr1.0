package org.firstinspires.ftc.teamcode;



import static org.firstinspires.ftc.teamcode.MotorControl.combinedMode.*;
import static org.firstinspires.ftc.teamcode.MotorControl.armMode.*;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.MotorControlActions.*;
@Autonomous(name = "TestAutoOpMode", group = "Test")
public class TestAutoOpMode extends ActionOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        MotorControl motorControl = new MotorControl(hardwareMap);
        MotorControlActions motorControlActions = new MotorControlActions(motorControl);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Action traj =
                drive.actionBuilder(drive.pose)
                .stopAndAdd(motorControlActions.setCurrentMode(BOTTOM))
                .splineTo(new Vector2d(0, 30), Math.PI / 2)
                .stopAndAdd(motorControlActions.arm.setMode(UP))
                .splineTo(new Vector2d(0, 60), Math.PI)
                .stopAndAdd(motorControlActions.arm.setMode(DOWN))
                .stopAndAdd(motorControlActions.waitUntilFinished())
                .build();


        waitForStart();

        if (isStopRequested()) return;

        runBlocking(new RaceParallelCommand(
                traj,
                motorControlActions.update()
        ));



        PoseStorage.currentPose = drive.pose;
    }

}
