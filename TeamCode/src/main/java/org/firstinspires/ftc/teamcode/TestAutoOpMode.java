package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TestAutoOpMode", group = "Test")
public class TestAutoOpMode extends ActionOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        MotorControl motorControl = new MotorControl(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Action traj = drive.actionBuilder(drive.pose)
                .stopAndAdd(new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        motorControl.setCurrentMode(MotorControl.combinedMode.BOTTOM);
                        return true;
                    }
                    @Override
                    public void preview(@NonNull Canvas canvas) {}
                })
                .splineTo(new Vector2d(30, 30), Math.PI / 2)
                .splineTo(new Vector2d(60, 0), Math.PI)
                .build();


        waitForStart();

        if (isStopRequested()) return;

        runBlockingWithMotors(traj, motorControl);


        PoseStorage.currentPose = drive.pose;
    }
}
