package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TestAutoOpMode", group = "Test")
public class TestAutoOpMode extends ActionOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        motorControl.init(hardwareMap);


        waitForStart();

        if (isStopRequested()) return;
    }
}
