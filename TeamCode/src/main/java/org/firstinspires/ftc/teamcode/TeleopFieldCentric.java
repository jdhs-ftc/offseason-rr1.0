package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MecanumDrive.*;
import static org.firstinspires.ftc.teamcode.MecanumDrive.IN_PER_TICK;

import android.provider.ContactsContract;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@TeleOp(name = "Teleop Field Centric")
@Config
public class TeleopFieldCentric extends LinearOpMode {

    // Declare a PIDF Controller to regulate heading
    private final PIDFController.PIDCoefficients HEADING_PID = new PIDFController.PIDCoefficients(0.5, 0.0, 0.0);
    private final PIDFController headingController = new PIDFController(HEADING_PID);
    double speed;


    @Override
    public void runOpMode() throws InterruptedException {

        //  Initialization Period

        // Enable Performance Optimization
        //PhotonCore.enable();

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);




        // RoadRunner Init
        MecanumDrive drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        headingController.setInputBounds(-Math.PI, Math.PI);

        // Motor Init
        MotorControl motorControl = new MotorControl(hardwareMap);

        // Telemetry Init
        telemetry.setMsTransmissionInterval(50);
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (isStopRequested()) return;


        // Run Period

        while (opModeIsActive() && !isStopRequested()) {
            double prevTime = System.nanoTime(); // set to current time

            // Update the speed
            if (gamepad1.left_bumper) {
                speed = .35;
            } else if (gamepad1.right_bumper) {
                speed = 1;
            } else {
                speed = .8;
            }

            if (gamepad1.dpad_down && gamepad1.dpad_left && gamepad1.dpad_up && gamepad1.dpad_right) {
                drive.pose = new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(90.0));
            }

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y * speed,
                    -gamepad1.left_stick_x * speed
            );
            Pose2d poseEstimate = drive.pose;
            //double rotationAmount = input.angleCast().real - poseEstimate.rot.real + Math.toRadians(90.0);
            double rotationAmount = -poseEstimate.heading.real + Math.toRadians(90.0);

            input = new Vector2d(input.x * Math.cos(rotationAmount) - input.y * Math.sin(rotationAmount), input.x * Math.sin(rotationAmount) + input.y * Math.cos(rotationAmount));

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            Vector2d controllerHeading = new Vector2d(-gamepad1.right_stick_y, -gamepad1.right_stick_x);

            if (controllerHeading.minus(new Vector2d(0.0,0.0)).norm() < 0.7) {
                drive.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(
                                        input.x,
                                        input.y
                                ),
                                (gamepad1.left_trigger - gamepad1.right_trigger)
                        )
                );
            } else {
                // Set the target heading for the heading controller to our desired angle


                headingController.targetPosition = controllerHeading.angleCast().log() + Math.toRadians(180);


                // Set desired angular velocity to the heading controller output + angular
                // velocity feedforward
                double headingInput = (headingController.update(poseEstimate.heading.log())
                        * kV
                        * TRACK_WIDTH_TICKS * IN_PER_TICK);
                drive.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(
                                        input.x,
                                        input.y
                                ),
                                headingInput
                        )
                );

            }
            if (gamepad1.right_stick_button) {
                //headingController.targetPosition = drive.getExternalHeading() + poleDetectionPipeline.getMaxRect().x

                // Set desired angular velocity to the heading controller output + angular
                // velocity feedforward
                double headingInput = (headingController.update(poseEstimate.heading.log())
                        * kV)
                        * TRACK_WIDTH_TICKS;
                drive.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(
                                        input.x,
                                        input.y
                                ),
                                headingInput
                        )
                );
            }
            double clawPowerAsDouble = 0.0;
            if (gamepad1.left_bumper) {
                clawPowerAsDouble = 1.0;
            }
            if (motorControl.arm.mode != MotorControl.armMode.MOVING_DOWN) {
                motorControl.claw.setPower(0.5 + gamepad2.left_trigger - gamepad2.right_trigger + clawPowerAsDouble);
            } else {
                motorControl.claw.setPower(0.9 + gamepad2.left_trigger - gamepad2.right_trigger + clawPowerAsDouble);
            }



            // Slide
            motorControl.slide.targetPosition = motorControl.slide.targetPosition + (-gamepad2.left_stick_y * 20);
            if (gamepad2.y || gamepad1.y) {
                motorControl.setCurrentMode(MotorControl.combinedMode.TOP);
            }
            if (gamepad2.b || gamepad1.b) {
                motorControl.slide.targetPosition = 400;
                motorControl.arm.mode = MotorControl.armMode.MOVING_UP;
            }
            if (gamepad2.a || gamepad1.a) {
                motorControl.setCurrentMode(MotorControl.combinedMode.BOTTOM);
            }
            if (gamepad2.x || gamepad1.x) {
                motorControl.setCurrentMode(MotorControl.combinedMode.MIDDLE);
            }
            if (gamepad2.dpad_right && gamepad2.dpad_left) {
                motorControl.reset();
            }


            gamepad1.rumble(PhotonCore.CONTROL_HUB.getCurrent(CurrentUnit.AMPS),PhotonCore.EXPANSION_HUB.getCurrent(CurrentUnit.AMPS),Gamepad.RUMBLE_DURATION_CONTINUOUS);

            drive.updatePoseEstimate();
            motorControl.update();
            // Timing
            // measure difference between current time and previous time
            double timeDifference = (System.nanoTime() - prevTime) / 1000000.0;

            TelemetryPacket packet = new TelemetryPacket();
            MecanumDrive.drawRobot(packet.fieldOverlay(), drive.pose); //new Pose2d(new Vector2d(IN_PER_TICK * drive.pose.trans.x,IN_PER_TICK * drive.pose.trans.y), drive.pose.rot)
            packet.put("slideTargetPosition", motorControl.slide.targetPosition);
            packet.put("slidePosition", motorControl.slide.motor.getCurrentPosition());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.position.x);
            telemetry.addData("y", poseEstimate.position.y);
            telemetry.addData("heading", poseEstimate.heading.log());
            telemetry.addData("armPosition", motorControl.arm.motor.getCurrentPosition());
            telemetry.addData("armCurrent", motorControl.arm.motor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("slideTargetPosition", motorControl.slide.targetPosition);
            telemetry.addData("slidePosition", motorControl.slide.motor.getCurrentPosition());
            telemetry.addData("controllerHeading", controllerHeading.angleCast().log());
            telemetry.addData("Arm Mode", motorControl.arm.getMode());
            telemetry.addData("servoPosition", motorControl.claw.servo.getPower());
            telemetry.addData("loopTimeMs", timeDifference);
            telemetry.addData("loopTimeHz", 1000.0 / timeDifference);
            telemetry.update();
        }
    }
}