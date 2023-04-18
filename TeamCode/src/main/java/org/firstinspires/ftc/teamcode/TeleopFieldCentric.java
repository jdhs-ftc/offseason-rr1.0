package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    private final PIDFController.PIDCoefficients HEADING_PID = new PIDFController.PIDCoefficients(4.0, 0.0, 0.0);
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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
                drive.pose = new Pose2d(drive.pose.trans.x, drive.pose.trans.y, Math.toRadians(90.0));
            }

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y * speed,
                    -gamepad1.left_stick_x * speed
            );
            Pose2d poseEstimate = drive.pose;
            double rotationAmount = input.angleCast().real - poseEstimate.rot.real + Math.toRadians(90.0);
            input = new Vector2d(input.x * Math.cos(rotationAmount) - input.y * Math.sin(rotationAmount), input.x * Math.sin(rotationAmount) + input.y * Math.cos(rotationAmount));

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            Vector2d controllerHeading = new Vector2d(-gamepad1.right_stick_y, -gamepad1.right_stick_x);

            if (controllerHeading.minus(new Vector2d(0.0,0.0)).norm() < 0.7) {
                drive.setDrivePowers(
                        new Twist2d(
                                new Vector2d(
                                        input.x,
                                        input.y
                                ),
                                (gamepad1.left_trigger - gamepad1.right_trigger)
                        )
                );
            } else {
                // Set the target heading for the heading controller to our desired angle


                headingController.targetPosition = controllerHeading.angleCast().log() + Math.toRadians(90.0);


                // Set desired angular velocity to the heading controller output + angular
                // velocity feedforward
                double headingInput = (headingController.update(poseEstimate.rot.log())
                        * MecanumDrive.kV)
                        * MecanumDrive.TRACK_WIDTH_TICKS;
                drive.setDrivePowers(
                        new Twist2d(
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
                double headingInput = (headingController.update(poseEstimate.rot.log())
                        * MecanumDrive.kV)
                        * MecanumDrive.TRACK_WIDTH_TICKS;
                drive.setDrivePowers(
                        new Twist2d(
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

            drive.updatePoseEstimateAndGetActualVel(); // this is technically private but the person who made rr says its the right way so /shrug
            // Timing
            // measure difference between current time and previous time
            double timeDifference = (System.nanoTime() - prevTime) / 1000.0;

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.trans.x);
            telemetry.addData("y", poseEstimate.trans.y);
            telemetry.addData("heading", poseEstimate.rot.log());
            telemetry.addData("armPosition", motorControl.arm.motor.getCurrentPosition());
            telemetry.addData("armCurrent", motorControl.arm.motor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("slideTargetPosition", motorControl.slide.targetPosition);
            telemetry.addData("slidePosition", motorControl.slide.motor.getCurrentPosition());
            telemetry.addData("controllerHeading", controllerHeading.angleCast().log());
            telemetry.addData("Arm Mode", motorControl.arm.getMode());
            telemetry.addData("servoPosition", motorControl.claw.servo.getPower());
            telemetry.addData("loopTime", 1/(timeDifference/1000));
            telemetry.update();
        }
    }
}