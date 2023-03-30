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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
    // Use the same gains as SampleMecanumDrive's heading controller
    private final PIDFController.PIDCoefficients HEADING_PID = new PIDFController.PIDCoefficients(4.0, 0.0, 0.0);
    private final PIDFController headingController = new PIDFController(HEADING_PID);
    DcMotorEx slide;
    DcMotorEx arm;
    //ColorSensor color;
    double slideTargetPosition;
    double slideError;
    CRServo claw;
    double speed;
    double slidePeakCurrentAmps;
    /*
    OpenCvCamera camera;
    RedConeDetectionPipeline redConeDetectionPipeline;
    BlueConeDetectionPipeline blueConeDetectionPipeline;
    PoleDetectionPipeline poleDetectionPipeline;

     */
    Mode armMode;




    @Override
    public void runOpMode() throws InterruptedException {
        armMode = Mode.IDLE;

        PhotonCore.enable();
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);




        //Initialization Period
        // RoadRunner Init
        // Initialize SampleMecanumDrive
        MecanumDrive drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);


        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        //blue = PoseStorage.blue;

        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);

        /* Motor Init */

        // Initiate Slide
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setCurrentAlert(8, CurrentUnit.AMPS);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initiate Arm
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setCurrentAlert(4, CurrentUnit.AMPS);
        //color = hardwareMap.get(ColorSensor.class, "color");


        // Initiate Claw
        claw = hardwareMap.get(CRServo.class, "claw");
        claw.setPower(0.5);


        // Vision
        /*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        blueConeDetectionPipeline = new BlueConeDetectionPipeline();
        redConeDetectionPipeline = new RedConeDetectionPipeline();
        poleDetectionPipeline = new PoleDetectionPipeline();

        camera.setPipeline(poleDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

         */

        telemetry.setMsTransmissionInterval(50);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //FtcDashboard.getInstance().startCameraStream(camera, 15);



        /* Variable Init */
        slideTargetPosition = 0.0;
        speed = .8;
        slidePeakCurrentAmps = 0.0;


        waitForStart();

        if (isStopRequested()) return;


        //Run Period

        while (opModeIsActive() && !isStopRequested()) {
            double prevTime = System.nanoTime(); // set to current time
            // Update the controller input bounds
            if (gamepad1.left_bumper) {
                speed = .35;
            } else if (gamepad1.right_bumper) {
                speed = 1;
            } else {
                speed = .35;
            }


            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y * speed,
                    -gamepad1.left_stick_x * speed
            );
            Pose2d poseEstimate = drive.pose;
            double rotationAmount = input.angleCast().minus(poseEstimate.rot.plus(Math.toRadians(90.0)));
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
            if (armMode != Mode.MOVING_DOWN) {
                claw.setPower(0.5 + gamepad2.left_trigger - gamepad2.right_trigger + clawPowerAsDouble);
            } else {
                claw.setPower(0.9 + gamepad2.left_trigger - gamepad2.right_trigger + clawPowerAsDouble); //TODO UPDATE
            }

            //arm.setPower(gamepad2.right_stick_x * 0.5);




            //FieldcENTRIC
            if (gamepad1.dpad_down && gamepad1.dpad_left && gamepad1.dpad_up && gamepad1.dpad_right) {
                drive.pose = new Pose2d(drive.pose.trans.x, drive.pose.trans.y, Math.toRadians(90.0));
            }

            // Slide
            slideTargetPosition = slideTargetPosition + (-gamepad2.left_stick_y * 20);
            slideTargetPosition = slideTargetPosition + (-gamepad2.left_stick_y * 20);
            if (gamepad2.y || gamepad1.y) {
                slideTargetPosition = 1200;
                armMode = Mode.MOVING_UP;
            }
            if (gamepad2.b || gamepad1.b) {
                slideTargetPosition = 400;
                armMode = Mode.MOVING_UP;
            }
            if (gamepad2.a || gamepad1.a) {
                slideTargetPosition = 20;
                armMode = Mode.MOVING_DOWN;
            }
            if (gamepad2.x || gamepad1.x) {
                slideTargetPosition = 1200;
                armMode = Mode.MOVING_DOWN;
            }
            if (gamepad2.dpad_right && gamepad2.dpad_left) {
                armMode = Mode.IDLE;
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (slideTargetPosition > 1100) {
                slideTargetPosition = 1100;
            } else if (slideTargetPosition < 0) {
                slideTargetPosition = 0;
            }


            // overly complex slide code

            // obtain the encoder position and calculate the error
            slideError = slideTargetPosition - slide.getCurrentPosition();
            slide.setTargetPosition((int) slideTargetPosition);
            slide.setTargetPositionTolerance(10);
            if (slideError > 0) {
                slide.setPower(0.8);
            } else {
                slide.setPower(-0.8);
            }
            if (!slide.isOverCurrent() && !(gamepad2.left_stick_y > 0)) {
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide.setPower(gamepad1.left_stick_y);
                slideTargetPosition = slide.getCurrentPosition();
            }
            // update peak current if larger then previous
            if (slide.getCurrent(CurrentUnit.AMPS) > slidePeakCurrentAmps) {
                slidePeakCurrentAmps = slide.getCurrent(CurrentUnit.AMPS);
            }


            // arm
            /*
            armController.setTargetPosition(armTargetPosition);
            // make sure arm is not overcurrent
            if (!arm.isOverCurrent()) {
                arm.setPower(0);
                //arm.setPower(armController.update(arm.getCurrentPosition()));
            } else {
                arm.setPower(0);
            }
             */




            switch (armMode) {
                case IDLE:
                    arm.setPower(-gamepad2.right_stick_y * 0.5);
                    break;
                case MOVING_UP:
                    if (arm.getCurrentPosition() >= 350 || gamepad2.x || gamepad1.x) {
                        armMode = Mode.IDLE;
                        arm.setPower(0);
                    } else {
                        if (slide.getCurrentPosition() > 100 && !arm.isOverCurrent()) {

                            arm.setPower(0.75);
                            // compensate for arm gravity using the arm angle, weight, and length
                            //arm.setPower((0.75 * Math.cos(Math.toRadians(arm.getCurrentPosition() / 2.0))) + 0.75);
                        } else {
                            arm.setPower(0);
                        }
                    }
                    break;
                case MOVING_DOWN:
                    if (arm.getCurrentPosition() <= 5) {
                        armMode = Mode.IDLE;
                        arm.setPower(0);
                    } else {
                        arm.setPower(-0.25);
                    }
                    break;
            }
            // if right bumper on gamepad2 pressed, reset the slide encoder

            // color

            drive.updatePoseEstimateAndGetActualVel(); // this is technically private but the person who made rr says its the right way so /shrug







            // Timing
            // measure difference between current time and previous time
            double timeDifference = (System.nanoTime() - prevTime) / 1000.0;

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.trans.x);
            telemetry.addData("y", poseEstimate.trans.y);
            telemetry.addData("heading", poseEstimate.rot.log());
            telemetry.addData("armPosition", arm.getCurrentPosition());
            telemetry.addData("armCurrent", arm.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("slidePeakCurrent", slidePeakCurrentAmps);
            telemetry.addData("slideTargetPosition", slideTargetPosition);
            telemetry.addData("slidePosition", slide.getCurrentPosition());
            telemetry.addData("controllerHeading", controllerHeading.angleCast().log());
            telemetry.addData("Arm Mode", armMode);
            //telemetry.addData("colorBlue", color.blue());
            //telemetry.addData("colorRed", color.red());
            telemetry.addData("servoPosition", claw.getPower());
            //telemetry.addData("poleWidth", poleDetectionPipeline.getMaxRect().width);
            telemetry.addData("loopTime", 1/(timeDifference/1000));
            telemetry.update();
        }
    }

    enum Mode {
        MOVING_UP,
        MOVING_DOWN,
        IDLE
    }
}