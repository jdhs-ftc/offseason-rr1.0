package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.PoleChatGPTPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@TeleOp
@Config
public class CenterConeOpMode extends LinearOpMode {

    private OpenCvCamera camera;
    private final PoleChatGPTPipeline pipeline = new PoleChatGPTPipeline();
    public static PIDFController.PIDCoefficients pidCoefficients = new PIDFController.PIDCoefficients(4, 0, 0);
    public static PIDFController pidController = new PIDFController(pidCoefficients); // Tuned PID values

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);

        // Initialize the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        // Start the camera
        camera.openCameraDeviceAsync(cameraListener);

        double targetX = 320;

        // Wait for the start button to be pressed
        waitForStart();


        // Run the robot code while the OpMode is active
        while (opModeIsActive()) {



            // Calculate the error between the target X-coordinate and the cone's X-coordinate
            // Target X-coordinate for center of the image


            double error = targetX - pipeline.getPoles().get(0).x;

            // Use the PID controller to calculate the output power for the motors
            double output = pidController.update(error);

            // Send the output power to the motors to center the cone
            // Note: This is just an example. You would need to modify this code to control your own robot.
            //       The example assumes you have two motors with names "leftMotor" and "rightMotor".
            drive.setDrivePowers(new Twist2d(
                    new Vector2d(
                            0,
                            0
                    )
                    , output
            ));

            telemetry.addData("Error", error);
            telemetry.addData("Output", output);
            telemetry.update();
        }

        // Stop the camera
        camera.stopStreaming();
    }

    @SuppressWarnings("AnonymousInnerClassWithTooManyMethods")
    private final OpenCvCamera.AsyncCameraOpenListener cameraListener = new OpenCvCamera.AsyncCameraOpenListener() {
        @Override
        public void onOpened() {
            // Set the pipeline for the camera
            camera.setPipeline(pipeline);
            camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
        }

        @Override
        public void onError(int errorCode) {
            // Handle camera error
        }
    };
}
