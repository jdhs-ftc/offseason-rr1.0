package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * This class is used to control the motor systems on the robot.
 */
public class MotorControl {
    Claw claw;
    Slide slide;
    Arm arm;

    /**
     * Gets the current state of the arm and slide together.
     * @return the current state of the arm and slide system
     */
    public combinedMode getCurrentMode() {
        return currentMode;
    }

    /**
     * Sets the target state of the arm and slide together.
     * @param targetMode The new state to set the arm and slide to.
     */
    public void setCurrentMode(combinedMode targetMode) {
        currentMode = targetMode;
    }


    /**
     * These are the valid modes for the arm and slide system.
     */
    enum combinedMode {
        TOP,
        MIDDLE,
        BOTTOM
    }

    /**
     * The current state of the arm and slide system.
     */
    private combinedMode currentMode;
    private combinedMode oldMode;

    /**
     * This initializes the arm and slide motors, and resets the mode to the default. This should be run before any other methods.
     * @param hardwareMap The hardware map to use to get the motors.
     */
    public MotorControl(@NonNull HardwareMap hardwareMap) {
        arm = new Arm(hardwareMap);
        slide = new Slide(hardwareMap);
        claw = new Claw(hardwareMap);

        setCurrentMode(combinedMode.BOTTOM);
    }



    /**
     * This class updates the arm and slide motors to match the current state.
     */
    public void update() {
        if (getCurrentMode() != oldMode) {
        switch (getCurrentMode()) {
            case BOTTOM:
                if (arm.mode != armMode.DOWN) {
                    arm.mode = armMode.MOVING_DOWN;
                    slide.targetPosition = 0;
                }

                break;
            case MIDDLE:
                if (arm.mode != armMode.UP) {
                    arm.mode = armMode.MOVING_UP;
                }
                slide.targetPosition = 0;
                break;
            case TOP:
                if (arm.mode != armMode.UP) {
                    arm.mode = armMode.MOVING_UP;
                    slide.targetPosition = 1100;
                }

                break;

        } }
        oldMode = getCurrentMode();
        slide.update();
        arm.armUpdate();
    }


    public enum armMode {
        UP,
        MOVING_UP,
        MOVING_DOWN,
        DOWN
    }

    /**
     * Reset all motors.
     */
    public void reset() {
        arm.reset();
        slide.reset();
    }

    /**
     * This class controls the arm motor.
     */
    public static class Arm {
        public DcMotorEx motor;
        armMode mode;

        public Arm(@NonNull HardwareMap hardwareMap) {
            mode = armMode.DOWN;
            motor = hardwareMap.get(DcMotorEx.class, "arm");
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setCurrentAlert(4, CurrentUnit.AMPS);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        /**
         * This stops the arm, sets the state to down, and resets the encoder.
         */
        public void reset() {
            motor.setPower(0);
            mode = armMode.DOWN;
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }



        /**
         * This gets the current state of the arm control.
         * @return The current state of the arm motor.
         */
        public armMode getMode() {
            return mode;
        }


        /**
         * This immediately stops the arm motor and sets the state to the destination.
         */
        @SuppressWarnings("unused")
        public void armForceStop() {
            if (mode == armMode.MOVING_UP) {
                mode = armMode.UP;
            } else if (mode == armMode.MOVING_DOWN) {
                mode = armMode.DOWN;
            }

        }

        /**
         * This updates the arm motor to match the current state. This should be run in a loop.
         */
        public void armUpdate() {
            switch (mode) {
                case UP:
                    motor.setPower(0);
                    break;
                case DOWN:
                    //noinspection DuplicateBranchesInSwitch
                    motor.setPower(0);
                    break;
                case MOVING_UP:
                    if (motor.getCurrentPosition() >= 350) {
                        mode = armMode.UP;
                        motor.setPower(0);
                    } else {
                        motor.setPower(0.75);
                    }
                    break;
                case MOVING_DOWN:
                    if (motor.getCurrentPosition() <= -2) {
                        mode = armMode.DOWN;
                        motor.setPower(0);
                    } else {
                        motor.setPower(-0.25);
                    }

                    break;
            }
        }

        public void setMode(armMode newMode) {
            mode = newMode;
        }

        public boolean isBusy() {
            return mode == armMode.MOVING_DOWN || mode == armMode.MOVING_UP;
        }
    }

    /**
     * This class controls the slide motor.
     */
    public static class Slide {
        /**
         * This manually accesses the motor for the slide.
         */
        public DcMotorEx motor;
        double targetPosition;

        /**
         * This initializes the slide motor. This should be run before any other methods.
         * @param hardwareMap The hardware map to use to get the motors.
         */
        public Slide(HardwareMap hardwareMap) {
            motor = hardwareMap.get(DcMotorEx.class, "slide");
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setCurrentAlert(8, CurrentUnit.AMPS);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            
        }

        /**
         * This stops the slide, sets the state to down, sets the target to 0, and resets the encoder.
         */
        public void reset() {
            motor.setPower(0);
            targetPosition = 0;
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        /**
         * This updates the slide motor to match the current state. This should be run in a loop.
         */
        public void update() {
            // overly complex slide code
            // obtain the encoder position and calculate the error
            double slideError = targetPosition - motor.getCurrentPosition();
            motor.setTargetPosition((int) targetPosition);
            motor.setTargetPositionTolerance(10);
            if (slideError > 0) {
                motor.setPower(0.8);
            } else {
                motor.setPower(-0.5);
            }
            if (!motor.isOverCurrent()) {
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }
        }

        public void setPosition(double position) {
            targetPosition = position;
        }

        public boolean isBusy() {
            return motor.isBusy();
        }
    }

    /**
     * This class controls the claw.
     */
    public static class Claw {
        public CRServo servo;

        /**
         * This initializes the claw servo. This should be run before any other methods.
         * @param hardwareMap The hardware map to use to get the servo.
         */
        public Claw(HardwareMap hardwareMap) {
            servo = hardwareMap.get(CRServo.class, "claw");
            servo.setPower(0.8);
        }

        /**
         * This opens or closes the claw.
         * @param power The power to set the claw servo to.
         */
        public void setPower(double power) {
            servo.setPower(power);
        }
    }
}
