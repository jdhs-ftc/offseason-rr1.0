package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * This class is used to control the motor systems on the robot.
 */
public class motorControl {

    /**
     * Gets the current state of the arm and slide together.
     * @return the current state of the arm and slide system
     */
    public static combinedMode getCurrentMode() {
        return currentMode;
    }

    /**
     * Sets the target state of the arm and slide together.
     * @param targetMode The new state to set the arm and slide to.
     */
    public static void setCurrentMode(combinedMode targetMode) {
        motorControl.currentMode = targetMode;
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
    private static combinedMode currentMode;
    private static combinedMode oldMode;

    /**
     * This initializes the arm and slide motors, and resets the mode to the default. This should be run before any other methods.
     * @param hardwareMap The hardware map to use to get the motors.
     */
    public static void init(@NonNull HardwareMap hardwareMap) {
        arm.init(hardwareMap);
        slide.init(hardwareMap);
        claw.init(hardwareMap);

        setCurrentMode(combinedMode.BOTTOM);
    }

    /**
     * Sets the target state of the arm and slide together.
     * @param newMode The new state to set the arm and slide to.
     */
    public static void setMode(combinedMode newMode) {
        setCurrentMode(newMode);
    }


    /**
     * This class updates the arm and slide motors to match the current state.
     */
    public static void update() {
        if (getCurrentMode() != oldMode) {
        switch (getCurrentMode()) {
            case BOTTOM:
                if (arm.mode != arm.armMode.DOWN) {
                    arm.mode = arm.armMode.MOVING_DOWN;
                    slide.targetPosition = 0;
                }

                break;
            case MIDDLE:
                if (arm.mode != arm.armMode.UP) {
                    arm.mode = arm.armMode.MOVING_UP;
                }
                slide.targetPosition = 0;
                break;
            case TOP:
                if (arm.mode != arm.armMode.UP) {
                    arm.mode = arm.armMode.MOVING_UP;
                    slide.targetPosition = 1100;
                }

                break;

        } }
        oldMode = getCurrentMode();
        slide.update();
        arm.armUpdate();
    }

    /**
     * This class controls the arm motor.
     */
    public static class arm {
        public static DcMotorEx motor;
        static armMode mode;

        /**
         * This initializes the arm motor. This should be run before any other methods.
         * @param hardwareMap The hardware map to use to get the motors.
         */
        public static void init(@NonNull HardwareMap hardwareMap) {
            mode = armMode.DOWN;
            motor = hardwareMap.get(DcMotorEx.class, "arm");
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setCurrentAlert(4, CurrentUnit.AMPS);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        /**
         * This gets the current state of the arm control.
         * @return The current state of the arm motor.
         */
        public static armMode getMode() {
            return mode;
        }

        /**
         * Manually override the power of the arm motor, if the state isn't precise enough for you.
         * @param power The power to set the arm motor to.
         */
        public static void setPower(double power) {
            motor.setPower(power);
        }

        /**
         * This immediately stops the arm motor and sets the state to the destination.
         */
        public static void armForceStop() {
            if (mode == armMode.MOVING_UP) {
                mode = armMode.UP;
            } else if (mode == armMode.MOVING_DOWN) {
                mode = armMode.DOWN;
            }

        }

        /**
         * This updates the arm controller for a new target state.
         *
         * @param newMode The new state to set the arm motor to.
         */
        public static void setMode(armMode newMode) {
            mode = newMode;
        }

        /**
         * This updates the arm motor to match the current state. This should be run in a loop.
         */
        public static void armUpdate() {
            switch (mode) {
                case UP:
                    motor.setPower(0);
                    break;
                case DOWN:
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

        /**
         * These are the valid modes for the arm motor.
         */
        enum armMode {
            UP,
            MOVING_UP,
            MOVING_DOWN,
            DOWN
        }
    }

    /**
     * This class controls the slide motor.
     */
    public static class slide {
        /**
         * This manually accesses the motor for the slide.
         */
        public static DcMotorEx motor;
        static double targetPosition;

        /**
         * This initializes the slide motor. This should be run before any other methods.
         * @param hardwareMap The hardware map to use to get the motors.
         */
        public static void init(@NonNull HardwareMap hardwareMap) {
            motor = hardwareMap.get(DcMotorEx.class, "slide");
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setCurrentAlert(8, CurrentUnit.AMPS);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            
        }


        /**
         * This gets the current goal of the slide control.
         * @return The current goal position of the slide motor.
         */
        public static double getTargetPosition() {
            return targetPosition;
        }

        /**
         * This sets the goal of the slide control.
         * @param newTarget The new goal position of the slide motor.
         */
        public static void setTargetPosition(double newTarget) {
            targetPosition = newTarget;
        }

        /**
         * This updates the slide motor to match the current state. This should be run in a loop.
         */
        public static void update() {
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
    }

    /**
     * This class controls the claw.
     */
    public static class claw {
        public static CRServo servo;

        /**
         * This initializes the claw servo. This should be run before any other methods.
         * @param hardwareMap The hardware map to use to get the servo.
         */
        public static void init(HardwareMap hardwareMap) {
            servo = hardwareMap.get(CRServo.class, "claw");
            servo.setPower(0.8);
        }

        /**
         * This opens or closes the claw.
         * @param power The power to set the claw servo to.
         */
        public static void setPower(double power) {
            servo.setPower(power);
        }
    }
}
