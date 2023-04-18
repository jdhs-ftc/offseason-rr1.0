package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorControlActions {
    private final MotorControl motorControl;
    public final Slide slide;
    public final Arm arm;

    public MotorControlActions(MotorControl motorControl) {
        this.motorControl = motorControl;
        this.slide = new Slide();
        this.arm = new Arm();
    }

    public Action setCurrentMode(MotorControl.combinedMode newMode) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket t) {
                motorControl.setCurrentMode(newMode);
                return true;
            }

            @Override
            public void preview(@NonNull Canvas canvas) {

            }
        };
    }
    public Action reset() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket t) {
                motorControl.reset();
                return true;
            }

            @Override
            public void preview(@NonNull Canvas canvas) {

            }
        };
    }

    public Action waitUntilFinished() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket t) {
                return !(motorControl.arm.isBusy() || motorControl.slide.isBusy());
            }

            @Override
            public void preview(@NonNull Canvas canvas) {

            }
        };
    }

    public class Arm {
        public Action reset() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    motorControl.arm.reset();
                    return true;
                }

                @Override
                public void preview(@NonNull Canvas canvas) {

                }
            };
        }

        public Action setMode(MotorControl.armMode newMode) {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    motorControl.arm.setMode(newMode);
                    return true;
                }

                @Override
                public void preview(@NonNull Canvas canvas) {

                }
            };
        }

        public Action waitUntilFinished() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    return !motorControl.arm.isBusy();
                }

                @Override
                public void preview(@NonNull Canvas canvas) {

                }
            };
        }
    }
    public class Slide {
        public Action reset() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    motorControl.slide.reset();
                    return true;
                }

                @Override
                public void preview(@NonNull Canvas canvas) {

                }
            };
        }

        public Action setPosition(double position) {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    motorControl.slide.setPosition(position);
                    return true;
                }

                @Override
                public void preview(@NonNull Canvas canvas) {

                }
            };
        }
        public Action waitUntilFinished() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    return !motorControl.slide.isBusy();
                }

                @Override
                public void preview(@NonNull Canvas canvas) {

                }
            };
        }
    }
}
