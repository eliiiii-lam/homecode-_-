package org.firstinspires.ftc.teamcode.Auto;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class mechs {
    private Servo armL;
    private Servo armR;

    private PIDController controller;
    public static double p = 0.0096, i = 0.03, d = 0.000525;
    public static double f = 0.08;
    public static int target;

    private final double ticks_in_degrees = 1425.1/360.0; //change the 360 back to 180 if no work

    private DcMotorEx motor ;

    Servo link;
    Servo inY;
    Servo inX;
    Servo inClaw;
    Servo inmainPiv;
    Servo outY;
    Servo outRot;
    Servo outClaw;

    public mechs(HardwareMap hardwareMap) {
        armL = hardwareMap.get(Servo.class, "LA");
        armR = hardwareMap.get(Servo.class, "RA");


        controller = new PIDController(p,i,d);
        motor = hardwareMap.get(DcMotorEx.class, "motor");
    }

    public class armDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            controller.setPID(p,i,d);
            int armPos = motor.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target/ticks_in_degrees)) * f;

            double power = pid + ff;
////////////////////////////////////////////////////////////////////////////////////////////
            outY.setPosition(0.4);
            outRot.setPosition(0.27);
            target = -25;
////////////////////////////////////////////////////////////////////////////////////////////
            motor.setPower(power); //setting the motor to the desired position
            return false;
        }
    }
    public Action armDown() {
        return new mechs.armDown();
    }

    public class armUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            controller.setPID(p,i,d);
            int armPos = motor.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target/ticks_in_degrees)) * f;

            double power = pid + ff;
////////////////////////////////////////////////////////////////////////////////////////////
            outClaw.setPosition(0.72);
            target = -400;
            outY.setPosition(0.6);
            outRot.setPosition(0.95);
////////////////////////////////////////////////////////////////////////////////////////////
            motor.setPower(power); //setting the motor to the desired position
            return false;
        }
    }
    public Action armUp() {
        return new mechs.armUp();
    }

    public class slam implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            controller.setPID(p,i,d);
            int armPos = motor.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target/ticks_in_degrees)) * f;

            double power = pid + ff;
////////////////////////////////////////////////////////////////////////////////////////////
            outClaw.setPosition(0.72);
            target = -690; // changed
            outY.setPosition(0.63);
            outRot.setPosition(0.95);
////////////////////////////////////////////////////////////////////////////////////////////
            motor.setPower(power); //setting the motor to the desired position
            return false;
        }
    }
    public Action slam() {
        return new mechs.slam();
    }

}