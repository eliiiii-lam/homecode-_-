package org.firstinspires.ftc.teamcode.PID;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class slides_kooky extends OpMode {
    private PIDController controller1;
    public static double p1 = 0.008, i1 = 0, d1 = 0;
    public static double f1 = 0.2;
    public static int target1;

    private final double ticks_in_degrees = (1425.1/360.0) / 2; //change the 360 back to 180 if no work

    private DcMotorEx uppies ;

    @Override
    public void init(){

        controller1 = new PIDController(p1,i1,d1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        uppies = hardwareMap.get(DcMotorEx.class, "uppies");


    }


    @Override
    public void loop(){

        controller1.setPID(p1,i1,d1);
        int armPos1 = uppies.getCurrentPosition();
        double pid1 = controller1.calculate(armPos1, target1);
        double ff1 = Math.cos(Math.toRadians(target1/ticks_in_degrees)) * f1;

        double power1 = pid1 + ff1;



        uppies.setPower(power1); //setting the motor to the desired position

        telemetry.addData("pos ", armPos1);
        telemetry.addData("target ", target1);
        telemetry.update();


    }
}