package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;
@Disabled
@TeleOp
public class FieldOrientedDrive extends LinearOpMode {
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;
    IMU imu;

    public void runOpMode(){
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(parameters);

        motorFL = hardwareMap.get(DcMotor.class, "FL");
        motorFR = hardwareMap.get(DcMotor.class, "FR");
        motorBL = hardwareMap.get(DcMotor.class, "BL");
        motorBR = hardwareMap.get(DcMotor.class, "BR");

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        Deadline gamepadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);

        waitForStart();

        while (opModeIsActive()) {

            double strafe = -gamepad1.left_stick_x;
            double drive = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;

            double max = Math.max(Math.abs(strafe) + Math.abs(drive) + Math.abs(turn), 1);

            double drivePower = 1 - (0.5 * gamepad1.right_trigger);

            if(gamepad1.left_bumper) imu.resetYaw();

            telemetry.addLine("Angulo do robô: "+ imu.getRobotYawPitchRollAngles().getYaw());

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double adjustedStrafe = drive * Math.sin(heading) + strafe * Math.cos(heading);
            double adjustedDrive = drive * Math.cos(heading) - strafe * Math.sin(heading);

            motorFL.setPower(((adjustedDrive + adjustedStrafe + turn) / max) * drivePower);
            motorBL.setPower(((adjustedDrive - adjustedStrafe + turn) / max) * drivePower);
            motorFR.setPower(((adjustedDrive - adjustedStrafe - turn) / max) * drivePower);
            motorBR.setPower(((adjustedDrive + adjustedStrafe - turn) / max) * drivePower);
            telemetry.update();
        }
    }
}
