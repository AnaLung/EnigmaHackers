package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Autonom")
public class Rick_Autonom_Dezinfectare extends LinearOpMode {

    Rick_HardwareMap robot = new Rick_HardwareMap();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     DRIVE_SPEED             = 0.4;
    static final double     TURN_SPEED              = 0.3;

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);

        robot.LeftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LeftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.LeftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LeftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        robot.LeftBrush.setPower(0.8);
        robot.RightBrush.setPower(0.8);

        ///drum

        robot.LeftBrush.setPower(0);
        robot.RightBrush.setPower(0);

    }

    public void encoderDrive(double speed, double left, double right)
    {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        if(opModeIsActive())
        {
            newLeftFrontTarget = robot.LeftFrontMotor.getCurrentPosition() + (int)(left * COUNTS_PER_INCH);
            newLeftBackTarget = robot.LeftBackMotor.getCurrentPosition() + (int)(left * COUNTS_PER_INCH);
            newRightFrontTarget = robot.RightFrontMotor.getCurrentPosition() + (int)(right * COUNTS_PER_INCH);
            newRightBackTarget = robot.RightBackMotor.getCurrentPosition() + (int)(right * COUNTS_PER_INCH);

            robot.LeftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.LeftBackMotor.setTargetPosition(newLeftBackTarget);
            robot.RightFrontMotor.setTargetPosition(newRightFrontTarget);
            robot.RightBackMotor.setTargetPosition(newRightBackTarget);

            robot.LeftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.LeftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.LeftFrontMotor.setPower(Math.abs(speed));
            robot.LeftBackMotor.setPower(Math.abs(speed));
            robot.RightFrontMotor.setPower(Math.abs(speed));
            robot.RightBackMotor.setPower(Math.abs(speed));

            robot.LeftFrontMotor.setPower(0);
            robot.LeftBackMotor.setPower(0);
            robot.RightFrontMotor.setPower(0);
            robot.RightBackMotor.setPower(0);

            robot.LeftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.LeftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

}
