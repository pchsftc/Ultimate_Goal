package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.Locale;

//All importation are copied from lowercaseFrankTeleOp.java

@TeleOp(name="FrankV2", group="TeleOp")

public class FrankV2 extends OpMode {

    private HardwarePushbot frank = new HardwarePushbot();
    Orientation angles;

    @Override
    public void init() {

        frank.init(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
       // frank.imu.initialize(parameters);
       // composeTelemetry();

        frank.leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frank.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frank.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frank.rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop() {
        updateDrive();
      //  composeTelemetry();
    }

    private void updateDrive() {

        // https://gm0.org/en/stable/docs/software/mecanum-drive.html

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.5;
        double rx = gamepad1.right_stick_x;

        frank.leftFrontDrive.setPower(y + x + rx);
        frank.leftBackDrive.setPower(y - x + rx);
        frank.rightFrontDrive.setPower(y - x - rx);
        frank.rightBackDrive.setPower(y + x - rx);
    }


//    void composeTelemetry() {
//
//        // At the beginning of each telemetry update, grab a bunch of data
//        // from the IMU that we will then display in separate lines.
//        telemetry.addAction(new Runnable() { @Override public void run()
//        {
//            // Acquiring the angles is relatively expensive; we don't want
//            // to do that in each of the three items that need that info, as that's
//            // three times the necessary expense.
//             angles  = frank.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        }
//        });
//
//        telemetry.addLine()
//                .addData("Z angle", new Func<String>()  {
//                    @Override public String value() {
//                        return formatAngle(angles.firstAngle);
//
//                    }
//                })
//                .addData("Y angle", new Func<String>() {
//                    @Override public String value() {
//                        return formatAngle(angles.secondAngle);
//                    }
//                })
//                .addData("X angle", new Func<String>() {
//                    @Override public String value() {
//                        return formatAngle(angles.thirdAngle);
//                    }
//                });
//
//    }

    String formatAngle( double angle) {
        return formatDegrees(angle);
    }



    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
