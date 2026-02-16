package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Test LED Matrix")
public class LedTestOpMode extends OpMode { // Using OpMode instead of LinearOpMode
    private AIP1640Driver display;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        DigitalChannel din = hardwareMap.get(DigitalChannel.class, "dinPin");
        DigitalChannel sclk = hardwareMap.get(DigitalChannel.class, "clkPin");
        display = new AIP1640Driver(din, sclk);
    }


    @Override
    public void loop() {
        // --- OTHER ROBOT LOGIC GOES HERE ---
        // Example: driveTrain.setPower(gamepad1.left_stick_y);
        // This code will continue to run at 50-100Hz without stuttering.

        // Example: Display a countdown based on match time
        int secondsRemaining = (int) (30 - timer.seconds());

//        // Only update the physical hardware every 1 second to save CPU
//        if (timer.milliseconds() - lastUpdateTime > 700) {
//            if (secondsRemaining >= 0) {
//                display.displayNumber(secondsRemaining);
//            }
//            lastUpdateTime = timer.milliseconds();
//        }
//        display.displayColumnGroups(1);
        display.displayColumnGroups(secondsRemaining % 5);

        telemetry.addData("secondsRemaining", "%d", secondsRemaining);
        telemetry.update();


//        int testIndex = 0; // Increment this slowly in your loop
//        for (testIndex = testIndex;testIndex < 128;testIndex++){
//            display.scanner(testIndex);
//        }

//        display.debugPixel(0);
//        display.debugPixel(1);
    }
}