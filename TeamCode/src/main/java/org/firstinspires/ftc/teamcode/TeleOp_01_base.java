package org.firstinspires.ftc.teamcode;

import dev.nextftc.core.commands.conditionals.WGIfElseCommand;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class TeleOp_01_base extends NextFTCOpMode {
    public TeleOp_01_base() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Catapult.INSTANCE, Camera.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE);
    }
    private final ElapsedTime opModeTimer = new ElapsedTime();
    private boolean telemetryOnFlag;
    @Override
    public void onInit() {
        Config.activeOpMode = Config.opModeOptions.TELEOP;
        Config.isTeleOpStartButtonPressed = false;
        Camera.INSTANCE.mapCameraHardware(hardwareMap);
        Intake.INSTANCE.mapIntakeStopperHardware(hardwareMap);
        telemetryOnFlag = false;
    }
    private final MotorEx frontLeftMotor = new MotorEx("leftFront").brakeMode();
    private final MotorEx frontRightMotor = new MotorEx("rightFront").reversed().brakeMode();
    private final MotorEx backLeftMotor = new MotorEx("leftRear").brakeMode();
    private final MotorEx backRightMotor = new MotorEx("rightRear").reversed().brakeMode();
    @Override
    public void onStartButtonPressed() {
        opModeTimer.reset();
        Config.isTeleOpStartButtonPressed = true;
        DriverControlledCommand driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().map(y -> Config.isDriverControlled ? -y : Camera.INSTANCE.forwardPIDPower),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX().map(x -> Config.isDriverControlled ? x * 0.75 : Camera.INSTANCE.turnPIDPower));  // Scalar to reduce turn power
        driverControlled.schedule();

        // Telemetry
        Gamepads.gamepad1().y().whenBecomesTrue(
                new WGIfElseCommand(() -> telemetryOnFlag, new InstantCommand(() -> telemetryOnFlag = false),
                    new InstantCommand(() -> telemetryOnFlag = true)));

        // Intake
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(Intake.INSTANCE.Inwards);
        Gamepads.gamepad1().leftTrigger().greaterThan(0.2).whenBecomesTrue(Intake.INSTANCE.Outwards);
        Gamepads.gamepad1().leftTrigger().lessThan(0.2).whenBecomesTrue(Intake.INSTANCE.Stop);
        Gamepads.gamepad1().dpadRight().whenBecomesTrue(
                new ParallelGroup(
                        Intake.INSTANCE.Stop,
                        Intake.INSTANCE.wiperToLaunchPosition));

        // Catapults
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(
                new SequentialGroup(
                        new ParallelGroup(Intake.INSTANCE.Stop,
                                Intake.INSTANCE.initIntakeStopper,
                                new SequentialGroup(
                                        Camera.INSTANCE.alignToGoal_byTurn,
                                        Camera.INSTANCE.alignToGoal_byForward)),
                        Catapult.INSTANCE.LaunchInParallel
                ));
        Gamepads.gamepad1().rightTrigger().greaterThan(0.2).whenBecomesTrue(
                new SequentialGroup(
                        new ParallelGroup(Intake.INSTANCE.Stop,
                                Intake.INSTANCE.initIntakeStopper,
                                Camera.INSTANCE.getCatapultArtifactColors,
                                new SequentialGroup(
                                        Camera.INSTANCE.alignToGoal_byTurn,
                                        Camera.INSTANCE.alignToGoal_byForward)),
                        Catapult.INSTANCE.LaunchByPattern
                ));

        // Reset
        Gamepads.gamepad1().dpadDown().not().and(Gamepads.gamepad1().x()).whenBecomesTrue(
                new ParallelGroup(
                        Intake.INSTANCE.Stop,
                        Intake.INSTANCE.initIntakeStopper,
                        new InstantCommand(() -> Config.isDriverControlled = true)));}
    @Override
    public void onUpdate() {
        Intake.INSTANCE.CountBalls();

        if (telemetryOnFlag) {
            telemetry.addData("run #", 1);
            telemetry.addData("alliance", Config.allianceColor.toString());
            telemetry.addData("pattern", Config.motifPattern.toString());
            telemetry.addData("intake (power)", "%.0f", Intake.INSTANCE.getPower());
            telemetry.addData("balls", "%d", Intake.INSTANCE.ballCounter);
            telemetry.addData("catapults (pos)", "01: %.0f | 02: %.0f | 03: %.0f", Catapult.INSTANCE.getPosition01(), Catapult.INSTANCE.getPosition02(), Catapult.INSTANCE.getPosition03());
            telemetry.addData("catapults (pattern)", "%s%s%s", Config.catapult01Color.toString().charAt(0), Config.catapult02Color.toString().charAt(0), Config.catapult03Color.toString().charAt(0));
            telemetry.addData("camera left", "x: %d | y: %d", Config.cameraLeftTagX, Config.cameraLeftTagY);
            telemetry.addData("camera right", "x: %d | y: %d", Config.cameraRightTagX, Config.cameraRightTagY);
            telemetry.addData("goal", "cX: %d | cY: %d", Camera.INSTANCE.deltaToCenterX, Camera.INSTANCE.deltaToCenterY);
            telemetry.addData("Timer", "%.1f", opModeTimer.seconds());

            telemetry.addLine("--- Buttons ---");
            telemetry.addLine("Intake: leftBumper=In/Off leftTrigger=Out/Off");
            telemetry.addLine("Launch: rightBumper=Parallel rightTrigger=Pattern");
            telemetry.addLine("Auto-Drive: x=Cancel Auto-Drive and Ball Count to 0");
            telemetry.addLine("dpadRight=Wiper to Launch");
            telemetry.update();
        }
    }
}
