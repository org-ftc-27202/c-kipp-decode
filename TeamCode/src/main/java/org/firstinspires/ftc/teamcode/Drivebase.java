package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.core.subsystems.Subsystem;

public class Drivebase implements Subsystem {
    public static final Drivebase INSTANCE = new Drivebase();
    private Drivebase() { }

    private MecanumDrive autoDrive;
    public void mapDrivebaseHardware(HardwareMap hardwareMap) {
        autoDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }
}