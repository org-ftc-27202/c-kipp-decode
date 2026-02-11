package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Disabled

@Autonomous(name = "03a Red Far From Goal (Auto)", group = "00 Robot", preselectTeleOp = "01A Red Near From Goal (TeleOp)")

public class auto_03a_FarFromGoal_Red extends auto_03_FarFromGoal_base {
    @Override
    public void onInit() {
        Config.allianceColor = Config.AllianceColors.RED;
        Config.goalOption = Config.GoalOptions.FAR;
        super.onInit();
    }
}