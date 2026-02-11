package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Disabled

@Autonomous(name = "02a Red Near From Goal Open Gate (Auto)", group = "00 Robot", preselectTeleOp = "01A Red Near From Goal (TeleOp)")

public class auto_02a_NearFromGoalOpenGate_Red extends auto_02_NearFromGoalOpenGate_base {
    @Override
    public void onInit() {
        Config.allianceColor = Config.AllianceColors.RED;
        Config.goalOption = Config.GoalOptions.NEAR;
        super.onInit();
    }
}