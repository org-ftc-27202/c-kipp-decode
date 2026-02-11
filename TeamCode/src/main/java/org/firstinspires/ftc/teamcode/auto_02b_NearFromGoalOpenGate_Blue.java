package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Disabled

@Autonomous(name = "02b Blue Near From Goal Open Gate (Auto)", group = "00 Robot", preselectTeleOp = "01B Blue Near From Goal (TeleOp)")

public class auto_02b_NearFromGoalOpenGate_Blue extends auto_02_NearFromGoalOpenGate_base {
    @Override
    public void onInit() {
        Config.allianceColor = Config.AllianceColors.BLUE;
        Config.goalOption = Config.GoalOptions.NEAR;
        super.onInit();
    }
}