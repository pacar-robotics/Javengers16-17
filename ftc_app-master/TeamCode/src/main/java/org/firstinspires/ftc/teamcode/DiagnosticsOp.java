package org.firstinspires.ftc.teamcode;

/**
 * Created by jorda on 9/24/2017.
 */

public class DiagnosticsOp extends vv_OpMode {
    private vv_Lib robotLibrary;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetryShow("Initializing...", false);
        robotLibrary = new vv_Lib(this);
    }


    // TODO: Create actual tests once common class methods are completed
    private boolean vuforiaDetection() {
        return true;
    }

    private boolean cubeArm() {
        return true;
    }

    private boolean cubeHand() {
        return true;
    }

    private boolean cubeHandGyro() {
        return true;
    }

    private boolean relicArm() {
        return true;
    }

    private boolean relicHand() {
        return true;
    }

    // Combine telemetryAddData and telemetryUpdate into one method for convenience
    private void telemetryShow(String s, boolean isInput) {
        telemetryAddData("DiagnosticsOp", (isInput ? "Input" : "Output"), s);
        telemetryUpdate();
    }
}
