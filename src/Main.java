import org.jbox2d.testbed.framework.*;
import org.jbox2d.testbed.framework.j2d.TestPanelJ2D;

import javax.swing.*;

public class Main{

    public static void main(String[] args){
        TestbedModel model = new TestbedModel();

        model.addCategory("Mahmut Simple Walking Learner");
        model.addTest(new MahmutTest());

        TestbedPanel panel = new TestPanelJ2D(model);
        JFrame testbed = new TestbedFrame(model, panel, TestbedController.UpdateBehavior.UPDATE_CALLED);
        testbed.setVisible(true);
        testbed.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

    }

}
