import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.*;
import org.jbox2d.testbed.framework.TestbedTest;

public class MahmutTest extends TestbedTest{
    private MahmutBuilder mahmutBuilder;
    @Override
    public void initTest(boolean deserialized) {
        setTitle("Mahmut Simple Walking Learner Test");
        setProperties();
        mahmutBuilder = new MahmutBuilder(getWorld(), 10f, 11f);
        mahmutBuilder.buildMahmut();
    }

    @Override
    public String getTestName() {
        return "Mahmut Simple Walking Learner";
    }

    @Override
    public void keyPressed(char keyCar, int keyCode) {
        if(keyCar == 'a'){
            mahmutBuilder.moveLeg1((float)Math.PI/6);
        }
    }

    private void setProperties(){
        getWorld().setGravity(new Vec2(0,-9.8f));

        EdgeShape ed = new EdgeShape();
        ed.set(new Vec2(-300f,-10f), new Vec2(300f,-10f));

        FixtureDef fd = new FixtureDef();
        fd.shape = ed;
        fd.friction = 1f;

        BodyDef bodyDef = new BodyDef();
        bodyDef.position = new Vec2(0,0);
        getWorld().createBody(bodyDef).createFixture(fd);
    }
}
