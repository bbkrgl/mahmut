import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;

import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.*;
import org.jbox2d.dynamics.joints.*;

public class MahmutBuilder{
    private float posX;
    private float posY;
    private World world;

    private RevoluteJoint body_leg1Joint;
    private RevoluteJoint body_leg2Joint;

    public MahmutBuilder(World world, float posX, float posY) {
        this.posX = posX;
        this.posY = posY;
        this.world = world;
    }

    public void buildMahmut(){
        Body foot1 = foot();
        Body leg1 = leg();

        Body foot2 = foot();
        Body leg2 = leg();

        //Join leg1 and foot1
        WeldJointDef foot1JointDef = new WeldJointDef();
        foot1JointDef.bodyA = leg1;
        foot1JointDef.bodyB = foot1;
        foot1JointDef.collideConnected = false;
        foot1JointDef.localAnchorA.set(new Vec2(0,-8f));

        world.createJoint(foot1JointDef);

        //Join leg2 and foot2
        WeldJointDef foot2JointDef = new WeldJointDef();
        foot2JointDef.bodyA = leg2;
        foot2JointDef.bodyB = foot2;
        foot2JointDef.collideConnected = false;
        foot2JointDef.localAnchorA.set(new Vec2(0,-8f));

        world.createJoint(foot2JointDef);

        //Create upper body
        PolygonShape bodyShape = new PolygonShape();
        bodyShape.setAsBox(3,5);

        FixtureDef bodyFixture = new FixtureDef();
        bodyFixture.shape = bodyShape;
        bodyFixture.density = 10;

        BodyDef bodyDef = new BodyDef();
        bodyDef.position = new Vec2(posX , posY + 2.5f);
        bodyDef.type = BodyType.DYNAMIC;
        //bodyDef.fixedRotation = true;

        Body body = world.createBody(bodyDef);
        body.createFixture(bodyFixture);

        //Join upper body and leg1
        RevoluteJointDef body_leg1JointDef = new RevoluteJointDef();
        body_leg1JointDef.bodyA = body;
        body_leg1JointDef.bodyB = leg1;
        body_leg1JointDef.collideConnected = false;
        body_leg1JointDef.localAnchorA = new Vec2(0,-5f);
        body_leg1JointDef.localAnchorB = new Vec2(0,8f);

        body_leg1Joint = (RevoluteJoint) world.createJoint(body_leg1JointDef);

        //Join upper body and leg2
        RevoluteJointDef body_leg2JointDef = new RevoluteJointDef();
        body_leg2JointDef.bodyA = body;
        body_leg2JointDef.bodyB = leg2;
        body_leg2JointDef.collideConnected = false;
        body_leg2JointDef.localAnchorA = new Vec2(0,-5f);
        body_leg2JointDef.localAnchorB = new Vec2(0,8f);

        body_leg2Joint = (RevoluteJoint) world.createJoint(body_leg2JointDef);
    }

    public void moveLeg1(){
        body_leg1Joint.enableMotor(true);
        body_leg1Joint.enableLimit(true);
        body_leg1Joint.setMotorSpeed(10);
        body_leg1Joint.setMaxMotorTorque(10000);
        body_leg1Joint.setLimits((float) Math.PI / 6, (float) Math.PI / 6);
    }

    public void moveLeg2(){
        body_leg2Joint.enableMotor(true);
        body_leg2Joint.enableLimit(true);
        body_leg2Joint.setMotorSpeed(10);
        body_leg2Joint.setMaxMotorTorque(10000);
        body_leg2Joint.setLimits((float) -Math.PI / 6, (float) -Math.PI / 6);
    }


    private Body foot(){
        CircleShape footShape = new CircleShape();
        footShape.m_radius = 3f;

        FixtureDef footFixture = new FixtureDef();
        footFixture.shape = footShape;
        footFixture.density = 1.0f;
        footFixture.filter.categoryBits = 0x0002;
        footFixture.filter.maskBits = 0x0001;

        BodyDef footDef = new BodyDef();
        footDef.type = BodyType.DYNAMIC;
        footDef.position = new Vec2(posX,posY-8f);

        Body foot = world.createBody(footDef);
        foot.createFixture(footFixture);

        return foot;
    }

    private Body leg(){
        PolygonShape legShape = new PolygonShape();
        legShape.setAsBox(1f,8f);

        FixtureDef legFixture = new FixtureDef();
        legFixture.shape = legShape;
        legFixture.density = 1.0f;
        legFixture.filter.categoryBits = 0x0002;
        legFixture.filter.maskBits = 0x0001;

        BodyDef legDef = new BodyDef();
        legDef.type = BodyType.DYNAMIC;
        legDef.position = new Vec2(posX,posY-4f);

        Body leg = world.createBody(legDef);
        leg.createFixture(legFixture);

        return leg;
    }

}
