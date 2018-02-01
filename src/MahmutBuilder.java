import org.jbox2d.callbacks.ContactImpulse;
import org.jbox2d.callbacks.ContactListener;
import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;

import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.*;
import org.jbox2d.dynamics.contacts.Contact;
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
        //Create foot1
        CircleShape foot1Shape = new CircleShape();
        foot1Shape.m_radius = 3f;

        FixtureDef foot1Fixture = new FixtureDef();
        foot1Fixture.shape = foot1Shape;
        foot1Fixture.density = 1.0f;
        foot1Fixture.friction = 1f;
        foot1Fixture.filter.categoryBits = 0x0002;
        foot1Fixture.filter.maskBits = 0x0001;

        BodyDef foot1Def = new BodyDef();
        foot1Def.type = BodyType.DYNAMIC;
        foot1Def.position = new Vec2(posX,posY-8f);

        Body foot1 = world.createBody(foot1Def);
        foot1.createFixture(foot1Fixture);

        //Create foot2
        CircleShape foot2Shape = new CircleShape();
        foot2Shape.m_radius = 3f;

        FixtureDef foot2Fixture = new FixtureDef();
        foot2Fixture.shape = foot1Shape;
        foot2Fixture.density = 1.0f;
        foot2Fixture.friction = 1f;
        foot2Fixture.filter.categoryBits = 0x0002;
        foot2Fixture.filter.maskBits = 0x0001;

        BodyDef foot2Def = new BodyDef();
        foot2Def.type = BodyType.DYNAMIC;
        foot2Def.position = new Vec2(posX,posY-8f);

        Body foot2 = world.createBody(foot2Def);
        foot2.createFixture(foot2Fixture);

        //Create leg1
        PolygonShape leg1Shape = new PolygonShape();
        leg1Shape.setAsBox(1f,8f);

        FixtureDef leg1Fixture = new FixtureDef();
        leg1Fixture.shape = leg1Shape;
        leg1Fixture.density = 1.0f;
        leg1Fixture.filter.categoryBits = 0x0002;
        leg1Fixture.filter.maskBits = 0x0001;

        BodyDef leg1Def = new BodyDef();
        leg1Def.type = BodyType.DYNAMIC;
        leg1Def.position = new Vec2(posX,posY-4f);

        Body leg1 = world.createBody(leg1Def);
        Fixture leg1Fix = leg1.createFixture(leg1Fixture);

        //Create leg1
        PolygonShape leg2Shape = new PolygonShape();
        leg2Shape.setAsBox(1f,8f);

        FixtureDef leg2Fixture = new FixtureDef();
        leg2Fixture.shape = leg1Shape;
        leg2Fixture.density = 1.0f;
        leg2Fixture.filter.categoryBits = 0x0002;
        leg2Fixture.filter.maskBits = 0x0001;

        BodyDef leg2Def = new BodyDef();
        leg2Def.type = BodyType.DYNAMIC;
        leg2Def.position = new Vec2(posX,posY-4f);

        Body leg2 = world.createBody(leg2Def);
        Fixture leg2Fix = leg2.createFixture(leg2Fixture);

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
        bodyFixture.density = 8;

        BodyDef bodyDef = new BodyDef();
        bodyDef.position = new Vec2(posX , posY + 2.5f);
        bodyDef.type = BodyType.DYNAMIC;

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

        //Sensor Positions
        float sensorPosX = posX+3;
        float sensorPosY = posY+11;

        //Leg1 Sensor
        PolygonShape sensor1Shape = new PolygonShape();
        sensor1Shape.setAsBox(0.001f,35 * (float)Math.sqrt(2)/15);

        FixtureDef sensor1Fixture = new FixtureDef();
        sensor1Fixture.shape = sensor1Shape;
        sensor1Fixture.density = 0.00001f;
        sensor1Fixture.isSensor = true;

        BodyDef sensor1Def = new BodyDef();
        sensor1Def.type = BodyType.DYNAMIC;
        sensor1Def.position = new Vec2(sensorPosX,sensorPosY);

        Body sensor1 = world.createBody(sensor1Def);
        Fixture sensor1Fix = sensor1.createFixture(sensor1Fixture);
        sensor1Fix.setUserData("sensor1");

        WeldJointDef sensor1Leg1JointDef = new WeldJointDef();
        sensor1Leg1JointDef.bodyA = body;
        sensor1Leg1JointDef.bodyB = sensor1;
        sensor1Leg1JointDef.localAnchorA.set(new Vec2(3,-5f));
        sensor1Leg1JointDef.localAnchorB.set(new Vec2(0,0));

        world.createJoint(sensor1Leg1JointDef);

        //Leg2 Sensor
        PolygonShape sensor2Shape = new PolygonShape();
        sensor1Shape.setAsBox(0.001f,35 * (float)Math.sqrt(2)/15);

        FixtureDef sensor2Fixture = new FixtureDef();
        sensor2Fixture.shape = sensor2Shape;
        sensor2Fixture.density = 0.00001f;
        sensor2Fixture.isSensor = true;

        BodyDef sensor2Def = new BodyDef();
        sensor2Def.type = BodyType.DYNAMIC;
        sensor2Def.position = new Vec2(sensorPosX-6,sensorPosY);

        Body sensor2 = world.createBody(sensor1Def);
        Fixture sensor2Fix = sensor2.createFixture(sensor1Fixture);
        sensor2Fix.setUserData("sensor2");

        WeldJointDef sensor2Leg2JointDef = new WeldJointDef();
        sensor2Leg2JointDef.bodyA = body;
        sensor2Leg2JointDef.bodyB = sensor2;
        sensor2Leg2JointDef.localAnchorA.set(new Vec2(-3,-5f));
        sensor2Leg2JointDef.localAnchorB.set(new Vec2(0,0));

        world.createJoint(sensor2Leg2JointDef);

        world.setContactListener(new SensorListener());
    }

    public void moveLeg1(float angle){
        body_leg1Joint.enableMotor(true);
        body_leg1Joint.enableLimit(true);
        body_leg1Joint.setMotorSpeed(0.01f);
        body_leg1Joint.setMaxMotorTorque(1000);
        body_leg1Joint.setLimits(angle, angle);
    }

    public void moveLeg2(float angle){
        body_leg2Joint.enableMotor(true);
        body_leg2Joint.enableLimit(true);
        body_leg2Joint.setMotorSpeed(0.01f);
        body_leg2Joint.setMaxMotorTorque(1000);
        body_leg2Joint.setLimits(angle, angle);
    }

    private class SensorListener implements ContactListener{
        private int contact_num = 0;

        private Fixture getSensor(Fixture f1, Fixture f2){
            if(f1 != null && f1.isSensor()){
                return f1;
            }else if(f2 != null && f2.isSensor()){
                return f2;
            }else {
                return null;
            }
        }

        @Override
        public void beginContact(Contact contact) {
            Fixture sensor;
            sensor = getSensor(contact.getFixtureA(), contact.getFixtureB());
            if(sensor != null && sensor.getUserData().equals("sensor1")){
                contact_num++;
                if(contact_num > 1){
                    moveLeg2((float)-Math.PI/6);
                }
            }else if(sensor != null && sensor.getUserData().equals("sensor2")){
                contact_num++;
                if(contact_num > 1){
                    moveLeg1(0);
                    moveLeg2(0);
                }
            }
        }

        @Override
        public void endContact(Contact contact) {

        }

        @Override
        public void preSolve(Contact contact, Manifold oldManifold) {

        }

        @Override
        public void postSolve(Contact contact, ContactImpulse impulse) {

        }

    }

}
