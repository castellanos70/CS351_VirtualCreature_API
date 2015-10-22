package vcreature.mainSimulation;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.math.Vector3f;
import vcreature.phenotype.Creature;
import vcreature.phenotype.Block;
import vcreature.phenotype.EnumNeuronInput;
import vcreature.phenotype.Neuron;
import com.jme3.scene.Node;


public class FlappyBird2 extends Creature
{
  public static final float PI = (float)(Math.PI);

  
  public FlappyBird2(PhysicsSpace physicsSpace, Node rootNode)
  {
    super(physicsSpace, rootNode);
    
    //a Center is only needed for the root.
    //Note: This is intentionally placed 10 meters above the ground.
    //The Creature method placeOnGround() will be used to place it on the ground before starting the simulation.
    Vector3f torsoCenter = new Vector3f( 0.0f, 12.5f, 0.0f);     
    
    Vector3f torsoSize = new Vector3f( 2.0f, 1.5f, 1.5f);
    Vector3f leg1Size  = new Vector3f( 3.0f, 0.5f, 1.0f);
    Vector3f leg2Size  = new Vector3f( 3.0f, 0.5f, 1.0f);
    
    //Euler rotation angles (x,y,z) aka (pitch, yaw, rall)). 
    //Note: Euler angles are applying in order: (y, z, x) aka (yaw, roll, pitch).
    float[] eulerAngles = {0, PI/6.0f, 0};
    
    
    Block torso = addRoot(torsoCenter, torsoSize, eulerAngles);
    
    Vector3f pivotA = new Vector3f( 2.0f, -1.5f,  0.0f); //Center of hinge in the block's coordinates
    Vector3f pivotB = new Vector3f(-3.0f,  0.5f,  0.0f); //Center of hinge in the block's coordinates
    

    //Notice that even though the blocks are rotated 30 degrees, since the pivot points and pivot axes are
    //   specified in each block's local coordinates, there is no change to these values.
    Block leg1  = addBlock(eulerAngles, leg1Size,torso, pivotA,  pivotB, Vector3f.UNIT_Z);
    
    
    
    Vector3f pivotC = new Vector3f(-2.0f, -1.5f,  0.0f); //Center of hinge in the block's coordinates
    Vector3f pivotD = new Vector3f( 3.0f,  0.5f,  0.0f); //Center of hinge in the block's coordinates
    
    Block leg2  = addBlock(eulerAngles, leg2Size,torso, pivotC,  pivotD, Vector3f.UNIT_Z);
    
    
    
    
    
    torso.setMaterial(Block.MATERIAL_GREEN);
    leg1.setMaterial(Block.MATERIAL_RED);
    leg2.setMaterial(Block.MATERIAL_BLUE);
    
    placeOnGround();
    
    
    Neuron leg1Neuron1 = new Neuron(EnumNeuronInput.TIME, null, EnumNeuronInput.CONSTANT,
                EnumNeuronInput.CONSTANT, null);
    
    leg1Neuron1.setInputValue(Neuron.C,11);
    leg1Neuron1.setInputValue(Neuron.D,-Float.MAX_VALUE);
    
    Neuron leg1Neuron2 = new Neuron(EnumNeuronInput.TIME, null, EnumNeuronInput.CONSTANT,
                EnumNeuronInput.CONSTANT, null);
    
    leg1Neuron2.setInputValue(Neuron.C,10);
    leg1Neuron2.setInputValue(Neuron.D,Float.MAX_VALUE);
    
    leg1.addNeuron(leg1Neuron1);
    leg1.addNeuron(leg1Neuron2);
    
    
    Neuron leg2Neuron1 = new Neuron(EnumNeuronInput.TIME, null, EnumNeuronInput.CONSTANT,
                EnumNeuronInput.CONSTANT, null);
    
    leg2Neuron1.setInputValue(Neuron.C,11);
    leg2Neuron1.setInputValue(Neuron.D,Float.MAX_VALUE);
    
    Neuron leg2Neuron2 = new Neuron(EnumNeuronInput.TIME, null, EnumNeuronInput.CONSTANT,
                EnumNeuronInput.CONSTANT, null);
    
    leg2Neuron2.setInputValue(Neuron.C,10);
    leg2Neuron2.setInputValue(Neuron.D,-Float.MAX_VALUE);
    
    leg2.addNeuron(leg2Neuron1);
    leg2.addNeuron(leg2Neuron2);
    
    //System.out.println(leg1Neuron1);
    //System.out.println(leg1Neuron2);
  }
}