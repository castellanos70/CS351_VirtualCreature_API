package vcreature.phenotype;

import com.jme3.math.Vector3f;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.scene.Node;
import java.util.ArrayList;
import com.jme3.bullet.joints.HingeJoint;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.material.Material;
import com.jme3.math.Transform;
import com.jme3.math.Quaternion;
import com.jme3.scene.Geometry;



/**
 * The Class Creature.<br>
 * This is the required phenotype that is passed to the simulator.<br>
 * A block contains a Joint to its parent. <br>
 * Each joint contains an ArrayList of Rules for each of its degrees of freedom.<br>
 * Each creature must have exactly one block with jointToParent == null.<br>
 */
public class Creature
{
  /**
   * Instantiates a new creature.
   * Copies all values of blocks, joints and rules to internal tree structure.
   * After a Creature is instantiated, any changes made to its structure, will not
   * be noticed by the physics engine. Thus, all calls to 
   * getJointAngle(), getBoxForwardVector(), etc will be based on the original
   * Creature structure.
   * 
   * This constructor throws an IllegalArgumentException if:<br>
   * 1) Any block has a length, width or height less than 0.5 meters.<br>
   * 2) The body does not have a root block (block with jointToParent == null).<br>
   * 3) The body has more than one root block.<br> 
   * 5) When all the degrees of freedom of all joint are at the starting angle
   * of zero radians, a pair of blocks that are not parent/child intersect.
   *
   * 
   * The root block will always have ID=0
   */
  
  
  private final PhysicsSpace physicsSpace; 
  private final Node jMonkeyRootNode;
  private ArrayList<Block> body = new ArrayList<Block>();
  
  //Temporary vectors used on each frame. They here to avoid instanciating new vectors on each frame
  private Vector3f tmpVec3; 
  private float maxHeightOfLowestPoint = 0;  //fitness
  
  private float elapsedSimulationTime;
  
  /**
   *
   * @param physicsSpace
   * @param jMonkeyRootNode
   */
  public Creature(PhysicsSpace physicsSpace, Node jMonkeyRootNode)
  {
    
    this.physicsSpace = physicsSpace;
    this.jMonkeyRootNode = jMonkeyRootNode;
  }
  
  /**
   *
   * @param rootCenter
   * @param rootHalfSize
   * @return
   */
  public Block addRoot(Vector3f rootCenter, Vector3f rootHalfSize)
  {
    float[] eulerAngles = {0,0,0};
   
    return addRoot(rootCenter, rootHalfSize, eulerAngles);
  }
 
  /**
   * This method adds the root block to this creature. The root block is different from all other
   * blocks in that it has no parant. This means that no joint can be given an impulse that can 
   * directly move the root block. The root block can only move as an indirect consequence of
   * an impulse applyed to one of its decendants, by gravity or drag. 
   * 
   * @
   * @param rootCenter 
   * @param eulerAngles 
   * @param rootHalfSize 
   * @return a pointer to the root block of the creature;
   */
  public Block addRoot(Vector3f rootCenter, Vector3f rootHalfSize, float[] eulerAngles)
  {
    if (!body.isEmpty()) 
    { throw new IllegalArgumentException("This creature already has a root.");
    }
    
    Quaternion rotation = new Quaternion(eulerAngles);
    
    Block root = new Block(physicsSpace, jMonkeyRootNode, body.size(), rootCenter, rootHalfSize, rotation);
    
    body.add(root);

    return root;
  }
  
  
  /**
   * @deprecated 
   * @see #addBlock(float[] eulerAngles, Vector3f size, Block parent, Vector3f pivotA, Vector3f pivotB, Vector3f axisA, Vector3f axisB)
   * The center parameter is ignored as it can be calculated from the other information. Also,
   * this can only be used to add axis alligned blocks.
   * 
   * 
   * @param center ignored as this is now calculated.
   * @param halfsize half the extent (in meters) of the block in the x, y and z direction.
   * @param parent Block instance onto which this block will be joined.
   * @param pivotA Location in local coordinates of the pivot point on the parent block. 
   * Local coordinates means the location on the block relitive to the block's center with zero rotation.
   * @param pivotB Location in local coordinates of the pivot point on this block.
   * @param axisA One-degree of freedom hinge axis in local coordinates of the parent block.
   * @param axisB One-degree of freedom hinge axis in local coordinates of the this block.
   * @return a reference to the newly added block.
   */
  public Block addBlock(Vector3f center, Vector3f size, Block parent, Vector3f pivotA, Vector3f pivotB, Vector3f axisA, Vector3f axisB)
  {
    float[] eulerAngles = {0,0,0};
    return addBlock(eulerAngles, size, parent, pivotA, pivotB, axisA, axisB);
  }
 
  /**
   * @param eulerAngles 
   * @param halfsize half the extent (in meters) of the block in the x, y and z direction. 
   * For example, a block with extent in the x dimension of 0.5 would extend from 0.5 meters from 
   * the origin in the -x direction and 0.5 meters from the origin in the +x direction.
   * @param parent Block instance onto which this block will be joined.
   * @param pivotA Location in local coordinates of the pivot point on the parent block. 
   * Local coordinates means the location on the block relitive to the block's center with zero rotation.
   * @param pivotB Location in local coordinates of the pivot point on this block.
   * @param axisA One-degree of freedom hinge axis in local coordinates of the parent block.
   * @param axisB One-degree of freedom hinge axis in local coordinates of the this block.
   * @return a reference to the newly added block.
   * @return
   */
  public Block addBlock(float[] eulerAngles, Vector3f halfsize, Block parent, Vector3f pivotA, Vector3f pivotB, Vector3f axisA, Vector3f axisB)
  {
    if (body.isEmpty()) 
    { throw new IllegalArgumentException("Must call addRoot() before calling addBlock()");
    }
    
    
    Quaternion rotation = new Quaternion(eulerAngles);
    
    Transform parantTransform = parent.getGeometry().getWorldTransform();
    Vector3f pivotA_World = new Vector3f();
    parantTransform.transformVector(pivotA, pivotA_World);
    
    Transform childTransform = new Transform(rotation);
    
    Vector3f centerB = new Vector3f();
    childTransform.transformVector(pivotB, centerB);
    centerB.negateLocal();        

    centerB.addLocal(pivotA_World);
    
    Block block = new Block(physicsSpace, jMonkeyRootNode, body.size(), centerB, halfsize, rotation);
    body.add(block);
    
    
    RigidBodyControl controlA = parent.getPhysicsControl();
    RigidBodyControl controlB = block.getPhysicsControl();
    HingeJoint joint = new HingeJoint(controlA, controlB, pivotA, pivotB, axisA, axisB);
    joint.setCollisionBetweenLinkedBodys(false);
    
    joint.setLimit(PhysicsConstants.JOINT_ANGLE_MIN, PhysicsConstants.JOINT_ANGLE_MAX);
    block.setJointToParent(parent, joint);
    
    physicsSpace.add(joint);
    
    return block;
  }
  
  //public void reset()
  //{
    //setSpeed(1f);
  //  physicsSpace.setTime(0);
  //}
  
  /*public final void move(Vector3f displacement)
  {
    reset();
    for (Block block : body)
    {
      
    }
  }
*/
  
  /**
   * Removes all blocks (and joints) in this creature from the physics engine 
   * and from the render space.
   */
  public void remove()
  {
    removeSubTree(0);
  }
  
  /**
   * Removes the block at the specified id and all of its descendants from this creature.
   * The root will always have an id=0. 
   * @param id of block within the creature. 
   */
  public void removeSubTree(int id)
  {
    Block block = body.get(id);
    removeSubTree(block);
  }
  
  /**
   * Removes the specified block and all of its descendants from this creature.
   * @param block
   */
  public void removeSubTree(Block block)
  {
    for (Block child : block.getChildList())
    {
      removeSubTree(child);
    }
    
    physicsSpace.remove(block.getPhysicsControl());
    HingeJoint jointToParent = block.getJoint();
    if (jointToParent != null) physicsSpace.remove(jointToParent);
    
    Geometry geometry = block.getGeometry();
    geometry.removeFromParent();
    
    //System.out.println("remove("+block+")");
    block.clear();
    body.remove(block);
  }
  
  /**
   *
   * @param id
   * @param mat
   */
  public void setBlockMaterial(int id, Material mat)
  {
    body.get(id).setMaterial(mat);
  }

  /**
   *
   * @return
   */
  public int getNumberOfBodyBlocks(){return body.size();}

  /**
   *
   * @param id
   * @return
   */
  public Block getBlockByID(int id){return body.get(id);}
 
  /**
   * Gets the angle of the joint connecting the given block index with its parent.
   * This value is calculated and returned by the bullet physics engine.<br>
   * At simulation time 0.0, the value of every angle will always be zero.
   *
   * @param id of the child box.
   * @return the joint angle in radians +- deflection the zero point defined by the 
   * block orientations at the time the blocks were joined.
   */
  public final float getJointAngle(int id)
  { return body.get(id).getJointAngle();
  }
  
  /**
   *
   * @param id
   * @param output
   * @return
   */
  public Vector3f getCenter(int id, Vector3f output) 
  { return body.get(id).getCenter(output); 
  }
  
 
  /**
   *
   * @param id
   * @return the lowest y value of the bounding box of the block with the given id.
   */
  public final float getHeight(int id)
  { 
    return body.get(id).getHeight();
  }
  

  //
  //public deleteBlockAndDescendants()
  //{
  //   descendants
  //}
  
  
  
  
  /**
   *
   * @param elapsedSimulationTime
   * @return
   */
  public float updateBrain(float elapsedSimulationTime)
  {
    this.elapsedSimulationTime = elapsedSimulationTime;
    for (Block block : body)
    {
      HingeJoint joint = block.getJoint();
      if (joint == null) continue;
      
      ArrayList<Neuron> neuronTable = block.getNeuronTable();
      for (Neuron neuron: neuronTable)
      {
        if (brainNeuronFire(neuron))
        { sendNeuronInpulse(block, joint, neuron);
          break;
        }
      }
    }
    return updateFitness();
  }
  
  
  private float updateFitness()
  {
    float currentHeightOfLowestPoint = Float.MAX_VALUE;
    for (Block block : body)
    {
      float height = block.getHeight();
      if (height < currentHeightOfLowestPoint) currentHeightOfLowestPoint = height;
    }
   
    if (currentHeightOfLowestPoint > maxHeightOfLowestPoint) maxHeightOfLowestPoint = currentHeightOfLowestPoint;
    return maxHeightOfLowestPoint;
  }
  
  /**
   *
   * @return
   */
  public float getFitness()
  {
    return maxHeightOfLowestPoint;
  }
  
  
  /**
   *
   * @param neuron
   * @return
   */
  public boolean brainNeuronFire(Neuron neuron)
  {
    float a = getNeuronInput(neuron, Neuron.A);
    float b = getNeuronInput(neuron, Neuron.B);
    float y = neuron.getOutput(a,b, Neuron.FIRST_HALF);
    float c = getNeuronInput(neuron, Neuron.C);
    
    if (y>c) return true;
    return false;
  }
  
  /**
   *
   * @param block
   * @param joint
   * @param neuron
   */
  public void sendNeuronInpulse(Block block, HingeJoint joint, Neuron neuron)
  {
    float d       = getNeuronInput(neuron, Neuron.D);
    float e       = getNeuronInput(neuron, Neuron.E);
    float impulse = neuron.getOutput(d,e, Neuron.SECOND_HALF);
    float speed   = PhysicsConstants.JOINT_MAX_ANGULAR_SPEED;
    if (impulse < 0)
    {
      speed   = -speed;
      impulse = -impulse;
    }
    
    if (impulse > block.getJointMaxImpulse()) impulse = block.getJointMaxImpulse();
   
    
    joint.enableMotor(true, speed, impulse);
    block.getPhysicsControl().activate();
  
  }
  
  
  //It seems like this method belongs in the Neuron class, but the Neuron class does not 
  //   know block heights, joints nor elapsed time.
  /**
   *
   * @param neuron
   * @param i
   * @return
   */
  public float getNeuronInput(Neuron neuron, int i)
  {
    float x = 0;
    if (neuron.getInputType(i) == EnumNeuronInput.CONSTANT)   x = neuron.getInputValue(i);
    else if (neuron.getInputType(i) == EnumNeuronInput.TIME)  x = elapsedSimulationTime;
    else if (neuron.getInputType(i) == EnumNeuronInput.JOINT) 
    { 
      x = getJointAngle(neuron.getBlockIdx(i));
    }
    else if (neuron.getInputType(i) == EnumNeuronInput.HEIGHT) 
    { 
      x = getHeight(neuron.getBlockIdx(i));
    }
             

    return x;
  }
  
  
  /**
   *
   * @param msg
   * @param vector
   */
  public void print(String msg, Vector3f vector)
  {
    String className = this.getClass().getSimpleName();
    System.out.format("%s.%s [%.3f, %.3f, %.3f]\n", className, msg, vector.x, vector.y, vector.z);
  }  
}
