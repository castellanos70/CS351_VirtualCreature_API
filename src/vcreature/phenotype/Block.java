package vcreature.phenotype;



import com.jme3.bullet.PhysicsSpace;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.shape.Box;
import com.jme3.bullet.joints.HingeJoint;
import com.jme3.scene.Node;
import com.jme3.renderer.queue.RenderQueue.ShadowMode;
import java.util.ArrayList;
import com.jme3.asset.AssetManager;
import com.jme3.bounding.BoundingBox;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.math.Quaternion;

/**
 * The Class Block.
 */
public class Block
{
  /**
   * @deprecated 
   * should use PhysicsConstants.BLOCK_DENSITY 
   */
  public static final float BLOCK_DENSITY = 4f; 
  
  
  /**
   * Basic red material with lighting properties.
   * This is provided for convenience and has no effect on the physics.
   */
  public static Material MATERIAL_RED;
  
  
  /**
   * Basic blue material with lighting properties.
   * This is provided for convenience and has no effect on the physics.
   */
  public static Material MATERIAL_BLUE;
  
  
  /**
   * Basic green material with lighting properties.
   * This is provided for convenience and has no effect on the physics.
   */
  public static Material MATERIAL_GREEN;
  
  
  
  /**
   * Basic brown material with lighting properties.
   * This is provided for convenience and has no effect on the physics.
   */
  public static Material MATERIAL_BROWN;
  
  
  /**
   * Basic gray material with lighting properties.
   * This is provided for convenience and has no effect on the physics.
   */
  public static Material MATERIAL_GRAY;
  
  
  private final float sizeX, sizeY, sizeZ; //meters
  private final Vector3f startCenter; //meters
  private final Quaternion startRotation;
  private final int id; //Assigned when added to Creature. 0=root and +1 for each block added in order the blocks are added. This is used by DNA and logic curits
  
  private Block parent;
  private HingeJoint jointToParent;
  private ArrayList<Neuron> neuronTable = new ArrayList<>();
  private ArrayList<Block> childList    = new ArrayList<>();
  private final Geometry geometry;
  private final RigidBodyControl physicsControl;
  
  //Temporary vectors used on each frame. They here to avoid instanciating new vectors on each frame
  private Vector3f tmpVec3 = new Vector3f(); //
  
  
  
  
  /**
   * Creates a box
   * @param physicsSpace
   * @param rootNode
   * @param id Index of this block in the creature's body ArrayList
   * @param center location in world coordinates of the center of the box.
   *     The address of this Vector3f is copied into this blocks startCenter field.
   *     Therefore, the address passed as center must be a new instrance of 
   *     Vector3f and not reused for other data.
   *
   * @param size  extent of box in each direction from the box's center. 
   * So, for example, a box with extent of 0.5 in the x dimension
   * would have a length in the x dimension of 1.0 meters.
   * @param rotation in world coordinates of this block.
   *     The address of this Quaternion is copied into this blocks startRotation field.
   *     Therefore, the address passed as rotation must be a new instrance of 
   *     Quaternion and not reused for other data.
   */
  public Block(PhysicsSpace physicsSpace, Node rootNode, int id, Vector3f center, Vector3f size, Quaternion rotation) 
  { 
    if (size.x < 0.5f || size.y < 0.5f || size.z < 0.5f) 
    { throw new IllegalArgumentException("No dimension may be less than 0.5 from block's center: ("+vectorToStr(size));
    }
    
    if (max(size) > 10*min(size))
    {  throw new IllegalArgumentException("Largest dimension must be no more than 10x the smallest: ("+vectorToStr(size)+")");
    }
    
    this.id = id;
    
    //Copies only the address, but in the creature class, 
    //  this addesss was created with new and is not reused
    startCenter   = center; 
    startRotation = rotation;
    
    sizeX = size.x*2;
    sizeY = size.y*2;
    sizeZ = size.z*2;

    
    //Creates a box that has a center of 0,0,0 and extends in the out from 
    //the center by the given amount in each direction. 
    // So, for example, a box with extent of 0.5 would be the unit cube.
    Box box = new Box(size.x, size.y, size.z);
    geometry = new Geometry("Box", box);
    geometry.setMaterial(MATERIAL_GRAY);
    rootNode.attachChild(geometry);
    geometry.setShadowMode(ShadowMode.Cast);
    geometry.rotate(rotation);
    geometry.move(startCenter);
    
    physicsControl = new RigidBodyControl(getMass());
    geometry.addControl(physicsControl);
    
    
    physicsControl.setPhysicsRotation(rotation);
    physicsControl.setPhysicsLocation(startCenter);
    physicsSpace.add(physicsControl);
    physicsControl.setRestitution(PhysicsConstants.BLOCK_BOUNCINESS);
    physicsControl.setFriction(PhysicsConstants.SLIDING_FRICTION);
    physicsControl.setDamping(PhysicsConstants.LINEAR_DAMPINING, 
            PhysicsConstants.ANGULAR_DAMPINING);
  }

  
  /**
   *
   * @param mat
   */
  public void setMaterial(Material mat)
  {
    geometry.setMaterial(mat);
  }


  /**
   *
   * @param parent
   * @param joint
   */
  public void setJointToParent(Block parent, HingeJoint joint)
  {
     jointToParent = joint;
     parent.childList.add(this);
     this.parent = parent;
  }

  /**
   * Adds the given Neuron to this blocks Neuron Table.
   * At the time this Neuron is added, if any of its
   * blockIds is UNDEFINED, then that blockId is set to the id of this block.
   * @param neuron to be added to Neuron Table.
   */
  public void addNeuron(Neuron neuron)
  {
    for (int i=0; i<Neuron.TOTAL_INPUTS; i++)
    { if (neuron.getBlockIdx(i) == Neuron.UNDEFINED)
      { neuron.setBlockIdx(i,id);
      }
    }
    neuronTable.add(neuron);
  }
  
  
  /**
   *
   * @return geometry
   */
  public Geometry getGeometry() {return geometry;}
  
  
  /**
   *
   * @return a pointer to the RigidBody physics control.
   */
  public RigidBodyControl getPhysicsControl() {return physicsControl;}
  
  
  
  /**
   *
   * @return a pointer to the joint to this block's parent (or null if 
   * this is the root block).
   */
  public HingeJoint getJoint(){ return jointToParent;}
  
  
  
  /**
   *
   * @return the current angle in the physics simulation of the joint 
   * to this block's parent. Returns 0 if there is no parent (if this is 
   * the root).
   */
  public float getJointAngle() 
  { if (jointToParent == null) return 0;
    return jointToParent.getHingeAngle(); 
  }
  
  
  /**
   * Sets the values in the given Vector3f output to the current center of the block
   * in world coordinates. 
   * 
   * @param output Vector into which the center is stored.
   * @return a pointer to the given Vector3f output for for easy chaining of calls.
   */
  public Vector3f getCenter(Vector3f output) {return physicsControl.getPhysicsLocation(output); }
  
  
  /**
   * Sets the values in the given Vector3f output to the original center (before the
   * strating the simulation) of the block in world coordinates. 
   * @param output Vector into which the center is stored.
   * @return a pointer to the given Vector3f output for for easy chaining of calls.
   */
  public Vector3f getStartCenter(Vector3f output) 
  { output.x = startCenter.x;
    output.y = startCenter.y;
    output.z = startCenter.z;
    return output; 
  }
  
  public Quaternion getStartRotation(Quaternion  output) 
  {
    return physicsControl.getPhysicsRotation(output);
  }
  
  
  /**
   * 
   * @return the height of the lowest face of this block's the bounding box 
   * in meters
   */
  public float getHeight()
  {
    BoundingBox box = (BoundingBox) geometry.getWorldBound();
    tmpVec3 = box.getMin(tmpVec3);
    //System.out.println("Block["+id+"].getHeight() = "+tmpVec3.y);
    float height = Math.max(0,tmpVec3.y);
    return height;
  }
  
  
  /**
   *
   * @return the index of this block in the creature.body ArrayList.
   */
  public int getID() {return id;}
  
  
   
  /**
   * 
   * @return the id of the parent or -1 if this is a root block.
   */
  public int getIdOfParent()
  { 
    if (parent == null) return -1;
    return parent.getID();
  }
  

  /**
   *
   * @return Full extent (not half extent) of the block along the x-axis of 
   * its local coordinates (before any rotations).
   */
  public float getSizeX() {return sizeX;}
  

  /**
   * 
   * @return Full extent (not half extent) of the block along the y-axis of 
   * its local coordinates (before any rotations).
   */
  public float getSizeY() {return sizeY;}
  

  /**
   *
   * @return Full extent (not half extent) of the block along the z-axis of 
   * its local coordinates (before any rotations).
   */
  public float getSize() {return sizeZ;}
  
  /**
   * Careful when using this. It gives a pointer to the block's childList.
   * If this is corrupted the results are continued use of the creature class 
   * are undefined.
   * @return
   */
  public ArrayList<Block> getChildList() {return childList;}
  
  
  /**
   * @return the ArrayList of Neurons that can send an impulse to 
   * the joint connecting this block to its parent. Returns null if this
   * block is the root block.
   */
  public ArrayList<Neuron> getNeuronTable() { return neuronTable;}
  
  
  
  
  /**
   * If a program is to used any of these simple materials, then 
   * this method must be called once some time before using the materials.
   * @param assetManager
   */
  public static void initStaticMaterials(AssetManager assetManager)
  {
    MATERIAL_RED   = initStaticMaterial(assetManager, ColorRGBA.Red);
    MATERIAL_BLUE  = initStaticMaterial(assetManager, ColorRGBA.Blue);
    MATERIAL_GREEN = initStaticMaterial(assetManager, ColorRGBA.Green);
    MATERIAL_BROWN = initStaticMaterial(assetManager, ColorRGBA.Brown);
    MATERIAL_GRAY  = initStaticMaterial(assetManager, ColorRGBA.Gray);
  }
  
  
  
  private static Material initStaticMaterial(AssetManager assetManager, ColorRGBA color)
  {
    Material mat = new Material(assetManager, "Common/MatDefs/Light/Lighting.j3md");
    mat.setBoolean("UseMaterialColors", true);
    mat.setColor("Ambient", color);
    mat.setColor("Diffuse", color);
    mat.setColor("Specular", color);
    mat.setFloat("Shininess", 64f);  // [0,128]
    return mat;
  }
  
  
  
  /**
   * This method may be useful in debugging. It returns a string that contains
   * some information about this block.
   * @return formatted String
   */
  public String toString()
  {
    String s = "Block["+id+"]: {" + sizeX + ", " + sizeY + ", " + sizeZ + "}\n";
    
    if (parent == null)
    { s += "     ROOT\n\n";
    }
    else
    {
      s += "     Parent: [ID=" + parent.getID() + "]: " + jointToParent + "\n"; 
    }
    return s;
  }
  
  
  /**
   * Return the maximium impulse that can be supplyed by a joint on the 
   * given parent.
   * the maximium impulse that can be applyed to a joint is proportional to the parent's surface area.
   * The maximium impulse is returned in Newton seconds.
   * @return the maximium impulse in Newton seconds
   */
  public final float getJointMaxImpulse() 
  {
    
    return parent.sizeX*parent.sizeY + parent.sizeY*parent.sizeZ + parent.sizeZ*parent.sizeX;
  }


  /**
   *
   * @return the mass of this block in kilograms.
   */
  public final float getMass()
  {
    return sizeX*sizeY*sizeZ*PhysicsConstants.BLOCK_DENSITY;
  }

  /**
   *
   * @param vec
   * @return the minimum value in the given Vector3f.
   */
  public static float min(Vector3f vec)
  {
    if (vec.x <= vec.y && vec.x <= vec.z) return vec.x;
    if (vec.y <= vec.x && vec.y <= vec.z) return vec.y;
    return vec.z;
  }

  /**
   *
   * @param vec
   * @return the maximum value in the given Vector3f.
   */
  public static float max(Vector3f vec)
  {
    if (vec.x >= vec.y && vec.x >= vec.z) return vec.x;
    if (vec.y >= vec.x && vec.y >= vec.z) return vec.y;
    return vec.z;
  }

  
  
  
  /**
   * This method may be useful in debugging. It returns a string that contains
   * the values in the vector.
   * @param vec
   * @return formatted String
   */
  public static String vectorToStr(Vector3f vec)
  {
    return "("+vec.x +", "+vec.y +", "+vec.z +")";
  }

}