import maya.cmds as cmds
import maya.OpenMaya as om



    
    
    
    
def makeIKjoints(objects):
       
    
        #this makes sure that we have all children selected so we only need to select start of IK jnt before running script
    
    print (objects)
    
        #Creates empty group that we'll fill with the iK Joints
    ikJoints = []
        
        #this will run a for loop for every of the three IK joints
    for index, obj in enumerate(objects):
            
            #this deselects current joint
        cmds.select(deselect=True)
            #These three lines create 3 vars which store the radius, translate, and jointOrient values. Radius is 1 value, translate and 
        #jointOrient are 3 floats that we can plug in later!
        jntRadius = cmds.getAttr("%s.radius" % obj)
        jntOrient = cmds.getAttr("%s.jointOrient" % obj)
            
            #This creates a string var which contains our new name for the IK joint that will go on top
        jointNameIK = "%s_%s" % (obj, "IK")
            
            #An if statement that will take the vars we made to store the values of the thigh joint and make an IK joint using those values
            #directly on top! The radius is also increased for visiblity
        if index == 0:
            iKJoint = cmds.joint(name=jointNameIK, orientation=(jntOrient[0]),radius=(jntRadius * 1.5), relative=True)
            ikJoints.append(iKJoint) #Store the joint for later!
                
            # An elif statement (else if) that runs for the next two jnts but not after, as then index will be 3, which is perfect. This as well
            #takes the values stored from the base joint and creates a new IK jnt with it but this time it will parent to the previous IK jointName
        elif index < 3:
            iKJoint = cmds.joint(name=jointNameIK, orientation=(jntOrient[0]),radius=(jntRadius * 1.5))
            cmds.parent(jointNameIK, objects[index-1]+"_IK", relative=True)
            ikJoints.append(iKJoint)
            #If statement to see when the 3 joints end to use parentConstraints to align joints 
        if index == 2:
                
            #These three lines are constraining the IK joints to the OG thigh, knee, and ankle joints to line them up translation wise
            cmds.parentConstraint(objects[0], ikJoints[0], maintainOffset = False)
            cmds.parentConstraint(objects[1], ikJoints[1], maintainOffset = False)
            cmds.parentConstraint(objects[2], ikJoints[2], maintainOffset = False)
            
            #These three lines are now removing those constraints from the previous part as they've served their purpose     
            cmds.parentConstraint(objects[0], ikJoints[0], remove=True)
            cmds.parentConstraint(objects[1], ikJoints[1], remove=True)
            cmds.parentConstraint(objects[2], ikJoints[2], remove=True)
                
            
            
            #Makes the OG Joints be constrained to the new IK joints no matter if the pelvis is attached!   
            cmds.parentConstraint(ikJoints[0], objects[0])
            cmds.parentConstraint(ikJoints[1], objects[1])
            cmds.parentConstraint(ikJoints[2], objects[2])
                
            #Applies transforms for the thighJointIK to remove rotation data and make ikHandle without snapping
            cmds.makeIdentity(ikJoints[0], apply=True, t=1, r=1, s=1, n=0) 
                
            #Makes an IK handle with the thigh jnt as the starting joint (sj) and end Effector (ee)
            ikHandleName = objects[0] + "_IKhandle"
            cmds.ikHandle(name = ikHandleName, sj=ikJoints[0], ee = ikJoints[2])
            
            #Creates the control
            ctrlName = ikHandleName+"_ctrl"
            cmds.circle( n=ctrlName, nr=(0, 90, 0), c=(0, 0, 0) ) 
            
             # Creates offset group's name in a var then creates the group itself.
            offsetName = ikHandleName+"_offset"
            cmds.group( em=True, name=offsetName )
    
            # Creates grp group's name in a var then creates the group itself.
            groupName = ikHandleName+"_grp"
            cmds.group( em=True, name=groupName )
            
            # Parents the ctrl to the offset and the offset to the grp!
            cmds.parent(ctrlName,offsetName)
            cmds.parent(offsetName,groupName)
            
            # then parents to the joint for rotation then constrains IK handle to it
            cmds.parentConstraint(ikJoints[2], ctrlName, maintainOffset = False)
            cmds.parentConstraint(ikJoints[2], ctrlName, remove = True)
            
            cmds.parent(ikHandleName,ctrlName)
            
         
            return ikHandleName
                
def create_loc(pos,ikHandleName):
    locName = ikHandleName+"_locator"
    loc = cmds.spaceLocator(name=ikHandleName+"_locator")
    cmds.move(pos.x, pos.y, pos.z, loc)
        
    return locName
    
    # Maya Vectors are great as we can use .x, .y, or .z instead of 0,1,2 in a list!
   

def get_pole_vec_pos(root_pos, mid_pos, end_pos):
        
        # We're creating a var to hold the mVectors, and using 0, 1, and 2 as it's a list
        # For X, Y,and Z
        
    root_joint_vec = om.MVector(root_pos[0], root_pos[1], root_pos[2])
    mid_joint_vec = om.MVector(mid_pos[0], mid_pos[1], mid_pos[2])
    end_joint_vec = om.MVector(end_pos[0], end_pos[1], end_pos[2])
    
        # Calculates the line vector from start to end joint
    line = (end_joint_vec - root_joint_vec)
    point = (mid_joint_vec - root_joint_vec)
        
        # Calculates where the Locator should be on the line between start and end joint for best 
        # Placement
    scale_value = (line * point) / (line * line)
    proj_vec = line * scale_value + root_joint_vec
        
        # these three length vars are holding distance from root to mid and then mid to end joints
        # before combining them together for the total length of the three joints
    root_to_mid_len = (mid_joint_vec - root_joint_vec).length()
    mid_to_end_len = (end_joint_vec - mid_joint_vec).length()
    total_length = root_to_mid_len + mid_to_end_len
        
        # We need to normalize the vector so it's length is equal to 1 and is consistent when we
        # Scale it up. We're creating a var that will hold the pole_vec position, which will be 
        # From the mid_point joint outwards as long as the entire IK joint chain
    pole_vec_pos = (mid_joint_vec - proj_vec).normal() * total_length + mid_joint_vec
        
        # Creates locator at the create_loc position
    return pole_vec_pos
        
        # Creates function to get the pole vec position with ik_handle as input argument
def get_ikh_pole_vec_pos(ik_handle):
        
        # Var that holds list of joints for iKHandle (only shows root and mid joint though)
    ik_joint_list = cmds.ikHandle(ik_handle, q=True, jointList=True)
        
        # Appends list by finding a joint child of mid joint which is the end joint and says to just get
        # First element of child with [0] so we only get the end joint itself and not a list within
        # a list! 
    ik_joint_list.append(cmds.listRelatives(ik_joint_list[-1], children=True, type="joint")[0])
        
        # Creates 3 vars holding the positions of the three joints 
    root_joint_pos = cmds.xform(ik_joint_list[0], q=True, ws=True, t=True)
    mid_joint_pos = cmds.xform(ik_joint_list[1], q=True, ws=True, t=True)
    end_joint_pos = cmds.xform(ik_joint_list[2], q=True, ws=True, t=True) 
        
        # Uses our get_pole_vec_pos function with our joint input vars to do the math for the pole vector pos
    pole_vec_pos = get_pole_vec_pos(root_joint_pos, mid_joint_pos, end_joint_pos)
        
        # Returns our pole_vec_pos for use in other functions! 
    return pole_vec_pos
        
def make_pole_vector(locator, ik_handle):
    cmds.poleVectorConstraint(locator, ik_handle)
    
objects = cmds.ls(selection=True, dag=True, long=False)

ikHandleName = makeIKjoints(objects)    
        
pole_vec_pos = get_ikh_pole_vec_pos(ikHandleName) 
   
locName = create_loc(pole_vec_pos, ikHandleName)
    
make_pole_vector(locName, ikHandleName)
