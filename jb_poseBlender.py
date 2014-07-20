import maya.OpenMaya as om
import maya.OpenMayaMPx as omMPx
from internalPackages.maya import mayaMain

systems = ('jbPlugins', 'mec_Plugins')
'''
# classes for different commands that utilizes the node
class jb_applyFakeDynamics(omMPx.MPxCommand):
    kPluginCmdName = "jb_applyFakeDynamics"

    def __init__(self):
        omMPx.MPxCommand.__init__(self)

    @staticmethod
    def cmdCreator():
        return omMPx.asMPxPtr( jb_applyFakeDynamics() )

    def doIt(self,argList):
        import maya.cmds as cmds
        for obj in cmds.ls( sl=True ):
            dynObj = cmds.duplicate( obj, n=(obj+'_dyn') )[0]
            fNode = cmds.createNode( 'jb_poseBlender' )
            cmds.connectAttr( obj+'.matrix', fNode+'.inMatrix' )
            cmds.setAttr( fNode+'.st', cmds.currentTime( q=True ) )
            cmds.connectAttr( 'time1.outTime', fNode+'.time' )
            cmds.connectAttr( fNode+'.translate', dynObj+'.translate' )
            cmds.connectAttr( fNode+'.rotate', dynObj+'.rotate' )
            cmds.connectAttr( fNode+'.scale', dynObj+'.scale' )
            cmds.connectAttr( fNode+'.shear', dynObj+'.shear' )
            cmds.connectAttr( dynObj+'.ro', fNode+'.ro' )

class jb_removeFakeDynamics(omMPx.MPxCommand):
    kPluginCmdName = "jb_removeFakeDynamics"

    def __init__(self):
        omMPx.MPxCommand.__init__(self)

    @staticmethod
    def cmdCreator():
        return omMPx.asMPxPtr( jb_removeFakeDynamics() )

    def doIt(self,argList):
        import maya.cmds as cmds
        sel = cmds.ls( sl=True )

        for obj in sel:
            if cmds.objExists( obj ):
                type = cmds.nodeType( obj )
                if type == 'jb_poseBlender' or type == 'transform':
                    fNode = obj
                    oNode = obj
                    if type == 'transform':
                        fNode = cmds.listConnections( obj, t='jb_poseBlender' )
                        fNode = fNode[0]

                    if fNode != None:
                        oNode = cmds.listConnections( (fNode+'.translate'), t='transform' )
                        if oNode != None:
                            oNode = oNode[0]
                            cmds.delete( oNode, fNode )

class jb_setFakeDynamics(omMPx.MPxCommand):
    kPluginCmdName = "jb_setFakeDynamics"

    def __init__(self):
        omMPx.MPxCommand.__init__(self)

    @staticmethod
    def cmdCreator():
        return omMPx.asMPxPtr( jb_setFakeDynamics() )

    def doIt(self,argList):
        import maya.cmds as cmds
        import math
        sel = cmds.ls( sl=True )
        for obj in sel:
            type = cmds.nodeType( obj )
            if type == 'jb_poseBlender' or type == 'transform':
                fNode = obj
                oNode = obj
                if type == 'transform':
                    fNode = cmds.listConnections( obj, t='jb_poseBlender' )
                    if fNode != None:
                        fNode = fNode[0]

                if fNode != None:
                    oNode = cmds.listConnections( (fNode+'.inMatrix'), t='transform' )
                    if oNode != None:
                        oNode = oNode[0]

                        time = cmds.currentTime( q=True )
                        thisPos = cmds.getAttr( (oNode+'.t'), time=( time ) )[0]
                        oldPos = cmds.getAttr( (oNode+'.t'), time=( time - 1 ) )[0]

                        xDelta = thisPos[0] - oldPos[0]
                        yDelta = thisPos[1] - oldPos[1]
                        zDelta = thisPos[2] - oldPos[2]

                        cmds.setAttr( fNode+'.ivx', xDelta )
                        cmds.setAttr( fNode+'.ivy', yDelta )
                        cmds.setAttr( fNode+'.ivz', zDelta )



                        attr = cmds.getAttr(oNode+'.matrix')

                        am = om.MMatrix()
                        om.MScriptUtil.createMatrixFromList(attr,am)
                        am = am.homogenize()

                        attr = cmds.getAttr(oNode+'.matrix', t=time-1)
                        bm = om.MMatrix()
                        om.MScriptUtil.createMatrixFromList(attr,bm)
                        bm = bm.homogenize()

                        cm = am * bm.inverse()
                        ctm = om.MTransformationMatrix( cm )
                        cquat = ctm.rotation()

                        scaleUtil = om.MScriptUtil()
                        scaleUtil.createFromDouble(0)
                        t = scaleUtil.asDoublePtr()
                        v = om.MVector()

                        cquat.getAxisAngle( v, t )

                        v = v * bm

                        cmds.setAttr( fNode+'.rvx', v.x )
                        cmds.setAttr( fNode+'.rvy', v.y )
                        cmds.setAttr( fNode+'.rvz', v.z )
                        cmds.setAttr( fNode+'.rs', math.degrees(om.MScriptUtil.getDouble(t)*jb_poseBlender.fps/100 ) )



                        liveAnim = cmds.listConnections( (fNode+'.li') )
                        if cmds.nodeType( liveAnim ) == 'animCurveTU':
                            cmds.delete( liveAnim )

                        cmds.setKeyframe( fNode, v=1, at='liveInput', t=time )
                        cmds.setKeyframe( fNode, v=0, at='liveInput', t=time+1 )
                        cmds.setAttr( fNode+'.startTime', time )


                    else:
                        print 'Source object could not be found from fakeDynamics node'
                else:
                    print 'No fakeDynamics node found on selected transform'
            else:
                print 'Selected object is neither a transform or a fakeDynamics node'
'''

class jb_poseBlenderAddPose(omMPx.MPxCommand):
    kPluginCmdName = "jb_poseBlenderAddPose"

    def __init__(self):
        omMPx.MPxCommand.__init__(self)

    @staticmethod
    def cmdCreator():
        return omMPx.asMPxPtr( jb_poseBlenderAddPose() )

    def doIt(self,argList):
        import maya.cmds as cmds
        for obj in cmds.ls( sl=True ):
            fNode = 'jb_poseBlender1'

            sel = om.MSelectionList()
            om.MGlobal.getSelectionListByName( 'jb_poseBlender1',sel)

            obj = om.MObject()
            sel.getDependNode(0,obj)
            #print obj.addPose
            #obj.addPose
            print jb_poseBlender.addPose(obj)
            #cmds.connectAttr( obj+'.matrix', fNode+'.inMatrix' )
            #cmds.setAttr( fNode+'.st', cmds.currentTime( q=True ) )
            #cmds.connectAttr( 'time1.outTime', fNode+'.time' )
            #cmds.connectAttr( fNode+'.translate', dynObj+'.translate' )
            #cmds.connectAttr( fNode+'.rotate', dynObj+'.rotate' )
            #cmds.connectAttr( fNode+'.scale', dynObj+'.scale' )
            #cmds.connectAttr( fNode+'.shear', dynObj+'.shear' )
            #cmds.connectAttr( dynObj+'.ro', fNode+'.ro' )

def addPose():
    uAttr = om.MFnUnitAttribute()
    jb_poseBlender.time = uAttr.create("time", "tm", om.MFnUnitAttribute.kTime, 0.0)
    uAttr.setHidden(True)
    jb_poseBlender.addAttribute(jb_poseBlender.time)

# build the main plugin class
class jb_poseBlender(omMPx.MPxNode):
    kPluginNodeName = "jb_poseBlender"
    kPluginNodeId = om.MTypeId(0x3AB27)

    #initialVelocity = om.MObject()
    #initialRotation = om.MObject()
    #inMatrix = om.MObject()
    #outMatrix = om.MObject()
    poses = []

    # do a cached version of the input transform matrix
    #inMatrixCache = om.MFloatMatrix()
    #fps = 0

    def __init__(self):
        omMPx.MPxNode.__init__(self)
        print self

    def compute(self, plug, data):
        import maya.cmds as cmds
        import math
        print self
        '''
        # get the variables from the attributes
        time = data.inputValue(jb_poseBlender.time).asTime().value()
        start = data.inputValue(jb_poseBlender.startTime).asTime().value()
        gravity = data.inputValue(jb_poseBlender.gravity).asFloat()
        live = data.inputValue(jb_poseBlender.liveInput).asBool()
        gv = data.inputValue(jb_poseBlender.gv).asFloat3()
        velocity = data.inputValue(jb_poseBlender.iv).asFloat3()
        rotationVector = data.inputValue(jb_poseBlender.rv).asFloat3()
        rotationSpeed = data.inputValue(jb_poseBlender.rs).asFloat()
        rotationOrder = data.inputValue(jb_poseBlender.ro).asInt()
        translate = data.outputValue(jb_poseBlender.translate)
        rx = data.outputValue(jb_poseBlender.rx)
        ry = data.outputValue(jb_poseBlender.ry)
        rz = data.outputValue(jb_poseBlender.rz)
        scale = data.outputValue(jb_poseBlender.scale)
        shear = data.outputValue(jb_poseBlender.shear)
        matrix = data.inputValue(jb_poseBlender.inMatrix).asFloatMatrix()

        # if the live attribute is set to "Live" then refresh the cached matrix attribute with the one from the input channel
        if live:
            jb_poseBlender.inMatrixCache = om.MFloatMatrix(matrix)
        inMatrix = jb_poseBlender.inMatrixCache

        # get the desired fps
        unit = om.MTime.uiUnit()
        fps = 0
        if unit == 6: fps = 24
        elif unit == 7: fps = 25
        elif unit == 8: fps = 30
        jb_poseBlender.fps = fps

        # create a  unit matrix to manipulate
        matrix = om.MFloatMatrix()

        # get the current matrix components from the input matrix
        pos = om.MVector( inMatrix(3,0), inMatrix(3,1), inMatrix(3,2) )
        x = om.MVector( inMatrix(0,0), inMatrix(0,1), inMatrix(0,2) )
        y = om.MVector( inMatrix(1,0), inMatrix(1,1), inMatrix(1,2) )
        z = om.MVector( inMatrix(2,0), inMatrix(2,1), inMatrix(2,2) )

        # it the current time is larger than the start time condition, run some computations
        if time > start:
            # get the time delta from the start time
            timeDelta = time - start

            v = om.MVector( gv[0], gv[1], gv[2] )
            if v.length() == 0:
                v.y = -1
            v.normalize()

            # modify the positional vector with gravity and initial speed
            px = pos.x+((gravity*(timeDelta*timeDelta)*v.x)/fps)+(velocity[0]*timeDelta)
            py = pos.y+((gravity*(timeDelta*timeDelta)*v.y)/fps)+(velocity[1]*timeDelta)
            pz = pos.z+((gravity*(timeDelta*timeDelta)*v.z)/fps)+(velocity[2]*timeDelta)
            pos = om.MVector( px, py, pz )

            # get the vector to rotate around
            vector = om.MVector( rotationVector[0], rotationVector[1], rotationVector[2] )
            vector.normalize()

            # build a rotated matrix to multiply the vectors with
            quat = om.MQuaternion()
            quat = quat.setAxisAngle( vector, math.radians(rotationSpeed*timeDelta/fps*100) )
            rot = quat.asMatrix()

            # do the actual rotation multiplication
            x = x * rot
            y = y * rot
            z = z * rot


        # apply the x vector to the matrix
        om.MScriptUtil.setFloatArray(matrix[0], 0, x.x)
        om.MScriptUtil.setFloatArray(matrix[0], 1, x.y)
        om.MScriptUtil.setFloatArray(matrix[0], 2, x.z)

        # apply the y vector to the matrix
        om.MScriptUtil.setFloatArray(matrix[1], 0, y.x)
        om.MScriptUtil.setFloatArray(matrix[1], 1, y.y)
        om.MScriptUtil.setFloatArray(matrix[1], 2, y.z)

        # apply the z vector to the matrix
        om.MScriptUtil.setFloatArray(matrix[2], 0, z.x)
        om.MScriptUtil.setFloatArray(matrix[2], 1, z.y)
        om.MScriptUtil.setFloatArray(matrix[2], 2, z.z)


        # apply the translationpart of the matrix
        om.MScriptUtil.setFloatArray(matrix[3], 0, pos.x)
        om.MScriptUtil.setFloatArray(matrix[3], 1, pos.y)
        om.MScriptUtil.setFloatArray(matrix[3], 2, pos.z)

        # rebuild the float matrix as a MMatrix
        matrix = om.MMatrix( matrix.matrix )
        # rebuild the matrix as a transformationmatrix
        tm = om.MTransformationMatrix( matrix )
        # get the quaternion from the matrix
        mquat = tm.rotation()
        # get the euler rotation from the quaternion
        rot = mquat.asEulerRotation()
        # rebuild the euler to the selected rotation order
        rot.reorderIt( rotationOrder )


        # set the translation attribute
        translate.set3Float(pos.x, pos.y, pos.z)

        # set the rotational attributes (individual since they are angles and must be set with setMAngle and cannot be set with set3Float)
        rx.setMAngle(om.MAngle(rot.x))
        ry.setMAngle(om.MAngle(rot.y))
        rz.setMAngle(om.MAngle(rot.z))

        # build an array or something.. don't really know what this shit does
        scaleUtil = om.MScriptUtil()
        scaleUtil.createFromList([0,0,0],3)
        scaleVec = scaleUtil.asDoublePtr()

        # but its used here so i can extract the scael components out of the transformation matrix
        tm.getScale( scaleVec, om.MSpace.kTransform )
        scl = [om.MScriptUtil.getDoubleArrayItem(scaleVec,i) for i in range(0,3)]
        scale.set3Float( scl[0], scl[1], scl[2] )

        # and the same with the shearing
        tm.getShear( scaleVec, om.MSpace.kTransform )
        shr = [om.MScriptUtil.getDoubleArrayItem(scaleVec,i) for i in range(0,3)]
        shear.set3Float( shr[0], shr[1], shr[2] )

        '''
        # and lastly clean the plug
        data.setClean(plug)

    def addPose(self):
        print len(poses)
        jb_poseBlender.time = uAttr.create("time", "tm", om.MFnUnitAttribute.kTime, 0.0)
        uAttr.setHidden(True)


    @classmethod
    def nodeCreator(self):
        return omMPx.asMPxPtr( jb_poseBlender() )

    @classmethod
    def nodeInitializer(self):
        uAttr = om.MFnUnitAttribute()
        nAttr = om.MFnNumericAttribute()
        cAttr = om.MFnCompoundAttribute()
        tAttr = om.MFnTypedAttribute()

        # build the pose attribute
        jb_poseBlender.pose = nAttr.create('pose', 'p', om.MFnNumericData.kDouble, 0.0 )
        nAttr.setStorable(True)
        nAttr.setArray(True)
        jb_poseBlender.addAttribute(jb_poseBlender.pose)

        # build the input attribute
        jb_poseBlender.input = nAttr.create('input', 'in', om.MFnNumericData.kFloat, 0 )
        nAttr.setKeyable(True)
        nAttr.setArray(True)
        jb_poseBlender.addAttribute(jb_poseBlender.input)

        # build the output attribute
        jb_poseBlender.output = nAttr.create('output', 'out', om.MFnNumericData.kFloat, 0 )
        nAttr.setKeyable(True)
        nAttr.setArray(True)
        jb_poseBlender.addAttribute(jb_poseBlender.output)


    def setOutputs(self):
        print 'bla'

# initialize the script plug-in
def initializePlugin(mobject):
    import sys
    # set some basic information about the plugin
    omMPx.MFnPlugin(mobject, 'Jonas Borgman', '1.0', 'Any')

    mplugin = omMPx.MFnPlugin(mobject)

    # register the node
    try:
        mplugin.registerNode( jb_poseBlender.kPluginNodeName, jb_poseBlender.kPluginNodeId, jb_poseBlender.nodeCreator, jb_poseBlender.nodeInitializer)

        mplugin.registerCommand( jb_poseBlenderAddPose.kPluginCmdName, jb_poseBlenderAddPose.cmdCreator )
        #mplugin.registerCommand( jb_removeFakeDynamics.kPluginCmdName, jb_removeFakeDynamics.cmdCreator )
        #mplugin.registerCommand( jb_setFakeDynamics.kPluginCmdName, jb_setFakeDynamics.cmdCreator )
        buildMenu()

    except:
        sys.stderr.write( "Failed to register node: %s" % jb_poseBlender.kPluginNodeName )
        raise


# uninitialize the script plug-in
def uninitializePlugin(mobject):
    mplugin = omMPx.MFnPlugin(mobject)
    try:
        import maya.cmds as cmds
        mplugin.deregisterNode( jb_poseBlender.kPluginNodeId )
        mplugin.deregisterCommand(jb_poseBlenderAddPose.kPluginCmdName)
        #mplugin.deregisterCommand(jb_removeFakeDynamics.kPluginCmdName)
        #mplugin.deregisterCommand(jb_setFakeDynamics.kPluginCmdName)
        cmds.deleteUI( 'jb_poseBlenderMenu' )
        mayaMain.removeToolsMenu('Plugins')
    except:
        import sys
        sys.stderr.write( "Failed to deregister jb_poseBlender" )
        raise

def buildMenu():
    import maya.cmds as cmds

    mayaMain.addToolsMenu('Plugins')
    allMenus = cmds.lsUI( type='menu' )
    pluginMenu = None
    i = 0
    while pluginMenu == None and i < len(systems):
        if systems[i] in allMenus:
            pluginMenu = systems[i]
        i += 1


    if pluginMenu:
        cmds.menuItem( 'jb_poseBlenderMenu', label='Pose Blender', subMenu=True, parent=pluginMenu )
        cmds.menuItem( 'jb_poseBlenderAdd', label='Add to object', parent='jb_poseBlenderMenu', c=cmds.jb_applyFakeDynamics )
        cmds.menuItem( 'jb_poseBlenderRemove', label='Remove from object', parent='jb_poseBlenderMenu', c=cmds.jb_removeFakeDynamics )
        cmds.menuItem( 'jb_poseBlenderSetActive', label='Make Dynamic On Frame', parent='jb_poseBlenderMenu', c=cmds.jb_setFakeDynamics )

        cmds.setParent( 'MayaWindow' )
