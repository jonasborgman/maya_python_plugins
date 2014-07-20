import maya.OpenMaya as om
import maya.OpenMayaMPx as omMPx
from internalPackages.maya import mayaMain

systems = ('jbPlugins', 'mec_Plugins')

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
            fNode = cmds.createNode( 'jb_fakeDynamics' )
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
                if type == 'jb_fakeDynamics' or type == 'transform':
                    fNode = obj
                    oNode = obj
                    if type == 'transform':
                        fNode = cmds.listConnections( obj, t='jb_fakeDynamics' )
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
            if type == 'jb_fakeDynamics' or type == 'transform':
                fNode = obj
                oNode = obj
                if type == 'transform':
                    fNode = cmds.listConnections( obj, t='jb_fakeDynamics' )
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
                        cmds.setAttr( fNode+'.rs', math.degrees(om.MScriptUtil.getDouble(t)*jb_fakeDynamics.fps/100 ) )



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

# build the main plugin class
class jb_fakeDynamics(omMPx.MPxNode):
    kPluginNodeName = "jb_fakeDynamics"
    kPluginNodeId = om.MTypeId(0x3AB21)

    time = om.MObject()
    startTime = om.MObject()
    gravity = om.MObject()
    initialVelocity = om.MObject()
    initialRotation = om.MObject()
    inMatrix = om.MObject()
    outMatrix = om.MObject()

    # do a cached version of the input transform matrix
    inMatrixCache = om.MFloatMatrix()
    fps = 0

    def __init__(self):
        omMPx.MPxNode.__init__(self)


    def compute(self, plug, data):
        import maya.cmds as cmds
        import math

        # get the variables from the attributes
        time = data.inputValue(jb_fakeDynamics.time).asTime().value()
        start = data.inputValue(jb_fakeDynamics.startTime).asTime().value()
        gravity = data.inputValue(jb_fakeDynamics.gravity).asFloat()
        live = data.inputValue(jb_fakeDynamics.liveInput).asBool()
        gv = data.inputValue(jb_fakeDynamics.gv).asFloat3()
        velocity = data.inputValue(jb_fakeDynamics.iv).asFloat3()
        rotationVector = data.inputValue(jb_fakeDynamics.rv).asFloat3()
        rotationSpeed = data.inputValue(jb_fakeDynamics.rs).asFloat()
        rotationOrder = data.inputValue(jb_fakeDynamics.ro).asInt()
        translate = data.outputValue(jb_fakeDynamics.translate)
        rx = data.outputValue(jb_fakeDynamics.rx)
        ry = data.outputValue(jb_fakeDynamics.ry)
        rz = data.outputValue(jb_fakeDynamics.rz)
        scale = data.outputValue(jb_fakeDynamics.scale)
        shear = data.outputValue(jb_fakeDynamics.shear)
        matrix = data.inputValue(jb_fakeDynamics.inMatrix).asFloatMatrix()

        # if the live attribute is set to "Live" then refresh the cached matrix attribute with the one from the input channel
        if live:
            jb_fakeDynamics.inMatrixCache = om.MFloatMatrix(matrix)
        inMatrix = jb_fakeDynamics.inMatrixCache

        # get the desired fps
        unit = om.MTime.uiUnit()
        fps = 0
        if unit == 6: fps = 24
        elif unit == 7: fps = 25
        elif unit == 8: fps = 30
        jb_fakeDynamics.fps = fps

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


        # and lastly clean the plug
        data.setClean(plug)

def nodeCreator():
    return omMPx.asMPxPtr( jb_fakeDynamics() )

def nodeInitializer():
    uAttr = om.MFnUnitAttribute()
    nAttr = om.MFnNumericAttribute()
    mAttr = om.MFnMatrixAttribute()
    cAttr = om.MFnCompoundAttribute()
    eAttr = om.MFnEnumAttribute()

    # create the attributes for the node
    jb_fakeDynamics.time = uAttr.create("time", "tm", om.MFnUnitAttribute.kTime, 0.0)
    uAttr.setHidden(True)
    jb_fakeDynamics.startTime = uAttr.create( 'startTime', 'st', om.MFnUnitAttribute.kTime, 0.0 )
    uAttr.setChannelBox(True)
    uAttr.setKeyable(False)
    jb_fakeDynamics.gravity = nAttr.create( 'gravity', 'g', om.MFnNumericData.kFloat, 9.82 )
    nAttr.setChannelBox(True)
    nAttr.setKeyable(False)
    jb_fakeDynamics.liveInput = nAttr.create( 'liveInput', 'li', om.MFnNumericData.kBoolean, True )
    nAttr.setKeyable(True)

    # build the gravitational vector attribute
    jb_fakeDynamics.gvx = nAttr.create( 'gravitationVectorX', 'gvx', om.MFnNumericData.kFloat, 0 )
    jb_fakeDynamics.gvy = nAttr.create( 'gravitationVectorY', 'gvy', om.MFnNumericData.kFloat, -1 )
    jb_fakeDynamics.gvz = nAttr.create( 'gravitationVectorZ', 'gvz', om.MFnNumericData.kFloat, 0 )
    jb_fakeDynamics.gv = cAttr.create( 'gravitationVector', 'gv' )
    cAttr.addChild ( jb_fakeDynamics.gvx )
    cAttr.addChild ( jb_fakeDynamics.gvy )
    cAttr.addChild ( jb_fakeDynamics.gvz )

    # build the initial velocity attribute
    jb_fakeDynamics.ivx = nAttr.create( 'initialVelocityX', 'ivx', om.MFnNumericData.kFloat )
    jb_fakeDynamics.ivy = nAttr.create( 'initialVelocityY', 'ivy', om.MFnNumericData.kFloat )
    jb_fakeDynamics.ivz = nAttr.create( 'initialVelocityZ', 'ivz', om.MFnNumericData.kFloat )
    jb_fakeDynamics.iv = cAttr.create( 'initialVelocity', 'iv' )
    cAttr.addChild ( jb_fakeDynamics.ivx )
    cAttr.addChild ( jb_fakeDynamics.ivy )
    cAttr.addChild ( jb_fakeDynamics.ivz )

    # build the rotation vector attribute
    jb_fakeDynamics.rvx = nAttr.create( 'rotationVectorX', 'rvx', om.MFnNumericData.kFloat, 1 )
    jb_fakeDynamics.rvy = nAttr.create( 'rotationVectorY', 'rvy', om.MFnNumericData.kFloat )
    jb_fakeDynamics.rvz = nAttr.create( 'rotationVectorZ', 'rvz', om.MFnNumericData.kFloat )
    jb_fakeDynamics.rv = cAttr.create( 'rotationVector', 'rv' )
    cAttr.addChild ( jb_fakeDynamics.rvx )
    cAttr.addChild ( jb_fakeDynamics.rvy )
    cAttr.addChild ( jb_fakeDynamics.rvz )

    jb_fakeDynamics.rs = nAttr.create( 'rotationSpeed', 'rs', om.MFnNumericData.kFloat )
    nAttr.setChannelBox(True)
    nAttr.setKeyable(False)
    jb_fakeDynamics.ro = eAttr.create( 'rotationOrder', 'ro' )
    eAttr.addField('xyz', 0)
    eAttr.addField('yzx', 1)
    eAttr.addField('zxy', 2)
    eAttr.addField('xzy', 3)
    eAttr.addField('yxz', 4)
    eAttr.addField('zyx', 5)
    eAttr.setKeyable(False)

    jb_fakeDynamics.inMatrix = mAttr.create('inMatrix', 'im', om.MFnMatrixAttribute.kFloat)
    mAttr.setHidden(True)

    # build the translation attribute
    jb_fakeDynamics.tx = nAttr.create( 'translateX', 'tx', om.MFnNumericData.kFloat )
    jb_fakeDynamics.ty = nAttr.create( 'translateY', 'ty', om.MFnNumericData.kFloat )
    jb_fakeDynamics.tz = nAttr.create( 'translateZ', 'tz', om.MFnNumericData.kFloat )
    jb_fakeDynamics.translate = cAttr.create( 'translate', 't' )
    cAttr.addChild( jb_fakeDynamics.tx )
    cAttr.addChild( jb_fakeDynamics.ty )
    cAttr.addChild( jb_fakeDynamics.tz )
    cAttr.setHidden(True)

    # build the rotation attribute
    jb_fakeDynamics.rx = uAttr.create( 'rotateX', 'rx', om.MFnUnitAttribute.kAngle )
    jb_fakeDynamics.ry = uAttr.create( 'rotateY', 'ry', om.MFnUnitAttribute.kAngle )
    jb_fakeDynamics.rz = uAttr.create( 'rotateZ', 'rz', om.MFnUnitAttribute.kAngle )
    jb_fakeDynamics.rotate = cAttr.create( 'rotate', 'r' )
    cAttr.addChild( jb_fakeDynamics.rx )
    cAttr.addChild( jb_fakeDynamics.ry )
    cAttr.addChild( jb_fakeDynamics.rz )
    cAttr.setHidden(True)

    # build the scale attribute
    jb_fakeDynamics.sx = nAttr.create( 'scaleX', 'sx', om.MFnNumericData.kFloat, 1 )
    jb_fakeDynamics.sy = nAttr.create( 'scaleY', 'sy', om.MFnNumericData.kFloat, 1 )
    jb_fakeDynamics.sz = nAttr.create( 'scaleZ', 'sz', om.MFnNumericData.kFloat, 1 )
    jb_fakeDynamics.scale = cAttr.create( 'scale', 's' )
    cAttr.addChild( jb_fakeDynamics.sx )
    cAttr.addChild( jb_fakeDynamics.sy )
    cAttr.addChild( jb_fakeDynamics.sz )
    cAttr.setHidden(True)

    # build the shear attribute
    jb_fakeDynamics.shearx = nAttr.create( 'shearX', 'shx', om.MFnNumericData.kFloat )
    jb_fakeDynamics.sheary = nAttr.create( 'shearY', 'shy', om.MFnNumericData.kFloat )
    jb_fakeDynamics.shearz = nAttr.create( 'shearZ', 'shz', om.MFnNumericData.kFloat )
    jb_fakeDynamics.shear = cAttr.create( 'shear', 'sh' )
    cAttr.addChild( jb_fakeDynamics.shearx )
    cAttr.addChild( jb_fakeDynamics.sheary )
    cAttr.addChild( jb_fakeDynamics.shearz )
    cAttr.setHidden(True)


    # add the attributes to the node
    jb_fakeDynamics.addAttribute(jb_fakeDynamics.time)
    jb_fakeDynamics.addAttribute(jb_fakeDynamics.startTime)
    jb_fakeDynamics.addAttribute(jb_fakeDynamics.gravity)
    jb_fakeDynamics.addAttribute(jb_fakeDynamics.liveInput)
    jb_fakeDynamics.addAttribute(jb_fakeDynamics.gv)
    jb_fakeDynamics.addAttribute(jb_fakeDynamics.iv)
    jb_fakeDynamics.addAttribute(jb_fakeDynamics.rv)
    jb_fakeDynamics.addAttribute(jb_fakeDynamics.rs)
    jb_fakeDynamics.addAttribute(jb_fakeDynamics.inMatrix)
    jb_fakeDynamics.addAttribute(jb_fakeDynamics.translate)
    jb_fakeDynamics.addAttribute(jb_fakeDynamics.rotate)
    jb_fakeDynamics.addAttribute(jb_fakeDynamics.scale)
    jb_fakeDynamics.addAttribute(jb_fakeDynamics.shear)
    jb_fakeDynamics.addAttribute(jb_fakeDynamics.ro)

    # set the update rules for the node
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.time, jb_fakeDynamics.translate)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.startTime, jb_fakeDynamics.translate)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.gravity, jb_fakeDynamics.translate)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.liveInput, jb_fakeDynamics.translate)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.gv, jb_fakeDynamics.translate)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.iv, jb_fakeDynamics.translate)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.rv, jb_fakeDynamics.translate)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.rs, jb_fakeDynamics.translate)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.ro, jb_fakeDynamics.translate)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.inMatrix, jb_fakeDynamics.translate)

    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.time, jb_fakeDynamics.rotate)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.startTime, jb_fakeDynamics.rotate)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.gravity, jb_fakeDynamics.rotate)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.liveInput, jb_fakeDynamics.rotate)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.gv, jb_fakeDynamics.rotate)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.iv, jb_fakeDynamics.rotate)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.rv, jb_fakeDynamics.rotate)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.rs, jb_fakeDynamics.rotate)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.ro, jb_fakeDynamics.rotate)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.inMatrix, jb_fakeDynamics.rotate)

    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.time, jb_fakeDynamics.scale)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.startTime, jb_fakeDynamics.scale)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.gravity, jb_fakeDynamics.scale)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.liveInput, jb_fakeDynamics.scale)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.gv, jb_fakeDynamics.scale)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.iv, jb_fakeDynamics.scale)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.rv, jb_fakeDynamics.scale)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.rs, jb_fakeDynamics.scale)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.ro, jb_fakeDynamics.scale)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.inMatrix, jb_fakeDynamics.scale)

    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.time, jb_fakeDynamics.shear)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.startTime, jb_fakeDynamics.shear)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.gravity, jb_fakeDynamics.shear)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.liveInput, jb_fakeDynamics.shear)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.gv, jb_fakeDynamics.shear)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.iv, jb_fakeDynamics.shear)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.rv, jb_fakeDynamics.shear)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.rs, jb_fakeDynamics.shear)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.ro, jb_fakeDynamics.shear)
    jb_fakeDynamics.attributeAffects(jb_fakeDynamics.inMatrix, jb_fakeDynamics.shear)

# initialize the script plug-in
def initializePlugin(mobject):
    import sys
    # set some basic information about the plugin
    omMPx.MFnPlugin(mobject, 'Jonas Borgman', '1.0', 'Any')

    mplugin = omMPx.MFnPlugin(mobject)

    # register the node
    try:
        mplugin.registerNode( jb_fakeDynamics.kPluginNodeName, jb_fakeDynamics.kPluginNodeId, nodeCreator, nodeInitializer)

        mplugin.registerCommand( jb_applyFakeDynamics.kPluginCmdName, jb_applyFakeDynamics.cmdCreator )
        mplugin.registerCommand( jb_removeFakeDynamics.kPluginCmdName, jb_removeFakeDynamics.cmdCreator )
        mplugin.registerCommand( jb_setFakeDynamics.kPluginCmdName, jb_setFakeDynamics.cmdCreator )
        buildMenu()

    except:
        sys.stderr.write( "Failed to register node: %s" % jb_fakeDynamics.kPluginNodeName )
        raise


# uninitialize the script plug-in
def uninitializePlugin(mobject):
    mplugin = omMPx.MFnPlugin(mobject)
    try:
        import maya.cmds as cmds
        mplugin.deregisterNode( jb_fakeDynamics.kPluginNodeId )
        mplugin.deregisterCommand(jb_applyFakeDynamics.kPluginCmdName)
        mplugin.deregisterCommand(jb_removeFakeDynamics.kPluginCmdName)
        mplugin.deregisterCommand(jb_setFakeDynamics.kPluginCmdName)
        cmds.deleteUI( 'jb_fakeDynamicsMenu' )
        mayaMain.removeToolsMenu('Plugins')

    except:
        import sys
        sys.stderr.write( "Failed to deregister jb_fakeDynamics" )
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
        cmds.menuItem( 'jb_fakeDynamicsMenu', label='Faked Dynamics', subMenu=True, parent=pluginMenu )
        cmds.menuItem( 'jb_fakeDynamicsAdd', label='Add to object', parent='jb_fakeDynamicsMenu', c=cmds.jb_applyFakeDynamics )
        cmds.menuItem( 'jb_fakeDynamicsRemove', label='Remove from object', parent='jb_fakeDynamicsMenu', c=cmds.jb_removeFakeDynamics )
        cmds.menuItem( 'jb_fakeDynamicsSetActive', label='Make Dynamic On Frame', parent='jb_fakeDynamicsMenu', c=cmds.jb_setFakeDynamics )

        cmds.setParent( 'MayaWindow' )
