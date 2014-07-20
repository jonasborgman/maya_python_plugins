
import maya.OpenMaya as om
import maya.OpenMayaMPx as omMPx
import maya.OpenMayaAnim as omAnim
import maya.OpenMayaUI as apiUI


systems = ('jbPlugins', 'mec_Plugins')
import maya.cmds as mc
import sys, os, math, copy, os

from PyQt4 import QtGui, QtCore, uic
from string import find
from internalPackages.maya import mayaMain


# create the command that creates the node
class jb_applyMotReader(omMPx.MPxCommand):
    kPluginCmdName = "jb_applyMotReader"

    def __init__(self):
        omMPx.MPxCommand.__init__(self)

    @staticmethod
    def cmdCreator():
        return omMPx.asMPxPtr( jb_applyMotReader() )

    def doIt(self,argList):
        import maya.cmds as cmds
        objs = cmds.ls( sl=True )
        if len(objs) == 1:
            obj = objs[0]
            node = cmds.createNode( 'jb_motReader' )
            cmds.connectAttr( 'time1.outTime', node+'.time' )
            cmds.connectAttr( node+'.translate', obj+'.translate' )
            cmds.connectAttr( node+'.rotate', obj+'.rotate' )
            cmds.connectAttr( node+'.scale', obj+'.scale' )
            cmds.setAttr( obj+'.rotateOrder', 2 )
            cmds.select( obj )
            
        else:
            print 'Select exactly one object to apply motion node to'

# bake motionData
class jb_exportMotFile(omMPx.MPxCommand):
    kPluginCmdName = "jb_exportMotionData"

    def __init__(self):
        omMPx.MPxCommand.__init__(self)

    @staticmethod
    def cmdCreator():
        return omMPx.asMPxPtr( jb_exportMotFile() )


    def doIt(self, argList):
        from sys import argv

        
        path = 'T:/Project_Resources/scripts/MEC_Tools2.0/TOOLS/Maya/Python/Pipeline/motionData'

        # fetch the ui file and process it
        uifile = path+'/ui/motionDataMain.ui'
        uifile = 'C:/Users/Jonas/Desktop/motionDataMain.ui'

        mayaMainWindow = QtGui.QApplication.activeWindow()
        while mayaMainWindow.parentWidget() != None:
            mayaMainWindow = mayaMainWindow.parentWidget()


        win = jb_motionBakerWin(mayaMainWindow, uifile)
        win.show()

def getVersion( fileName):
    i = 0
    versions = []
    while i < len(fileName):
        if fileName[i].lower() == 'v':
            pos = i
            number = fileName[i]
            i += 1
            while fileName[i].isdigit() and i < len(fileName):
                number += fileName[i]
                i += 1
            
            versions.append((number, int(number[1:]), pos))

        else:
            i += 1
    if len(versions) == 1:
        return versions[0]
    else:
        return None

class jb_motionBakerWin(QtGui.QDialog):
    def __init__(self, parent, uifile):
        QtGui.QDialog.__init__(self, parent)
        

        self.buildUi()

        self.bakeBtn.pressed.connect(self.bakeObjects)
        self.browseBtn.pressed.connect(self.setPath)
        self.worldBtn.pressed.connect(self.wsButton)

    def buildVersionString(self, version):
        output = str(version)
        while len(output) < 3:
            output = '0' + output
        output = 'v' + output
        return output


    def buildUi(self):
        self.setWindowTitle('Motion Baker')

        mainLayout = QtGui.QVBoxLayout()
        mainLayout.setSpacing(0)

        
        browseLayout = QtGui.QHBoxLayout()
        self.pathField = QtGui.QLineEdit()
        self.pathField.setText('C:\Users\Jonas\Desktop')
        browseLayout.addWidget(self.pathField)
        self.browseBtn = QtGui.QToolButton()
        browseLayout.addWidget(self.browseBtn)
        mainLayout.addLayout(browseLayout)

        #self.previewText = QtGui.QLineEdit()
        #self.previewText.setReadOnly(True)
        #mainLayout.addWidget(self.previewText)


        versionLayout = QtGui.QHBoxLayout()
        self.versioningBtn = QtGui.QPushButton()
        self.versioningBtn.setText('Versioning')
        self.versioningBtn.setCheckable(True)
        #self.versioningBtn.setChecked(True)
        versionLayout.addWidget(self.versioningBtn)
        self.newVersionButton = QtGui.QPushButton()
        self.newVersionButton.setText('New Version')
        self.newVersionButton.setCheckable(True)
        versionLayout.addWidget(self.newVersionButton)
        mainLayout.addLayout(versionLayout)


        optionLayout = QtGui.QHBoxLayout()
        self.worldBtn = QtGui.QPushButton()
        self.worldBtn.setSizePolicy(3,3)
        self.worldBtn.setCheckable(True)
        self.worldBtn.setChecked(True)
        self.worldBtn.setText('Worldspace')
        optionLayout.addWidget(self.worldBtn)
        self.optimizeBtn = QtGui.QPushButton()
        self.optimizeBtn.setSizePolicy(3,3)
        self.optimizeBtn.setCheckable(True)
        self.optimizeBtn.setChecked(True)
        self.optimizeBtn.setText('Optimize')
        optionLayout.addWidget(self.optimizeBtn)
        mainLayout.addLayout(optionLayout)
        mainLayout.setStretch(1, 0.5)

        self.bakeBtn = QtGui.QPushButton()
        self.bakeBtn.setText('BAKE')
        self.bakeBtn.setSizePolicy(3,3)
        mainLayout.addWidget(self.bakeBtn)

        self.progressBar = QtGui.QProgressBar()
        mainLayout.addWidget(self.progressBar)

        self.setLayout(mainLayout)
     
    def wsButton(self):
        
        text = 'LocalSpace'
        if not self.worldBtn.isChecked():
            text = 'WorldSpace'
            
        self.worldBtn.setText( text )

    def getTimeInfo(self):
        min = omAnim.MAnimControl.minTime()
        min = min.asUnits(min.unit())
        max = omAnim.MAnimControl.maxTime()
        max = max.asUnits(max.unit())
        
        tc = om.MTime().unit()
        fps = None
        if tc == 8: fps = 30.0
        elif tc == 7: fps = 25.0
        elif tc == 6: fps = 24.0

        return(min, max, fps, tc)

    def getNodeWorldMatrix(self, node, world):
        fnThisNode = om.MFnDependencyNode(node)
        matrixAttr = fnThisNode.attribute("matrix")
        if world: matrixAttr = fnThisNode.attribute("worldMatrix")
        matrixPlug = om.MPlug( node, matrixAttr )
        if world: matrixPlug = matrixPlug.elementByLogicalIndex( 0 )
        matrixObject = matrixPlug.asMObject(  )
        matrix = om.MFnMatrixData( matrixObject )

        worldMatrixData = om.MFnMatrixData( matrixObject )
        worldMatrix = worldMatrixData.matrix( )
        return worldMatrix

    def decompMatrix(self, matrix):
        transformMatrix = om.MTransformationMatrix(matrix)
        
        trans = transformMatrix.getTranslation(om.MSpace.kWorld)
        
        rotateOrder = 2
        quat = transformMatrix.rotation()
        rot = quat.asEulerRotation()
        rot.reorderIt( rotateOrder )
        
        scaleUtil = om.MScriptUtil()
        scaleUtil.createFromDouble(0, 0, 0)
        scalePtr = scaleUtil.asDoublePtr();
        transformMatrix.getScale(scalePtr, om.MSpace.kWorld)
        scale = [om.MScriptUtil.getDoubleArrayItem(scalePtr, i) for i in (0,1,2)]
        
        list = [trans.x,trans.y,trans.z, rot[0],rot[1],rot[2], scale[0],scale[1],scale[2]]
        list = [float(("%.5f" % i)) for i in list]
        return list

    def optimizeData(self, data, optimize):
        temp = []
        for i in range(9):
            chan = []
            for t in range(len(data)):
                chan.append(data[t][i])
            temp.append( chan )
        data = copy.copy(temp)

        if optimize:
            for i in range(9):
                go = True
                end = len(data[i])-1
                while end > 0 and go:
                    if data[i][end] == data[i][end-1]:
                        data[i][end] = None
                        end = end-1
                    else:
                        go = False
                go = True
                start = 0
                while start < len(data[i]) and go:
                    if start < len(data[i])-1:
                        if data[i][start] == data[i][start+1]:
                            data[i][start] = None
                            start = start+1
                        else:
                            go = False
        return data

    def setPath(self):
        self.pathField.setText(QtGui.QFileDialog.getExistingDirectory ())
        self.checkVersions()

    def getObjFromList(self, list, num):

        obj = om.MObject()
        list.getDependNode(num,obj)
        return obj


    def bakeObjects(self):
        path = str(self.pathField.text())
        if os.path.isdir( path ):
            objs = om.MSelectionList()
            om.MGlobal.getActiveSelectionList(objs)

            min, max, fps, tc = self.getTimeInfo()

            objData = []
            for i in range(objs.length()): objData.append( [] )

            for p in range(objs.length()):
                obj = self.getObjFromList( objs, p )
                thisObj = []

                for i in range(int(min), int(max)+1):
                    time = om.MTime()
                    time.setUnit(tc)
                    time.setValue(i)
                    omAnim.MAnimControl.setCurrentTime(time)

                    world = self.worldBtn.isChecked()
                    matrix = self.getNodeWorldMatrix(obj, world)
                    values = self.decompMatrix(matrix)
                    thisObj.append( values )

                objData[p] = thisObj

            
            for o in range(len(objData)):
                obj = self.getObjFromList( objs, o )
                thisObjData = objData[o]
                optimize = self.optimizeBtn.isChecked()
                thisObjData = self.optimizeData(thisObjData, optimize)
                objName = om.MFnDependencyNode(obj).name()

                if self.versioningBtn.isChecked():
                    lastVersion = 1
                    for motFile in os.listdir(path):
                        if motFile.split('.')[-1] == 'mot':
                            if motFile.startswith(objName):
                                fileVersion = getVersion(motFile)
                                if fileVersion and fileVersion[1] > lastVersion:
                                    lastVersion = fileVersion[1]

                    if self.newVersionButton.isChecked():
                        lastVersion += 1
                    versionString = self.buildVersionString(lastVersion)
                    objName += '_' + versionString

                objFileName = str(objName + '.mot')
                file = os.path.join(path, objFileName)
                with open(file, 'w') as f:
                    f.write( 'LWMO\n3\n\nNumChannels 9\n' )
                    data = []
                    channels = ['tx','ty','tz','ry','rx','rz','sx','sy','sz']
                    chanData = []

                    for i in range(9):
                        f.write( 'Channel ' + str(i) + '\n{ Envelope\n' )
                        count = len(thisObjData[i])
                        f.write(  '  ' + str(count) + '\n' )
                
                        for time in range(int(min), int(max)+1):
                            index = int(time - min)
                            val = thisObjData[i][index]
                            if val != None:
                                if i in range(2,5):
                                     val *= -1


                                f.write(  '  Key ' + str(val) + ' ' + str(time/fps) + '0 0 0 0 0 0 0\n' )


                        f.write( '  Behaviors 1 1\n}\n' )
                progress = (o+1)/float(len(objData))
                self.progressBar.setValue(progress*100)
        else:
            print 'Path doesn\'t exist'


def getLatestVersion(filename):
    version = getVersion(filename)
    if version:
        v = 1
        for motFile in os.listdir(os.path.dirname(filename)):
            if motFile.split('.')[-1] == 'mot':
                print motFile
                


# build the node itself
class jb_motReader(omMPx.MPxNode):
    kPluginNodeName = "jb_motReader"
    kPluginNodeId = om.MTypeId(0x28D05)
    objectData = ''
    fps = 0

    time = om.MObject()
    posScale = om.MObject()
    file = om.MObject()
    trans = om.MObject()
    rot = om.MObject()
    scale = om.MObject()
    
    
    def __init__(self):
        jb_motReader.objectData = { 'fps':0, 'keys':[], 'file':'' }
        omMPx.MPxNode.__init__(self)
    
    def hermite(self, t, p0=1, p1=0, n0=0, n1=0):
        val = (2*(t*t*t)-3*(t*t)+1)*p0
        val += ((t*t*t)-2*(t*t)+t)*n0
        val += (-2*(t*t*t)+3*(t*t))*p1
        val += ((t*t*t)-(t*t))*n1
        return val


    def compute(self, plug, data):
        import maya.cmds as mc
        time = data.inputValue(self.time).asTime().value()
        timeOffset = data.inputValue(self.timeOffset).asTime().value()
        posScale = data.inputValue(self.posScale).asFloat()
        sclScale = data.inputValue(self.sclScale).asFloat()
        translate = data.outputValue(self.translate)
        rotate = data.outputValue(self.rotate)
        rx = data.outputValue(self.rx)
        ry = data.outputValue(self.ry)
        rz = data.outputValue(self.rz)
        scale = data.outputValue(self.scale)
        interp = data.outputValue(self.interp).asInt()
        rotEval = data.outputValue(self.rotEval).asInt()
        update = data.inputValue(self.update).asBool()
        
        
        file = data.inputValue(self.file).asString()
        # these lines checks if the user wants quaternion interpolations
        # and since that is not implemented, this code resets it to euler and gives the user a message
        if rotEval == 1:
            import maya.mc as mc
            print (self.name() + ' warning: Quaternion Interpolation is not implemented yet.. whine a little if you want to. Attribute set to Euler' )
            mc.setAttr( (self.name()+'.re'), 0 )

        # if the user pressed update, acknowledge that, and uncheck the button again
        if update:
            print ( self.name() + ' updated the .mot file' )
            mc.setAttr( (self.name()+'.ud'), False )
        
        
        matrix = om.MFloatMatrix()
        if self.objectData['file'] != file or update:
            if file == '':
                self.objectData = { 'fps':0, 'keys':[], 'file':'' }
            else:
                if os.path.isfile(file):

                    print getLatestVersion(file)

                    self.objectData = fetchData(file)
                else:
                    mc.error( 'Filepath specified in motionNode does not exist', sl=False )

        if file != '':
        
            # get the current time in the scene
            time += timeOffset
            val = [0,0,0,0,0,0,1,1,1]
            odata = self.objectData

            
            t = time - math.floor(time)
            odata = self.objectData

            for i in range(9):
                index = (time-(odata['keys'][i][0])*odata['fps'])
                index = int(math.floor(index))

                val[i] = 0
                length = len(odata['keys'][i][1])-1
                
                # if requested frame is before first data
                if index < 0:
                    val[i] = odata['keys'][i][1][0]
                    
                # if it's after last
                elif index > length:
                    val[i] = odata['keys'][i][1][-1]
                    
                # otherwise its in the range that we have data for
                else:

                    # if the requested time is an exact frame that we have data for in the list
                    if t==0:
                        val[i] = odata['keys'][i][1][int(index)]

                    # otherwise we do interpolations
                    else:

                        # if interpolationtype is set to linear
                        if interp == 0:

                            a = odata['keys'][i][1][int(index)]
                            try:
                                b = odata['keys'][i][1][int(index)+1]
                            except:
                                b = a
                            val[i] = (a*((t*-1)+1)) + (b*t)

                
                        # else if the interpolationtype is set to hermite
                        elif interp == 1:
                            # get the hermite points from that time to evaluate the interpolation
                            p1i = index
                            p2i = min(index+1, length)
                            n1i = max(index-1, 0)
                            n2i = min(index+2, length)

                            p1 = odata['keys'][i][1][p1i]
                            p2 = odata['keys'][i][1][p2i]
                            n1 = odata['keys'][i][1][n1i]
                            n2 = odata['keys'][i][1][n2i]

                            n1 = ((((n1-p1)*-1)+(p2-p1))/2)
                            n2 = (((n2-p2)+((p1-p2)*-1))/2)

                        
                            val[i] = self.hermite(t, p1, p2, n1, n2)
                
                
            pos = [val[0],val[1],val[2]]
            #rot = [val[4]*-1,val[3]*-1,val[5]]
            rot = [val[4]*-1,val[3]*-1,val[5]]
            scl = [val[6],val[7],val[8]]
            
            pos = [i*posScale for i in pos]
            scl = [i*sclScale for i in scl]
            
            
            # set the positional attributes
            translate.set3Float(pos[0], pos[1], pos[2])

            # set the rotational attributes (individual since they are angles and must be set with setMAngle and cannot be set with set3Float)
            rx.setMAngle(om.MAngle(rot[1]))
            ry.setMAngle(om.MAngle(rot[0]))
            rz.setMAngle(om.MAngle(rot[2]))
                
                
            # set the scale attributes
            scale.set3Float(scl[0], scl[1], scl[2])

        # clean the plug
        data.setClean(plug)

    @classmethod
    def nodeCreator(self):
        pointer = omMPx.asMPxPtr( jb_motReader() )
        return pointer

    @classmethod
    def nodeInitializer(self):
        uAttr = om.MFnUnitAttribute()
        nAttr = om.MFnNumericAttribute()
        tAttr = om.MFnTypedAttribute()
        mAttr = om.MFnMatrixAttribute()
        cAttr = om.MFnCompoundAttribute()
        eAttr = om.MFnEnumAttribute()
    
    
        # build the time attribute
        jb_motReader.time = uAttr.create("time", "tm", om.MFnUnitAttribute.kTime, 0.0)
        uAttr.setHidden(True)
        jb_motReader.addAttribute(jb_motReader.time)

        # build the time offset attribute
        jb_motReader.timeOffset = uAttr.create("timeOffset", "to", om.MFnUnitAttribute.kTime, 0.0)
        uAttr.setKeyable(True)
        jb_motReader.addAttribute(jb_motReader.timeOffset)

    
        # build the positional scale attribute
        jb_motReader.posScale = nAttr.create('posScale', 'ps', om.MFnNumericData.kFloat, 1.0 )
        nAttr.setKeyable(True)
        jb_motReader.addAttribute(jb_motReader.posScale)
    
        # build the scale scale attribute
        jb_motReader.sclScale = nAttr.create('sclScale', 'ss', om.MFnNumericData.kFloat, 1.0 )
        nAttr.setKeyable(True)
        jb_motReader.addAttribute(jb_motReader.sclScale)
    
        # build the interpolation attribute
        jb_motReader.interp = eAttr.create( 'interpolation', 'i', 1 )
        eAttr.addField('Linear', 0)
        eAttr.addField('Hermite', 1)
        eAttr.setKeyable(False)
        jb_motReader.addAttribute(jb_motReader.interp)

    
        # build the rotational evaluation attribute
        jb_motReader.rotEval = eAttr.create( 'rotationEvaluation', 're' )
        eAttr.addField('Euler Interpolation', 0)
        eAttr.addField('Quaternion Interpolation', 1)
        eAttr.setKeyable(False)
        jb_motReader.addAttribute(jb_motReader.rotEval)

        # build the update button
        jb_motReader.update = nAttr.create( 'update', 'ud', om.MFnNumericData.kBoolean, False )
        jb_motReader.addAttribute(jb_motReader.update)


        # build the file attribute
        jb_motReader.file = tAttr.create('file', 'f', om.MFnData.kString )
        tAttr.setStorable(True)
        tAttr.setKeyable(False)
        jb_motReader.addAttribute(jb_motReader.file)

    
        #
        # build the output attributes
        #
    
        # build the translation attribute
        jb_motReader.tx = nAttr.create( 'translateX', 'tx', om.MFnNumericData.kFloat )
        jb_motReader.ty = nAttr.create( 'translateY', 'ty', om.MFnNumericData.kFloat )
        jb_motReader.tz = nAttr.create( 'translateZ', 'tz', om.MFnNumericData.kFloat )
        jb_motReader.translate = cAttr.create( 'translate', 't' )
        cAttr.addChild( jb_motReader.tx )
        cAttr.addChild( jb_motReader.ty )
        cAttr.addChild( jb_motReader.tz )
        cAttr.setHidden(True)
        jb_motReader.addAttribute(jb_motReader.translate)

        # and the affects
        jb_motReader.attributeAffects(jb_motReader.time, jb_motReader.translate)
        jb_motReader.attributeAffects(jb_motReader.timeOffset, jb_motReader.translate)
        jb_motReader.attributeAffects(jb_motReader.posScale, jb_motReader.translate)
        jb_motReader.attributeAffects(jb_motReader.sclScale, jb_motReader.translate)
        jb_motReader.attributeAffects(jb_motReader.rotEval, jb_motReader.translate)
        jb_motReader.attributeAffects(jb_motReader.file, jb_motReader.translate)
        jb_motReader.attributeAffects(jb_motReader.update, jb_motReader.translate)

    
    
    
    
        # build the rotation attribute
        jb_motReader.rx = uAttr.create( 'rotateX', 'rx', om.MFnUnitAttribute.kAngle )
        jb_motReader.ry = uAttr.create( 'rotateY', 'ry', om.MFnUnitAttribute.kAngle )
        jb_motReader.rz = uAttr.create( 'rotateZ', 'rz', om.MFnUnitAttribute.kAngle )
        jb_motReader.rotate = cAttr.create( 'rotate', 'r' )
        cAttr.addChild( jb_motReader.rx )
        cAttr.addChild( jb_motReader.ry )
        cAttr.addChild( jb_motReader.rz )
        cAttr.setHidden(True)
        jb_motReader.addAttribute(jb_motReader.rotate)

        # and the affects
        jb_motReader.attributeAffects(jb_motReader.time, jb_motReader.rotate)
        jb_motReader.attributeAffects(jb_motReader.timeOffset, jb_motReader.rotate)
        jb_motReader.attributeAffects(jb_motReader.posScale, jb_motReader.rotate)
        jb_motReader.attributeAffects(jb_motReader.sclScale, jb_motReader.rotate)
        jb_motReader.attributeAffects(jb_motReader.rotEval, jb_motReader.rotate)
        jb_motReader.attributeAffects(jb_motReader.file, jb_motReader.rotate)

    
    
        # build the scale attribute
        jb_motReader.sx = nAttr.create( 'scaleX', 'sx', om.MFnNumericData.kFloat, 1 )
        jb_motReader.sy = nAttr.create( 'scaleY', 'sy', om.MFnNumericData.kFloat, 1 )
        jb_motReader.sz = nAttr.create( 'scaleZ', 'sz', om.MFnNumericData.kFloat, 1 )
        jb_motReader.scale = cAttr.create( 'scale', 's' )
        cAttr.addChild( jb_motReader.sx )
        cAttr.addChild( jb_motReader.sy )
        cAttr.addChild( jb_motReader.sz )
        cAttr.setHidden(True)
        jb_motReader.addAttribute(jb_motReader.scale)

        # and the affects
        jb_motReader.attributeAffects(jb_motReader.time, jb_motReader.scale)
        jb_motReader.attributeAffects(jb_motReader.timeOffset, jb_motReader.scale)
        jb_motReader.attributeAffects(jb_motReader.posScale, jb_motReader.scale)
        jb_motReader.attributeAffects(jb_motReader.sclScale, jb_motReader.scale)
        jb_motReader.attributeAffects(jb_motReader.rotEval, jb_motReader.scale)
        jb_motReader.attributeAffects(jb_motReader.file, jb_motReader.scale)


def initializePlugin(mobject):
    import sys
    omMPx.MFnPlugin(mobject, 'Jonas Borgman', '1.0', 'Any')
    mplugin = omMPx.MFnPlugin(mobject)
    try:
        mplugin.registerNode( jb_motReader.kPluginNodeName, jb_motReader.kPluginNodeId, jb_motReader.nodeCreator, jb_motReader.nodeInitializer )
        mplugin.registerCommand( jb_applyMotReader.kPluginCmdName, jb_applyMotReader.cmdCreator )
        mplugin.registerCommand( jb_exportMotFile.kPluginCmdName, jb_exportMotFile.cmdCreator )
        buildMenu()
    except:
        sys.stderr.write( "Failed to register jb_motionData" )
        raise


def uninitializePlugin(mobject):
    import sys
    import maya.cmds as cmds
    
    mplugin = omMPx.MFnPlugin(mobject)
    try:
        mplugin.deregisterCommand(jb_applyMotReader.kPluginCmdName)
        mplugin.deregisterCommand(jb_exportMotFile.kPluginCmdName)
        mplugin.deregisterNode( jb_motReader.kPluginNodeId )
        cmds.deleteUI( 'jb_motionDataMenu' )
        mayaMain.removeToolsMenu('Plugins')

    except:
        sys.stderr.write( "Failed to unregister jb_motionData" )
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
        cmds.menuItem( 'jb_motionDataMenu', label='Motion Data', subMenu=True, parent=pluginMenu )
        cmds.menuItem( 'jb_motionDataApply', label='Apply motion node', parent='jb_motionDataMenu', c=cmds.jb_applyMotReader )
        cmds.menuItem( 'jb_motionDataBake', label='Bake motion data', parent='jb_motionDataMenu', c=cmds.jb_exportMotionData )

        cmds.setParent( 'MayaWindow' )


def fetchData(filePath):
    import maya.cmds as cmds
    data = []
    thisObj = { 'fps':0, 'keys':[], 'file':filePath }
    with open(filePath) as f:
        for line in f:
            data.append( line.strip() )

    if data[0] == "LWMO":
        if data[1] == "3":
            channelCount = int(data[3][12:])


            allTimeStart = 0
            allTimeEnd = 0
                    
            allKeys = []
            for i in range( channelCount ):

                channelKeys = []
                startTimePos = data.index( 'Channel ' + str(i) ) + 3
                startTime = float(data[startTimePos].split( ' ' )[2])
                p = startTimePos
                channelData = [startTime,[]]
                
                if thisObj['fps'] == 0:
                    if data[p+1][:3] == 'Key':
                        now = float(data[p].split( ' ' )[2])
                        next = float(data[p+1].split( ' ' )[2])
                        thisObj['fps'] = int(round(1 / (next - now)))
                        
                while data[p][:3] == 'Key':
                    thisVal = float(data[p].split( ' ' )[1])
                    if i == 2:
                        thisVal *= -1
                    channelKeys.append( thisVal )
                            
                    p += 1
                channelData[1] = channelKeys
                allKeys.append( channelData )

            thisObj['keys'] = allKeys
    return thisObj

