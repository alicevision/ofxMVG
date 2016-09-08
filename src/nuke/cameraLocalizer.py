import nuke
import numbers

def createCamera(input):
    "This function create the camera of the Camera Localizer Plugin for the given input"
    strInput = str(input)
    localizerNode = nuke.thisNode()
    xPos = localizerNode.xpos() + 250
    yPos = localizerNode.ypos() - ((input + 1) * 75)
    camera = nuke.nodes.Camera (name="Camera" + str(input + 1), xpos=xPos, ypos=yPos)
    camera["rot_order"].setValue("ZYX")
    camera["translate"].setExpression(localizerNode.name() + ".outputTranslate_" + strInput)
    camera["rotate"].setExpression(localizerNode.name() + ".outputRotate_" + strInput)
    camera["scaling"].setExpression(localizerNode.name() + ".outputScale_" + strInput)
    camera["focal"].setExpression(localizerNode.name() + ".outputFocalLength_" + strInput)
    camera["haperture"].setExpression(localizerNode.name() + ".inputSensorWidth_" + strInput)
    camera["vaperture"].setExpression("haperture * " + localizerNode.name() + ".height / " + localizerNode.name() + ".width")
    camera["near"].setExpression(localizerNode.name() + ".outputNear_" + strInput)
    camera["far"].setExpression(localizerNode.name() + ".outputFar_" + strInput)
    camera["win_translate"].setExpression("2 * ((" + localizerNode.name() +".width * 0.5) - " +  localizerNode.name() + ".outputOpticalCenter_"+ strInput + ".x) / " +  localizerNode.name() + ".width", 0)
    camera["win_translate"].setExpression("2 * (" + localizerNode.name() +".outputOpticalCenter_" + strInput + ".y - ( " +  localizerNode.name() + ".height * 0.5)) / " +  localizerNode.name() + ".width", 1)
    
    return camera


def createScene():
	"This function create the complete 3D Scene for the Camera Localizer Plugin"
	localizerNode = nuke.thisNode()
	xPos = localizerNode.xpos()
	yPos = localizerNode.ypos()
	renderNode = nuke.nodes.ScanlineRender(name="ScanLineRender",xpos=xPos+500,ypos=yPos)
	sceneNode = nuke.nodes.Scene(name = 'Scene',xpos=xPos+510,ypos=yPos-150)
	readGeoNode = nuke.nodes.ReadGeo(name="3DReconstruction", xpos=xPos+500, ypos=yPos-300)
	readGeoNode['file'].setValue(localizerNode['reconstructionFile'].getValue())
	renderNode.setInput(1, sceneNode)
	renderNode.setInput(0, localizerNode)
	sceneNode.setInput(0, readGeoNode)
	defViewer = nuke.toNode("Viewer1")
	defViewer.setInput(0, renderNode)
	for input in range(localizerNode.inputs()):
		camera = createCamera(input)
		if(input == 0) :
			renderNode.setInput(2, camera)
	return
	
	
def importLensCalibration(input):
    "This function import selected LensCalibration output to the given input of the Camera Localizer Plugin"
    strInput = str(input)
    localizerNode = nuke.thisNode()
    calibrationNode = nuke.selectedNode()
    localizerNode["inputOpticalCenter_" + strInput].setValue(calibrationNode["outputPrincipalPointOffset"].getValue())
    localizerNode["inputOpticalCenter_" + strInput].setValue(calibrationNode["outputPrincipalPointOffset"].getValue())
    localizerNode["inputFocalLengthMode_" + strInput].setValue("Known")
    localizerNode["inputFocalLength_" + strInput].setValue(calibrationNode["outputFocalLenght"].getValue())
    localizerNode["inputDistortion_" + strInput].setValue("Known")
    localizerNode["inputDistortionMode_" + strInput].setValue("Radial3") #Only Mode support 
    localizerNode["inputDistortionCoef1_" + strInput].setValue(calibrationNode["outputRadialCoef1"].getValue())
    localizerNode["inputDistortionCoef2_" + strInput].setValue(calibrationNode["outputRadialCoef2"].getValue())
    localizerNode["inputDistortionCoef3_" + strInput].setValue(calibrationNode["outputRadialCoef3"].getValue())
    
    return
	
	
#Init PlugIn
def initOFXcameralocalizer():
	n = nuke.thisNode()
	sceneBtn = n['createScene']
	sceneBtn.setValue("python createScene()")
	sceneBtn.setEnabled(True)
	for input in range(n.maximumInputs()):
		strInput = str(input)
		cameraBtn = n["outputCreateCamera_" + strInput]
		cameraBtn.setValue("python createCamera(" + strInput + ")")
		cameraBtn.setEnabled(True)
		calibBtn = n["inputImportLensCalibration_" + strInput]
		calibBtn.setValue("python importLensCalibration(" + strInput + ")")
		calibBtn.setEnabled(True)
		
nuke.addOnCreate(initOFXcameralocalizer, (), nodeClass='OFXopenmvg.cameralocalizer_v1')
