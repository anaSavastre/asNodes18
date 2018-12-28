#include "asRivet.h"

#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>

MStatus initializePlugin(MObject obj) {
	MFnPlugin plugin(obj, "Ana", "1.0", "2018");
	MStatus status = plugin.registerNode("asRivet", asRivet::id, asRivet::nodeCreator, asRivet::nodeInitializer);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	const char *aeTemplate = "\
global proc AEasRivetTemplate( string $nodeName )\
{\
   editorTemplate -beginScrollLayout;\
   editorTemplate -beginLayout \"asRivet Attributes\" -collapse 0;\
       editorTemplate -addControl \"tangent\";\
       editorTemplate -addControl \"rotateOrder\";\
       editorTemplate -addControl \"percentage\";\
       editorTemplate -addControl \"inputSurface\";\
       editorTemplate -addControl \"parameterU\";\
       editorTemplate -addControl \"parameterV\"; \
       editorTemplate -addControl \"forward\"; \
       editorTemplate -addControl \"up\"; \
   editorTemplate -endLayout;\
\
   AEdependNodeTemplate $nodeName;\
\
   editorTemplate -addExtraControls;\
\
   editorTemplate -suppress \"parentInverseMatrix\";\
   editorTemplate -suppress \"outTranslation\";\
   editorTemplate -suppress \"outRotation\";\
\
   editorTemplate -endScrollLayout;\
}";
	MGlobal::executeCommand(aeTemplate);
	MGlobal::executeCommand("refreshEditorTemplates; refreshAE;");
	return status;
}

MStatus uninitializePlugin(MObject obj) {
	MFnPlugin plugin(obj);
	MStatus status = plugin.deregisterNode(asRivet::id);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	return status;
}
