#include "asMatloft.h"

#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>

MStatus initializePlugin(MObject obj) {
	MFnPlugin plugin(obj, "Ana", "1.0", "2018");
	MStatus status = plugin.registerNode("asMatloft", asMatloft::id, asMatloft::nodeCreator, asMatloft::nodeInitializer);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	
	return status;
}

MStatus uninitializePlugin(MObject obj) {
	MFnPlugin plugin(obj);
	MStatus status = plugin.deregisterNode(asMatloft::id);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	return status;
}
