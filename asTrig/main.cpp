#include "asTrig.h"

#include <maya/MFnPlugin.h>

MStatus initializePlugin(MObject obj) {
	MFnPlugin plugin(obj, "Ana", "1.0", "2017");
	MStatus status = plugin.registerNode("asTrig", asTrig::id, asTrig::nodeCreator, asTrig::nodeInitializer);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	return status;
}

MStatus uninitializePlugin(MObject obj) {
	MFnPlugin plugin(obj);
	MStatus status = plugin.deregisterNode(asTrig::id);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	return status;
}