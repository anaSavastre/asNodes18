#include "asSurfaceSliding.h"

#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>

MStatus initializePlugin(MObject obj) {
	MFnPlugin plugin(obj, "Ana", "1.0", "2018");
	MStatus status = plugin.registerNode("asSurfaceSliding", asSurfaceSliding::as_id, asSurfaceSliding::nodeCreator, asSurfaceSliding::nodeInitializer, MPxNode::kDeformerNode);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return status;
}

MStatus uninitializePlugin(MObject obj) {
	MFnPlugin plugin(obj);
	MStatus status = plugin.deregisterNode(asSurfaceSliding::as_id);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	return status;
}
