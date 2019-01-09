#include "asSlidingSkinning.h"

#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>

MStatus initializePlugin(MObject obj) {
	MFnPlugin plugin(obj, "Ana", "1.0", "2018");
	MStatus status = plugin.registerNode("asSlidingSkinning", asSlidingSkinning::as_id, asSlidingSkinning::nodeCreator, asSlidingSkinning::nodeInitializer, MPxNode::kDeformerNode);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	if (!status) {
		status.perror("registerNode");
		return status;
	}


	return status;
}

MStatus uninitializePlugin(MObject obj) {
	MFnPlugin plugin(obj);
	MStatus status = plugin.deregisterNode(asSlidingSkinning::as_id);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	return status;
}
