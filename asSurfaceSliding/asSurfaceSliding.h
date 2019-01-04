#pragma once
#ifndef ASSURFACESLIDING_H
#define ASSURFACESLIDING_H


// Deformer Classes
#include <maya/MObject.h>
#include <maya/MStatus.h>
#include <maya/MTypeId.h>
#include <maya/MPxDeformerNode.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MItGeometry.h>

// Deform
#include <maya/MFnMesh.h>
#include <maya/MVector.h>
#include <maya/MMeshIntersector.h>
#include <maya/MGlobal.h>
#include <maya/MPointArray.h>
#include <maya/MPlane.h>
#include <maya/MFloatVectorArray.h>
#include <math.h>


// Attribute
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnMatrixAttribute.h>


class asSurfaceSliding : public MPxDeformerNode {
public:
	asSurfaceSliding() {};
	virtual MStatus deform(MDataBlock&, MItGeometry&, const MMatrix&, unsigned int);
	static void* nodeCreator();
	static MStatus nodeInitializer();
	static MTypeId as_id;

	/*

	// when the accessory is deleted, this node will clean itself up
	virtual MObject&			accessoryAttribute() const;

	// create accessory nodes when the node is created
	virtual MStatus				accessoryNodeSetup(MDagModifier& _cmd);
	*/

private:
	// ATTRIBUTES
	// Temporary attr
	static MObject as_vertexHandle;
	static MObject as_radius;
	static MObject as_displacement;
	static MObject as_transformation;
	static MObject as_tx;
	static MObject as_ty;
	static MObject as_tz;
	static MObject as_inWorldMatrix;
	static int as_restPoseEvaluated;
	double clamp(double, double, double);
	double smoothStep(double, double, double);


	// OUTPUT TRANSFORMATION
	static MObject as_outTranslation;
	static MObject as_outTx;
	static MObject as_outTy;
	static MObject as_outTz;

};
#endif
