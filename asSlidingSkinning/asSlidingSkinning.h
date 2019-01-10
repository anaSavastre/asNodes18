#pragma once
#ifndef ASSLIDINGSKINNING_H
#define ASSLIDINGSKINNING_H


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
#include <maya/MItMeshVertex.h>
#include <math.h>


// Attribute
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnMatrixAttribute.h>



class asSlidingSkinning : public MPxDeformerNode {
public:
	asSlidingSkinning() {};
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
	static MObject as_strength;
	static MObject as_iterations;
	static MObject as_steps;
	static MObject as_elasticity;
	static int as_restPoseEvaluated;
	MObject getInputMesh(MDataBlock&, unsigned int);
	MPoint averageVertex(int pointIndex, MItMeshVertex, MPointArray);
	double clamp(double, double, double);
	double smoothStep(double, double, double);
	MVector displacementVector(MVector startPose, MVector inTransformation, MPoint vertexPoz, MVector vertexNormal);


};
#endif