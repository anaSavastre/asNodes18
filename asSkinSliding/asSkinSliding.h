#pragma once
#ifndef ASSKINSLIDING_H
#define ASSKINSLIDING_H


// Deformer Classes
#include <maya/MObject.h>
#include <maya/MStatus.h>
#include <maya/MTypeId.h>
#include <maya/MPxDeformerNode.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MItGeometry.h>


// Deform 
#include <maya/MDataHandle.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MPoint.h>
#include <maya/MTypes.h>
#include <maya/MPointArray.h>
#include <maya/MFloatVectorArray.h>
#include <maya/MStatus.h>
#include <maya/MFnMesh.h>
#include <math.h>
#include <maya/MGlobal.h>
#include <maya/MAnimControl.h>
#include <maya/MTime.h>
#include <maya/MIntArray.h>
#include "BasicMatrix.h"

// Other
/*
#include <string.h>
#include <iostream>*/

// Attribute Classes
#include <maya/MFnNumericAttribute.h>

// Variables
#include <maya/MFnNumericData.h>


class asSkinSliding : public MPxDeformerNode {
public:
	asSkinSliding() {};
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
	static MObject as_stiffness;
	static MObject as_wi;
	static MObject as_mass;
	static int as_restPoseEvaluated;
	static void getCoeff(int, int, int*, int*);
	



};
#endif
