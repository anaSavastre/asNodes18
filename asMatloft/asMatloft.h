#pragma once
#ifndef ASMATLOFT_H
#define ASMATLOFT_H

#include <maya/MObject.h>
#include <maya/MPxNode.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnNurbsSurface.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnNumericData.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnData.h>
#include <maya/MPoint.h>
#include <maya/MPointArray.h>
#include <maya/MDoubleArray.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnNurbsSurfaceData.h>
#include <maya/MAngle.h>
#include <maya/MMatrix.h>
#include <maya/MTypes.h>
#include <maya/MStatus.h>
#include <maya/MTypeId.h>
#include <math.h>

class asMatloft : public MPxNode {
public:
	asMatloft() {};
	virtual MStatus compute(const MPlug&, MDataBlock&);
	static void* nodeCreator();
	static MStatus nodeInitializer();
	static MObject matloft(MArrayDataHandle, MObject, MStatus, MVector, float);
	static MPointArray getPointsFromMatrice(MArrayDataHandle, MStatus stat);
	//static MPointArray getFromMatrice(MArrayDataHandle, MStatus stat);
	static MTypeId id;
private:
	static MObject as_inWorldMatrix;
	static MObject as_outSurface;
	static MObject as_inWidthOffset;
	static MObject as_inRevolveVectorX;
	static MObject as_inRevolveVectorY;
	static MObject as_inRevolveVectorZ;
	static MObject as_inRevolveVector;

	static MObject as_spams;

	

};
#endif
