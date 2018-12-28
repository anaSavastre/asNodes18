#pragma once
#ifndef ASTRIG_H
#define ASTRIG_H

#include <maya/MObject.h>
#include <maya/MPxNode.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnNumericData.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnData.h>
#include <maya/MAngle.h>
#include <maya/MTypeId.h>
#include <math.h>

class asTrig : public MPxNode {
public:
	asTrig() {};
	virtual MStatus compute(const MPlug&, MDataBlock&);
	static void* nodeCreator();
	static MStatus nodeInitializer();

	static MTypeId id;
private:
	static MObject inputAngleAttr;
	static MObject outputAttr;
	static MObject inFunctionAttr;
};
#endif