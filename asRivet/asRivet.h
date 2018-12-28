#pragma once
#ifndef ASRIVET_H
#define ASRIVET_H

#include <maya/MObject.h>
#include <maya/MPxNode.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnNumericData.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnNurbsSurface.h>
#include <maya/MFnData.h>
#include <maya/MPoint.h>
#include <maya/MEulerRotation.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MAngle.h>
#include <maya/MMatrix.h>
#include <maya/MTypes.h>
#include <maya/MStatus.h>
#include <maya/MTypeId.h>
#include <math.h>

class asRivet : public MPxNode {
public:
	asRivet() {};
	virtual MStatus compute(const MPlug&, MDataBlock&);
	static void* nodeCreator();
	static MStatus nodeInitializer();
	static MTypeId id;
private:
	static MObject m_inSurfaceWS;
	static MObject m_inU;
	static MObject m_inV;

	static MObject m_tangent;
	static MObject m_forwardX, m_forwardY, m_forwardZ;
	static MObject m_upX, m_upY, m_upZ;
	static MObject m_forward;
	static MObject m_up;
	static MObject m_rotateOrder;
	static MObject m_percentage;

	// Parent Inverse Matrix
	static MObject m_parentInverseMatrix;
	static MObject m_tx, m_ty, m_tz, m_rx, m_ry, m_rz;
	static MObject m_outTranslation;
	static MObject m_outRotation;
	MEulerRotation::RotationOrder m_rotationOrderVect[6] = { MEulerRotation::RotationOrder::kXYZ,
		MEulerRotation::RotationOrder::kYZX ,
		MEulerRotation::RotationOrder::kZXY ,
		MEulerRotation::RotationOrder::kXZY ,
		MEulerRotation::RotationOrder::kYXZ ,
		MEulerRotation::RotationOrder::kZYX };
	MMatrix createMatrixFromTangent(MVector normal, MVector tangentU);
};
#endif