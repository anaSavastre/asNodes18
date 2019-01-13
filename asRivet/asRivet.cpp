#include "asRivet.h"

#include <cstdlib>

MTypeId asRivet::id(0x09940);


MObject asRivet::m_inSurfaceWS;
MObject asRivet::m_inU;
MObject asRivet::m_inV;
MObject asRivet::m_tangent;

MObject asRivet::m_tx;
MObject asRivet::m_ty;
MObject asRivet::m_tz;
MObject asRivet::m_rx;
MObject asRivet::m_ry;
MObject asRivet::m_rz;

MObject asRivet::m_forwardX;
MObject asRivet::m_forwardY;
MObject asRivet::m_forwardZ;
MObject asRivet::m_upX;
MObject asRivet::m_upY;
MObject asRivet::m_upZ;
MObject asRivet::m_forward;
MObject asRivet::m_up;
MObject asRivet::m_rotateOrder;
MObject asRivet::m_percentage;

MObject asRivet::m_parentInverseMatrix;

MObject asRivet::m_outTranslation;
MObject asRivet::m_outRotation;




void* asRivet::nodeCreator() { return new asRivet; }

MStatus asRivet::compute(const MPlug& pPlug, MDataBlock& pDataBlock)
{
	if (pPlug == m_tx || pPlug == m_ty || pPlug == m_tz || pPlug == m_rx || pPlug == m_ry || pPlug == m_rz || pPlug == m_outTranslation || pPlug == m_outRotation)
	{
		// INPUTS
		MFnNurbsSurface surface(pDataBlock.inputValue(m_inSurfaceWS).asNurbsSurfaceTransformed());
		double uParameter = pDataBlock.inputValue(m_inU).asDouble();
		double vParameter = pDataBlock.inputValue(m_inV).asDouble();
		int inTangent = pDataBlock.inputValue(m_tangent).asShort();
		int inRotationOrder = pDataBlock.inputValue(m_rotateOrder).asShort();
		MVector inForward = pDataBlock.inputValue(m_forward).asVector();
		MVector inUp = pDataBlock.inputValue(m_up).asVector();
		MMatrix parentInverseMatrix = pDataBlock.inputValue(m_parentInverseMatrix).asMatrix();
		bool inPercentage = pDataBlock.inputValue(m_percentage).asBool();
			   		 	  
		// DEBUGGING PARENT INVERSE
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				cerr <<" " << parentInverseMatrix[i][j] << " ";
			}

			cerr << "\n";
		}
		cerr << "\n";
		cerr << "\n";
			

		// OUTPUT
		MEulerRotation eulerRotation;
		MDataHandle outTranslationHandle, outRotationHandle;

		// SURFACE PARAMETERS		
		double rangeEndU, rangeEndV, rangeStartU, rangeStartV;
		MVector nrbS_normal, nrbS_uTangent, nrbS_vTangent;
		MPoint nrbS_point;


		// GET RANGE IN U AND V
		rangeEndU = surface.knotInU(surface.numKnotsInU() - 1);
		rangeEndV = surface.knotInV(surface.numKnotsInV() - 1);
		rangeStartU = surface.knotInU(0);
		rangeStartV = surface.knotInV(0);

		// PARAMETERS IF PERCENTAGE
		if (inPercentage)
		{
			uParameter = rangeStartU + ((rangeEndU - rangeStartU)*uParameter);
			vParameter = rangeStartV + ((rangeEndV - rangeStartV)*vParameter);
		}

		if (uParameter <= rangeStartU)
			uParameter = rangeStartU;
		if (uParameter >= rangeEndU)
			uParameter = rangeEndU;
		if (vParameter <= rangeStartV)
			vParameter = rangeStartV;
		if (vParameter >= rangeEndV)
			vParameter = rangeEndV;


		// GET NORMAL
		nrbS_normal = surface.normal(uParameter, vParameter, MSpace::kWorld);

		// GET TANGENT U
		surface.getTangents(uParameter, vParameter, nrbS_uTangent, nrbS_vTangent, MSpace::kWorld);

		// GET POINT 
		surface.getPointAtParam(uParameter, vParameter, nrbS_point, MSpace::kWorld);

		// CREATING TRANSFORMATION MATRIX
		/// aimMatrix 
		MMatrix aimMatrix;
		if (inTangent)
		{
			aimMatrix = createMatrixFromTangent(nrbS_normal, nrbS_vTangent);
		}
		else
		{
			aimMatrix = createMatrixFromTangent(nrbS_normal, nrbS_uTangent);
		}
		/// alignMatix
		MMatrix alignMatrix;
		alignMatrix = createMatrixFromTangent(inForward, inUp);

		//MTransformationMatrix transformMatrix(parentInverseMatrix*alignMatrix*aimMatrix);
		MTransformationMatrix transformMatrix(alignMatrix*aimMatrix*parentInverseMatrix);

		/// Extracting Rotation
		//eulerRotation = aimTransformationMatrix.eulerRotation();
		eulerRotation = transformMatrix.eulerRotation().reorder(m_rotationOrderVect[inRotationOrder]);

		//	TEMPORARY
		// std::cerr << nrbS_normal.x << " " << nrbS_normal.y << " " << nrbS_normal.z << " \n";
		// std::cerr << nrbS_normal[3];

		//Translation Handle
		outTranslationHandle = pDataBlock.outputValue(m_outTranslation);
		//outTranslationHandle.setMVector(MVector(nrbS_point)*parentInverseMatrix);
		outTranslationHandle.setMVector(transformMatrix.getTranslation(MSpace::kWorld)+MVector(nrbS_point));
		outTranslationHandle.setClean();

		//Rotation Handle
		outRotationHandle = pDataBlock.outputValue(m_outRotation);
		outRotationHandle.set3Double(eulerRotation.x, eulerRotation.y, eulerRotation.z);
		outRotationHandle.setClean();

	}
	return MStatus::kSuccess;
}

MStatus asRivet::nodeInitializer()
{
	MFnUnitAttribute unitAttributeFn;
	MFnNumericAttribute numericAttributeFn;
	MFnEnumAttribute enumAttributeFn;
	MFnTypedAttribute typedAttributeFn;
	MFnCompoundAttribute compoundAttributeFn;
	MFnMatrixAttribute matrixAttributeFN;

	// INPUTS
	// TangentU TangentV
	m_tangent = enumAttributeFn.create("tangent", "tg");
	enumAttributeFn.addField("tangentU", 0);
	enumAttributeFn.addField("tangentV", 1);
	addAttribute(m_tangent);
	// Rotate Order
	m_rotateOrder = enumAttributeFn.create("rotateOrder", "roo");
	enumAttributeFn.addField("xyz", 0);
	enumAttributeFn.addField("yzx", 1);
	enumAttributeFn.addField("zxy", 2);
	enumAttributeFn.addField("xzy", 3);
	enumAttributeFn.addField("yxz", 4);
	enumAttributeFn.addField("zyx", 5);
	addAttribute(m_rotateOrder);
	// Percentage
	m_percentage = numericAttributeFn.create("percentage", "p", MFnNumericData::kBoolean);
	addAttribute(m_percentage);
	// Surface World Space Input
	m_inSurfaceWS = typedAttributeFn.create("inputSurface", "sws", MFnData::kNurbsSurface, 0);
	addAttribute(m_inSurfaceWS);
	// U and V Parameters  Input
	m_inU = numericAttributeFn.create("parameterU", "u", MFnNumericData::kDouble, 0.5);
	addAttribute(m_inU);
	m_inV = numericAttributeFn.create("parameterV", "v", MFnNumericData::kDouble, 0.5);
	addAttribute(m_inV);
	// Vector Forward
	m_forwardX = numericAttributeFn.create("forwardX", "fx", MFnNumericData::kDouble, 0);
	m_forwardY = numericAttributeFn.create("forwardY", "fy", MFnNumericData::kDouble, 0);
	m_forwardZ = numericAttributeFn.create("forwardZ", "fz", MFnNumericData::kDouble, 1);
	m_forward = numericAttributeFn.create("forward", "f", m_forwardX, m_forwardY, m_forwardZ);

	addAttribute(m_forward);
	// Vector up
	m_upX = numericAttributeFn.create("upX", "ux", MFnNumericData::kDouble, 0);
	m_upY = numericAttributeFn.create("upY", "uy", MFnNumericData::kDouble, 1);
	m_upZ = numericAttributeFn.create("upZ", "uz", MFnNumericData::kDouble, 0);
	m_up = numericAttributeFn.create("up", "up", m_upX, m_upY, m_upZ);
	addAttribute(m_up);
	// Parent Inverse Matrix
	m_parentInverseMatrix = matrixAttributeFN.create("parentInverseMatrix", "pim", MFnMatrixAttribute::kDouble);
	addAttribute(m_parentInverseMatrix);

	// OUTPUTS	
	m_tx = numericAttributeFn.create("outTranslateX", "tx", MFnNumericData::kDouble);
	m_ty = numericAttributeFn.create("outTranslateY", "ty", MFnNumericData::kDouble);
	m_tz = numericAttributeFn.create("ouTranslateZ", "tz", MFnNumericData::kDouble);
	m_rx = unitAttributeFn.create("outRotateX", "rx", MFnUnitAttribute::kAngle);
	m_ry = unitAttributeFn.create("outRotateY", "ry", MFnUnitAttribute::kAngle);
	m_rz = unitAttributeFn.create("outRotateZ", "rz", MFnUnitAttribute::kAngle);

	m_outTranslation = numericAttributeFn.create("outTranslation", "outTranslation", m_tx, m_ty, m_tz);
	addAttribute(m_outTranslation);

	m_outRotation = compoundAttributeFn.create("outRotation", "outRotation");
	compoundAttributeFn.addChild(m_rx);
	compoundAttributeFn.addChild(m_ry);
	compoundAttributeFn.addChild(m_rz);
	addAttribute(m_outRotation);

	// ATTRIBUTE AFFECTS
	attributeAffects(m_inSurfaceWS, m_outTranslation);
	attributeAffects(m_inSurfaceWS, m_outRotation);
	attributeAffects(m_inU, m_outTranslation);
	attributeAffects(m_inU, m_outRotation);
	attributeAffects(m_inV, m_outTranslation);
	attributeAffects(m_inV, m_outRotation);
	attributeAffects(m_tangent, m_outTranslation);
	attributeAffects(m_tangent, m_outRotation);
	attributeAffects(m_forward, m_outTranslation);
	attributeAffects(m_forward, m_outRotation);
	attributeAffects(m_up, m_outTranslation);
	attributeAffects(m_up, m_outRotation);
	attributeAffects(m_parentInverseMatrix, m_outTranslation);
	attributeAffects(m_parentInverseMatrix, m_outRotation);
	attributeAffects(m_rotateOrder, m_outTranslation);
	attributeAffects(m_rotateOrder, m_outRotation);
	attributeAffects(m_percentage, m_outTranslation);
	attributeAffects(m_percentage, m_outRotation);

	return MStatus::kSuccess;
}
MMatrix asRivet::createMatrixFromTangent(MVector normal, MVector tangent)
{
	/// Matrix 
	MMatrix matrix;
	MVector  yLocal, zLocal;

	/// Creating local space axis
	zLocal = normal ^ tangent;
	yLocal = normal ^ zLocal;

	/// Creating aim matrix
	matrix[0][0] = normal.x; matrix[0][1] = normal.y; matrix[0][2] = normal.z; matrix[0][3] = 0;
	matrix[1][0] = yLocal.x; matrix[1][1] = yLocal.y; matrix[1][2] = yLocal.z; matrix[1][3] = 0;
	matrix[2][0] = zLocal.x; matrix[2][1] = zLocal.y; matrix[2][2] = zLocal.z; matrix[2][3] = 0;

	return matrix;

}