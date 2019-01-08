#include "asMatloft.h"
//#include <cstdlib>

MTypeId asMatloft::id(0x09742);

MObject asMatloft::as_inWorldMatrix;
MObject asMatloft::as_inRevolveX;
MObject asMatloft::as_inRevolveY;
MObject asMatloft::as_inRevolveZ;
MObject asMatloft::as_inRevolveNegativeX;
MObject asMatloft::as_inRevolveNegativeY;
MObject asMatloft::as_inRevolveNegativeZ;
MObject asMatloft::as_spams; 

MObject asMatloft::as_outSurface;

void* asMatloft::nodeCreator() { return new asMatloft; }

MStatus asMatloft::compute(const MPlug& pPlug, MDataBlock& pDataBlock)
{
	MStatus stat;
	if (pPlug == as_outSurface)
	{
		MArrayDataHandle inMatrixArray = pDataBlock.inputArrayValue(as_inWorldMatrix, &stat);
		// int Spams
		int spams = pDataBlock.inputValue(as_spams).asInt();
		//bool inPercentage = pDataBlock.inputValue(m_percentage).asBool();
		bool revolveX = pDataBlock.inputValue(as_inRevolveX).asBool();
		bool revolveY = pDataBlock.inputValue(as_inRevolveY).asBool();
		bool revolveZ = pDataBlock.inputValue(as_inRevolveZ).asBool();
		bool revolveNegativeX = pDataBlock.inputValue(as_inRevolveNegativeX).asBool();
		bool revolveNegativeY = pDataBlock.inputValue(as_inRevolveNegativeY).asBool();
		bool revolveNegativeZ = pDataBlock.inputValue(as_inRevolveNegativeZ).asBool();
		MFnNurbsSurfaceData surfaceData;
		MObject newSurfaceData = surfaceData.create(&stat);
		matloft(inMatrixArray, newSurfaceData, stat, revolveX, revolveY, revolveZ, revolveNegativeX, revolveNegativeY, revolveNegativeZ);
		
		if (!stat)
			return stat;

		//Output
		MDataHandle outSurfaceHandle = pDataBlock.outputValue(as_outSurface);
		outSurfaceHandle.setMObject(newSurfaceData);
		outSurfaceHandle.setClean();
	}
	return MStatus::kSuccess;
}
MStatus asMatloft::nodeInitializer()
{
	MFnNurbsSurface nurbsSurfaceFn;
	MFnTypedAttribute typedAttributeFn;
	MFnMatrixAttribute matrixAttributeFN;
	MFnNumericAttribute numericAttributeFN;

	// Attribute: inWorldMatrix	
	as_inWorldMatrix = matrixAttributeFN.create("inputMatrix", "inM",MFnMatrixAttribute::kDouble, 0);
	matrixAttributeFN.setArray(true);
	addAttribute(as_inWorldMatrix);

	// Attribute: spams
	as_spams = numericAttributeFN.create("spams", "spams", MFnNumericData::kInt);
	addAttribute(as_spams);
	// Attribute: inRevolveX
	as_inRevolveX = numericAttributeFN.create("revolveX", "rX", MFnNumericData::kBoolean);
	addAttribute(as_inRevolveX);
	// Attribute: inRevolveNegativeX
	as_inRevolveNegativeX = numericAttributeFN.create("revolveNegativeX", "rnX", MFnNumericData::kBoolean);
	addAttribute(as_inRevolveNegativeX);
	// Attribute: inRevolveY
	as_inRevolveY = numericAttributeFN.create("revolveY", "rY", MFnNumericData::kBoolean);
	addAttribute(as_inRevolveY);
	// Attribute: inRevolveY
	as_inRevolveNegativeY = numericAttributeFN.create("revolveNegativeY", "rnY", MFnNumericData::kBoolean);
	addAttribute(as_inRevolveNegativeY);
	// Attribute: inRevolveZ
	as_inRevolveZ = numericAttributeFN.create("revolveZ", "rZ", MFnNumericData::kBoolean, 1);
	addAttribute(as_inRevolveZ); 
	// Attribute: inRevolveZ
	as_inRevolveNegativeZ = numericAttributeFN.create("revolveNegativeZ", "rnZ", MFnNumericData::kBoolean);
	addAttribute(as_inRevolveNegativeZ);

	// Attribute: outSurface
	as_outSurface = typedAttributeFn.create("outputSurface", "outS", MFnData::kNurbsSurface, 0);
	addAttribute(as_outSurface);

	// Attribute Affects
	attributeAffects(as_inWorldMatrix, as_outSurface);
	attributeAffects(as_spams, as_outSurface);
	attributeAffects(as_inRevolveX, as_outSurface);
	attributeAffects(as_inRevolveY, as_outSurface);
	attributeAffects(as_inRevolveZ, as_outSurface);
	attributeAffects(as_inRevolveNegativeX, as_outSurface);
	attributeAffects(as_inRevolveNegativeY, as_outSurface);
	attributeAffects(as_inRevolveNegativeZ, as_outSurface);

	return MStatus::kSuccess;
}

MObject asMatloft::matloft(MArrayDataHandle inMatrixArray, MObject surfaceObj, MStatus stat, bool revX, bool revY, bool revZ, bool revNegativeX, bool revNegativeY, bool revNegativeZ)
{
	MFnNurbsSurface surfFn;
	MPointArray cvs, points;
	MDoubleArray knotsU, knotsV;
	MObject returnSurface;
	int i, numCVs;
	int numbElements = inMatrixArray.elementCount();
	if (numbElements < 1)
		return MObject::kNullObj;
	points = getPointsFromMatrice(inMatrixArray, stat);
	int spansU = numbElements-2;
	int knots = spansU + 2 * 3 - 1;
	//cerr << "spams: " << spansU << "\n";
	// Get CV locations

	// CREATING KNOT ARRAY
	// U dimension: numb of element triple knoted 
	for (i = 0; i < 3; i++)
		knotsU.append(0.0);
	for (i = 1; i < spansU; i++)
		knotsU.append((double)i);
	for (i = 0; i < 3; i++)
		knotsU.append((double)spansU );
	// V dimension 
	for (i = 0; i < 3; i++)
		knotsV.append(0.0);

	for (i = 0; i < 3; i++)
		knotsV.append(1.0);

	// CREATING CVs ARRAY
	
	for (i = 0; i <  spansU+3; i++)
	{
		
		for (float j =- 0.5; j < 0.5; j+=0.33)
		{
			MPoint point = points[i];
			//cerr << "point: " << point.x <<" "<< point.y << " " << point.z << "\n";
			if (revX)
			{
				point.x += (double)j;
			}
			if (revY)
			{
				point.y += (double)j;
			}
			if (revZ)
			{
				point.z += (double)j;
			}
			if (revNegativeX)
			{
				point.x -= (double)j;
			}
			if (revNegativeY)
			{
				point.y -= (double)j;
			}
			if (revNegativeZ)
			{
				point.z -= (double)j;
			}
			cvs.append(point);
		}
		
	}
	int len = cvs.length();

	double surfaceLength = cvs[len - 1].x - cvs[0].x;
	//cerr << "knotsU array length: " << knotsU.length() << "\n";
	//cerr << "knorsV array length: " << knotsV.length() << "\n";
	//cerr << "cvs array length: " << len << "\n";


	returnSurface = surfFn.create(cvs, knotsU, knotsV, 3, 3,
		MFnNurbsSurface::kOpen, MFnNurbsSurface::kOpen,
		false, surfaceObj, &stat);
	if (stat != MS::kSuccess)
	{
		cerr << "Error in creating surface: " << stat << "\n";
		return MObject::kNullObj;
	}
	return returnSurface;

}

MPointArray asMatloft::getPointsFromMatrice(MArrayDataHandle inMatrixArray, MStatus stat)
{
	int i;
	int numbElem = inMatrixArray.elementCount();
	MPointArray points;



	for (i = 0; i <= numbElem; i++)
	{
		inMatrixArray.jumpToElement(i);
		MDataHandle elementHandle = inMatrixArray.inputValue(&stat);
		MMatrix valueMatrix(elementHandle.asMatrix());
		// Creating point
		MPoint point;
		point.x = valueMatrix[3][0];
		point.y = valueMatrix[3][1];
		point.z = valueMatrix[3][2];
		points.append(point);
		//std::cerr << "elementA: " << valueMatrix[3][0] << "\n";
	}
	return points;

}