#include "asMatloft.h"
//#include <cstdlib>

MTypeId asMatloft::id(0x09742);

MObject asMatloft::as_inWorldMatrix;
MObject asMatloft::as_inRevolveVector;
MObject asMatloft::as_inRevolveVectorX;
MObject asMatloft::as_inRevolveVectorY;
MObject asMatloft::as_inRevolveVectorZ;
MObject asMatloft::as_inWidthOffset;

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
		MVector inRevolveVector = pDataBlock.inputValue(as_inRevolveVector).asVector();
		MFnNurbsSurfaceData surfaceData;
		MObject newSurfaceData = surfaceData.create(&stat);
		float widthOffset = pDataBlock.inputValue(as_inWidthOffset).asFloat();
		matloft(inMatrixArray, newSurfaceData, stat, inRevolveVector, widthOffset);
		
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
	MFnMatrixAttribute matrixAttributeFn;
	MFnNumericAttribute numericAttributeFn;

	// Attribute: inWorldMatrix	
	as_inWorldMatrix = matrixAttributeFn.create("inputMatrix", "inM",MFnMatrixAttribute::kDouble, 0);
	matrixAttributeFn.setArray(true);
	addAttribute(as_inWorldMatrix);

	// Attribute: spams
	as_spams = numericAttributeFn.create("spams", "spams", MFnNumericData::kInt);
	addAttribute(as_spams);
	// Attribute: widthOffset
	as_inWidthOffset = numericAttributeFn.create("widthOffset", "wo", MFnNumericData::kFloat, 1);
	numericAttributeFn.setMin(0.01);
	addAttribute(as_inWidthOffset);
	// Attribute: revolveVector
	as_inRevolveVectorX = numericAttributeFn.create("revolveVectorX", "revX", MFnNumericData::kDouble, 0);
	as_inRevolveVectorY = numericAttributeFn.create("revolveVectorY", "revY", MFnNumericData::kDouble, 0);
	as_inRevolveVectorZ = numericAttributeFn.create("revolveVectorZ", "revZ", MFnNumericData::kDouble, 1);
	as_inRevolveVector = numericAttributeFn.create("revolveVector", "rev", as_inRevolveVectorX, as_inRevolveVectorY, as_inRevolveVectorZ);
	addAttribute(as_inRevolveVector);
	
	// Attribute: outSurface
	as_outSurface = typedAttributeFn.create("outputSurface", "outS", MFnData::kNurbsSurface, 0);
	addAttribute(as_outSurface);

	// Attribute Affects
	attributeAffects(as_inWorldMatrix, as_outSurface);
	attributeAffects(as_spams, as_outSurface);
	attributeAffects(as_inRevolveVector, as_outSurface);
	attributeAffects(as_inWidthOffset, as_outSurface);
	
	return MStatus::kSuccess;
}

MObject asMatloft::matloft(MArrayDataHandle inMatrixArray, MObject surfaceObj, MStatus stat, MVector revolveVector, float widthOffset)
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
		
		for (float j =- 3*widthOffset/2; j <= 3 * widthOffset / 2; j+=widthOffset)
		{
			// EXTRACTING VALUE MATRIX
			inMatrixArray.jumpToElement(i);
			MMatrix valueMatrix(inMatrixArray.inputValue(&stat).asMatrix());
			MPoint point = points[i];
			MVector revolve = revolveVector * valueMatrix;
			point += (double)j * revolve;
			//cerr << "point: " << point.x <<" "<< point.y << " " << point.z << "\n";
			
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
		MMatrix valueMatrix(inMatrixArray.inputValue(&stat).asMatrix());
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