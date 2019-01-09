#include "asSlidingSkinning.h"

#include <cstdlib>


MTypeId asSlidingSkinning::as_id(0x09455);
MObject asSlidingSkinning::as_iterations;
MObject asSlidingSkinning::as_steps;
MObject asSlidingSkinning::as_strength;
//MItMeshVertex asSlidingSkinning::as_vertexIterator;

int asSlidingSkinning::as_restPoseEvaluated;


void* asSlidingSkinning::nodeCreator() { return new asSlidingSkinning; }

MStatus asSlidingSkinning::deform(MDataBlock& pDataBlock, MItGeometry& pGeoIterator, const MMatrix& pMatrix, unsigned int intGeometryIndex)
{
	MStatus status;
	// GETTING DATA
	MDataHandle envData = pDataBlock.inputValue(envelope, &status);
	if (status != MS::kSuccess) return status;

	float inputEnvelope = envData.asFloat();
	int inSimulationIterations = pDataBlock.inputValue(as_iterations, &status).asInt();
	if (status != MS::kSuccess) return status;
	int inSteps = pDataBlock.inputValue(as_steps, &status).asInt();
	if (status != MS::kSuccess) return status;
	double inStrength = pDataBlock.inputValue(as_strength, &status).asDouble();
	if (status != MS::kSuccess) return status;

	// GETTING INPUT MESH
	MArrayDataHandle inputMeshHandle = pDataBlock.outputArrayValue(input, &status);
	inputMeshHandle.jumpToElement(0);
	MDataHandle inputMeshElementHandle = inputMeshHandle.outputValue(&status);
	MDataHandle inputMeshGeomHandle = inputMeshElementHandle.child(outputGeom);
	MObject inputMeshObj = inputMeshGeomHandle.data();
	MFnMesh fnInputMesh(inputMeshObj, &status);
	/*MObject inputMeshObj = getInputMesh(pDataBlock, intGeometryIndex);
	MFnMesh fnInputMesh(inputMeshObj, &status);*/
	if (status != MS::kSuccess) return status;

	// GETTING VERTEX MESH ITERATOR
	MItMeshVertex meshVertIt(inputMeshObj, &status);
	if (status != MS::kSuccess) return status;

	// GETTING VERTEX START POSITION
	MPointArray meshVertex, allPoints;
	fnInputMesh.getPoints(meshVertex, MSpace::kObject);
	int numVert=meshVertex.length();

	// GETTING POSITIONS FROM ITER
	pGeoIterator.allPositions(allPoints);
	meshVertex = allPoints.length();

	// COMPUTE DEFORMATION
	for (int i = 0; i < inSimulationIterations; i++)
	{
		for (int step = 0; step < inSteps; step++)
		{
			// INITIALIZE NEW VERTEX ARRAY
			MPointArray newMeshVertex;
			newMeshVertex.copy(allPoints);
			for (int index = 0; index < numVert; index++)
			{
				// CURRENT WEIGHT 
				////////////////////

				// CURRENT POINT
				MPoint pointPosition = allPoints[index];


				// COMPUTE AVERAGE if position difference
				////////////////////////////	AVERAGE FUNCTION	////////////////////////////
				MPoint averagePosition;

				// GET INDEX
				int prevVertex;
				//cerr << "vert: " << index;
				//prevVertex = new int[1];
				//cerr << "====================================> vtxIter.setIndex(pointIndex, prevVertex);\n";
				meshVertIt.setIndex(index, prevVertex);

				// GET CONNECTION VTX
				MIntArray connectingVerts;

				meshVertIt.getConnectedVertices(connectingVerts);
				//cerr << "====================================> vtxIter.getConnectedVertices(connectingVerts);\n";


				// GET ADD ALL POSITIONS
				MPoint sumPoint = MPoint(1.0, 1.0, 1.0, 1.0);
				for (int i = 0; i < connectingVerts.length(); i++)
				{

					averagePosition += allPoints[connectingVerts[i]];
					//cerr << "====================================> sumPoint += pointArray[connectingVerts[i]];\n";
					//cerr << "index: " << i << "\n";
				}

				// RETURN AVERAGE
				averagePosition = averagePosition/connectingVerts.length();



				////////////////////////////	AVERAGE FUNCTION	////////////////////////////
				//cerr << "====================================> averagePosition:"<<averagePosition.x<<" " << averagePosition.y<< " " << averagePosition.z  <<"\n";
				// COMPUTE NEW COMPONENT POSITION
				MPoint offset_pos;
				offset_pos = (averagePosition - pointPosition) * inputEnvelope * inStrength;
				newMeshVertex[index] = pointPosition + offset_pos / (inSteps - step);

			}
			// UPDATE POSITION
			allPoints.copy(newMeshVertex);

		}

	}
	pGeoIterator.setAllPositions(allPoints);

	//fnInputMesh.setPoints(deformedPointArray);

		
	return MStatus::kSuccess;

	

}

MStatus asSlidingSkinning::nodeInitializer()
{
	
	// INITIALIZATIONS
	as_restPoseEvaluated = 0;

	MFnNumericAttribute numericAttributeFn;


	// INPUTS
	// vtxHandle
	as_strength = numericAttributeFn.create("strength", "strength", MFnNumericData::kDouble, 0.5);
	numericAttributeFn.setKeyable(1);
	numericAttributeFn.setMin(0.0);
	//numericAttributeFn.setMax(1.0);
	addAttribute(as_strength);
	// Radius
	as_iterations = numericAttributeFn.create("simulationIterations", "itr", MFnNumericData::kInt, 1);
	//numericAttributeFn.setMin(0.0);
	numericAttributeFn.setKeyable(1);
	addAttribute(as_iterations);
	// Elasticuty
	as_steps = numericAttributeFn.create("steps", "stp", MFnNumericData::kInt, 3);
	numericAttributeFn.setMin(0.0);
	//numericAttributeFn.setMax(1.0);
	numericAttributeFn.setKeyable(1);
	addAttribute(as_steps);

	// ATTRIBUTE AFFECTS
	attributeAffects(as_strength, outputGeom);
	attributeAffects(as_iterations, outputGeom);
	attributeAffects(as_steps, outputGeom);
	
	return MStatus::kSuccess;
}
MObject asSlidingSkinning::getInputMesh(MDataBlock& pDataBlock, unsigned int intGeometryIndex)
{
	MStatus status;
	MArrayDataHandle inputMeshHandle = pDataBlock.outputArrayValue(input, &status);
	inputMeshHandle.jumpToElement(intGeometryIndex);
	MDataHandle inputMeshElementHandle = inputMeshHandle.outputValue(&status);
	MDataHandle inputMeshGeomHandle = inputMeshElementHandle.child(outputGeom);
	MObject inputMeshObj = inputMeshGeomHandle.data();

	return inputMeshObj;
	
}

MPoint asSlidingSkinning::averageVertex(int pointIndex, MItMeshVertex vtxIter, MPointArray pointArray)
{
	// GET INDEX
	int prevVertex;
	//cerr << "vert: " << pointIndex;
	//prevVertex = new int[1];
	//cerr << "====================================> vtxIter.setIndex(pointIndex, prevVertex);\n";
	vtxIter.setIndex(pointIndex, prevVertex);
	
	// GET CONNECTION VTX
	MIntArray connectingVerts;

	vtxIter.getConnectedVertices(connectingVerts);
	//cerr << "====================================> vtxIter.getConnectedVertices(connectingVerts);\n";


	// GET ADD ALL POSITIONS
	MPoint sumPoint = MPoint(1.0, 1.0, 1.0, 1.0);
	for (int i = 0; i < connectingVerts.length(); i++)
	{

		sumPoint += pointArray[connectingVerts[i]];
		//cerr << "====================================> sumPoint += pointArray[connectingVerts[i]];\n";
		//cerr << "index: " << i << "\n";
	}

	// RETURN AVERAGE
	return sumPoint / connectingVerts.length();


	
}