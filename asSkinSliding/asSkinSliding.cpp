#include "asSkinSliding.h"

#include <cstdlib>

MTypeId asSkinSliding::as_id(0x09765);
MObject asSkinSliding::as_stiffness;
MObject asSkinSliding::as_wi;
MObject asSkinSliding::as_mass;
int asSkinSliding::as_restPoseEvaluated;




void* asSkinSliding::nodeCreator() { return new asSkinSliding; }

MStatus asSkinSliding::deform(MDataBlock& pDataBlock, MItGeometry& pGeoIterator, const MMatrix& pMatrix, unsigned int intGeometryIndex)
{
	MStatus status;
	// GETTING DATA
	MDataHandle envData = pDataBlock.inputValue(envelope, &status);
	if (status != MS::kSuccess) return status;

	float inputEnvelope = envData.asFloat();
	float stiffnessVal = pDataBlock.inputValue(as_stiffness, &status).asFloat();
	float wiParameterVal = pDataBlock.inputValue(as_wi, &status).asFloat();
	float massVal = pDataBlock.inputValue(as_mass, &status).asFloat();


	// Undeformed Mesh
	MArrayDataHandle inputMeshHandle = pDataBlock.inputArrayValue(input, &status);
	inputMeshHandle.jumpToElement(0);
	MDataHandle inputMeshElementHandle = inputMeshHandle.inputValue(&status);
	MDataHandle inputMeshGeomHandle = inputMeshElementHandle.child(inputGeom);
	MObject inputMeshObj = inputMeshGeomHandle.data();

	MFnMesh fnInputMesh(inputMeshObj, &status);
	unsigned int numVertices = fnInputMesh.numVertices();
	MPointArray origMeshVertexArray(numVertices);
	fnInputMesh.getPoints(origMeshVertexArray);
	// Get Vertex Normals Attray
	MFloatVectorArray normalsArray;
	fnInputMesh.getNormals(normalsArray, MSpace::kObject);


	MPointArray deformedPointArray;
	// Getting time frame
	MAnimControl animControl;
	MTime time;
	time = animControl.currentTime();
	MTime prevTime;
	

	// Creating Vertex Matrix
	static BasicMatrix restPose(numVertices, 3);
	static BasicMatrix prevPosition(numVertices, 3);
	static BasicMatrix currentPosition(numVertices, 3);
	BasicMatrix predictedVtx(numVertices, 3);
	int numEdges = fnInputMesh.numEdges();
	BasicMatrix edgeMatrix(numEdges, numVertices);
	// Simulation Variable 
	BasicMatrix P(numEdges, 3);
	BasicMatrix Z(numEdges, 3);
	BasicMatrix U(numEdges, 3);
	BasicMatrix M(numVertices, numVertices);
	BasicMatrix W(numEdges, numEdges);
	BasicMatrix velocity(numVertices, 3);
	static int timeStep;


	if (as_restPoseEvaluated == 0)
	{

		for (unsigned int i = 0; i < numVertices; i++)
		{
			restPose(i, 0) = origMeshVertexArray[i].x;
			restPose(i, 1) = origMeshVertexArray[i].y;
			restPose(i, 2) = origMeshVertexArray[i].z;

		}
		// EDGE MATRIX ~ D matrix
		
		edgeMatrix.SetToZero();
		// Creating D matrix
		for (int i = 0; i < numEdges; i++)
		{
			// GET EDGE VERTEX
			int coef0, coef1;
			int2 vertexList;
			fnInputMesh.getEdgeVertices(i, vertexList);
			getCoeff(vertexList[0], vertexList[1], &coef0, &coef1);
			edgeMatrix(i, vertexList[0]) = coef0;
			edgeMatrix(i, vertexList[1]) = coef1;


		}

		
		// Initialising
		P.SetToZero();
		U.SetToZero();
		M.SetToIndentity();
		W.SetToIndentity();
		M = M * massVal;
		W = W* wiParameterVal;
		prevTime = animControl.currentTime();
		timeStep = 1;
		prevPosition = restPose;
		as_restPoseEvaluated = 1;
	}

	// Simulation
	// VTX current position

	for (unsigned int i = 0; i < numVertices; i++)
	{
		currentPosition(i, 0) = origMeshVertexArray[i].x;
		currentPosition(i, 1) = origMeshVertexArray[i].y;
		currentPosition(i, 2) = origMeshVertexArray[i].z;

	}

	// Calculating predicted vtx Position
	velocity = currentPosition - prevPosition;
	velocity = velocity * (time.asUnits(MTime::k25FPS) - prevTime.asUnits(MTime::k25FPS));
	predictedVtx = currentPosition + velocity * timeStep;

	// Simulation VTX Position
	BasicMatrix tempMatrix(numEdges, 3);
	tempMatrix = edgeMatrix * predictedVtx;
	tempMatrix = tempMatrix + U;
	
	//P
	P = tempMatrix;
	// MAKE PROJECTION
	double wiSq = wiParameterVal * wiParameterVal;
	// Z
	Z = P * stiffnessVal;
	Z = Z + (tempMatrix*wiSq);
	Z = Z * (1 / (stiffnessVal + wiSq));
	// U
	tempMatrix = edgeMatrix * predictedVtx;
	U = U + tempMatrix;
	U = U - Z;
	
	//New X
	double hSq = timeStep * timeStep;
	BasicMatrix D = edgeMatrix;
	BasicMatrix Dtranspose = edgeMatrix.Transpose();
	BasicMatrix Wtranspose = W.Transpose();
	BasicMatrix DtWt(numVertices, numEdges);
	BasicMatrix DtWtW(numVertices, numEdges);
	BasicMatrix DtWtWD(numVertices, numVertices);
	BasicMatrix MX(numVertices, 3);
	BasicMatrix secondTerm(numVertices, 3);
	DtWt = Dtranspose * Wtranspose;
	DtWtW = DtWt * W;
	DtWtWD = DtWtW * D;
	DtWtWD = DtWtWD * hSq;
	DtWtWD = M + DtWtWD;
	DtWtWD.Inverse();
	MX = M * predictedVtx;
	secondTerm = Z - U;
	secondTerm = DtWtW * secondTerm;
	secondTerm = secondTerm * hSq;
	secondTerm = MX + secondTerm;

	currentPosition = DtWtWD * secondTerm;

	//UPDATING SIMULATION
	prevTime = time;
	timeStep++;
	prevPosition = predictedVtx;

	


	for (; !pGeoIterator.isDone(); pGeoIterator.next())
	{
		// GETTING WEIGHTS
		float weight = weightValue(pDataBlock, intGeometryIndex, pGeoIterator.index());
		MPoint pointPosition = pGeoIterator.position();
		float weightVal = 0.1;
		//float stiffnessVal = 0.3;
		pointPosition.x += sin(pGeoIterator.index() + weightVal + time.asUnits(MTime::k25FPS)) * stiffnessVal * normalsArray[pGeoIterator.index()].x * weight * inputEnvelope;
		pointPosition.y += sin(pGeoIterator.index() + weightVal + time.asUnits(MTime::k25FPS)) * stiffnessVal * normalsArray[pGeoIterator.index()].y * weight * inputEnvelope;
		pointPosition.z += sin(pGeoIterator.index() + weightVal + time.asUnits(MTime::k25FPS)) * stiffnessVal * normalsArray[pGeoIterator.index()].z * weight * inputEnvelope;
		deformedPointArray.append(pointPosition);
	}

	
	pGeoIterator.setAllPositions(deformedPointArray);

	

	return MStatus::kSuccess;
}
MStatus asSkinSliding::nodeInitializer()
{
	/*
	Creating the Attributes
	Attaching the Attributes

	USER DEF ATTR:
			(temporary attributes)
			stiffness: -> Float 
			weight: -> Float 
			mass: -> Float

	*/

	// as_restPoseEvaluated
	as_restPoseEvaluated = 0;
	MFnNumericAttribute numericAttributeFn;


	// INPUTS
	// Stiffness
	as_stiffness = numericAttributeFn.create("stiffness", "st", MFnNumericData::kFloat, 0);
	numericAttributeFn.setMin(0.0);
	numericAttributeFn.setMax(1.0);
	numericAttributeFn.setKeyable(1);
	addAttribute(as_stiffness);
	// Weight
	as_wi = numericAttributeFn.create("wiParameter", "wiParameter", MFnNumericData::kFloat, 0);
	//numericAttributeFn.setMin(0.0);
	//numericAttributeFn.setMax(10.0);
	numericAttributeFn.setKeyable(1);
	addAttribute(as_wi);
	// Mass
	as_mass = numericAttributeFn.create("mass", "m", MFnNumericData::kFloat, 0);
	numericAttributeFn.setMin(0.0);
	numericAttributeFn.setMax(10.0);
	numericAttributeFn.setKeyable(1);
	addAttribute(as_mass);
	// OUTPUTS	

	// ATTRIBUTE AFFECTS
	attributeAffects(as_stiffness, outputGeom);
	attributeAffects(as_wi, outputGeom);
	attributeAffects(as_mass, outputGeom);
	return MStatus::kSuccess;
}

void asSkinSliding::getCoeff(int a, int b, int *aCoef, int *bCoef)
{
	if (a > b)
	{
		*aCoef = -1;
		*bCoef = 1;
	}
	else
	{
		*aCoef = 1;
		*bCoef = -1;
	}
}