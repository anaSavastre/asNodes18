#include "asSlidingSkinning.h"

#include <cstdlib>


MTypeId asSlidingSkinning::as_id(0x09455);
MObject asSlidingSkinning::as_iterations;
MObject asSlidingSkinning::as_steps;
MObject asSlidingSkinning::as_strength;
MObject asSlidingSkinning::as_elasticity;
MObject asSlidingSkinning::as_vertexHandle;
MObject asSlidingSkinning::as_radius;
MObject asSlidingSkinning::as_displacement;

MObject asSlidingSkinning::as_transformation;
MObject asSlidingSkinning::as_tx;
MObject asSlidingSkinning::as_ty;
MObject asSlidingSkinning::as_tz;

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
	double inElasticity = pDataBlock.inputValue(as_elasticity, &status).asDouble();
	if (status != MS::kSuccess) return status;
	int vertexHandle = pDataBlock.inputValue(as_vertexHandle, &status).asInt();
	if (status != MS::kSuccess) return status;
	float radius = pDataBlock.inputValue(as_radius, &status).asFloat();
	if (status != MS::kSuccess) return status;
	float displacement = pDataBlock.inputValue(as_displacement, &status).asFloat();
	if (status != MS::kSuccess) return status;
	MVector inTransformation = pDataBlock.inputValue(as_transformation).asVector();
	if (status != MS::kSuccess) return status;
	
	// GETTING INPUT MESH
	MObject inputMeshObj = getInputMesh(pDataBlock, intGeometryIndex);
	MFnMesh fnInputMesh(inputMeshObj, &status);
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

	// EDGES ARRAY INITIALIZATION
	int numEdges = fnInputMesh.numEdges();
	static MDoubleArray restEdgeLength(numEdges, 0.0), currentEdgeLength(numEdges, 0.0), prevEdgeLength(numEdges, 0.0);

	// VERTEX ARRAY INITIALIZATION
	MVectorArray vertexForce(meshVertex.length());
	int numVertex = meshVertex.length();
	static MVectorArray restVertexArray(numVertex), prevVertexArray(numVertex);
	static MVectorArray previousVertexArray(numVertex);

	// INITIALIZATION
	// STAGE THAT IS COMPUTED JUST ON THE FIST EXECUTION OF THE DEFORMER
	MVector startPose, dTransform;
	if (as_restPoseEvaluated == 0)
	{
		as_restPoseEvaluated = 1;
		startPose = inTransformation;
		// GETTING REST EDGE LENGTH
		for (int i = 0; i < numEdges; i++)
		{
			// GET VERT ID FOR EDGE I
			int2 vertID;
			fnInputMesh.getEdgeVertices(i, vertID);
			restEdgeLength[i] = allPoints[vertID[1]].distanceTo(allPoints[vertID[0]]);
			

		}
		prevEdgeLength.copy(restEdgeLength);
		// GETTING REST VERTEX
		for (int i = 0; i < numVertex; i++)
		{
			restVertexArray[i] = meshVertex[i];

			previousVertexArray[i] = meshVertex[i];
		}
	}


	// GET POINT TANGENTS
	// Normals
	MFloatVectorArray uTangents, vTangents, vertexNormals;
	status = fnInputMesh.getVertexNormals(true, vertexNormals, MSpace::kObject);
	if (!status)
	{
		MGlobal::displayError("Failed to get normals");
		return MStatus::kFailure;

	}

	// U and V Tangents
	for (unsigned int i = 0; i < vertexNormals.length(); i++)
	{
		// Calculating u tangent
		MVector tg1 = vertexNormals[i] ^ MVector(0, 1, 0);
		MVector tg2 = vertexNormals[i] ^ MVector(0, 0, 1);
		// Checking if the normal is different than the arbitray chosen vects
		if (tg1 != MVector(0, 0, 0))
		{
			uTangents.append(tg1);
		}
		else
		{
			uTangents.append(tg2);
		}

		vTangents.append(vertexNormals[i] ^ uTangents[i]);
	}

	// GET TRANSFORM PROJECTION ON SURFACE
	// Calculate displacement for vtxHandle

	MVector displaceVect = displacementVector(startPose, inTransformation, meshVertex[vertexHandle], vertexNormals[vertexHandle]);


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
				MPoint pointPosition = allPoints[index], vertexHandleDisplacement = allPoints[index];

				//////////////////////////////////////////////////////////////////////////////////////////
				//////////////////////////		VERTEX HANDLE DISPLACEMENT		//////////////////////////

				if (vertexHandleDisplacement.distanceTo(meshVertex[vertexHandle]) <= radius)
				{
					double falloff, uCoef, vCoef;
					falloff = smoothStep(vertexHandleDisplacement.distanceTo(meshVertex[vertexHandle]), 0.0, radius);
					// GET U AND V COMPONENTS
					// POJECT DISPLACEMENT VECT ON U AND V
					uCoef = displaceVect.length()*cos(displaceVect.angle(uTangents[pGeoIterator.index()]));

					vCoef = displaceVect.length()*cos(displaceVect.angle(vTangents[pGeoIterator.index()]));
					//pointPosition += MPoint(displaceVect) + (pointPosition-meshVertex[vertexHandle]);
					vertexHandleDisplacement += ((vCoef*vTangents[pGeoIterator.index()].normal()) + (uCoef*uTangents[pGeoIterator.index()].normal()))*falloff*inputEnvelope*displacement;

				}
				//////////////////////////		VERTEX HANDLE DISPLACEMENT		//////////////////////////
				//////////////////////////////////////////////////////////////////////////////////////////


				//////////////////////////////////////////////////////////////////////////////////////////
				////////////////////////////////	PREDICTED POSITION	 /////////////////////////////////
				MPoint predictedPosition;

				//////////////////////////////////////////////////////////////////////////////////////////
				////////////////////////////////	EDGE RESISTANCE FORCE	//////////////////////////////
				// SET INDEX: for meshVertexIterator
				int prevVertex;
				meshVertIt.setIndex(index, prevVertex);

				// GET CONNECTION EDGES: of current vertex
				MIntArray connectingEdges;
				meshVertIt.getConnectedEdges(connectingEdges);


				// GET RESULTING FORCE APPLIED ON VERTEX
				MVector vertexForce = MVector(0, 0, 0);
				for (int edge = 0; edge < connectingEdges.length(); edge++)
				{
					int edgeIndex = connectingEdges[edge];
					// GET VERT ID FOR EDGE I
					int2 vertID;
					fnInputMesh.getEdgeVertices(edgeIndex, vertID);
					currentEdgeLength[edgeIndex] = allPoints[vertID[1]].distanceTo(allPoints[vertID[0]]);
					// CALCULATING FORCE
					if (currentEdgeLength[edgeIndex] < restEdgeLength[edgeIndex] * 0.5 || currentEdgeLength[edgeIndex] > restEdgeLength[edgeIndex] * 0.5)
					{
						/*cerr << "differenceFound \n";
						cerr << "restEdge: " << restEdgeLength[edgeIndex]<<"\n";
						cerr << "currentEdge: " << currentEdgeLength[edgeIndex] << "\n";*/
						double deltaLen = currentEdgeLength[edgeIndex] - restEdgeLength[edgeIndex];
						double elasticForce = inElasticity * deltaLen *0.5;
						int endPointIndex;
						MVector direction;

						// CHECKING WHICH OF THE POINTS FORMING THE EDGE IS NOT THE CURRENT VERTEX
						if (vertID[0] != index)
						{
							endPointIndex = vertID[0];
						}
						else
						{
							endPointIndex = vertID[1];
						}
						// CLACULATING DIRECTION VECTOR OF FORCE
						direction = allPoints[endPointIndex] - allPoints[index];

						// UPDATING VERTEX FORCE
						vertexForce += direction * elasticForce;
						

					}
					
				}
				////////////////////////////////	EDGE RESISTANCE FORCE	//////////////////////////////
				//////////////////////////////////////////////////////////////////////////////////////////


				// COMPUTE NEW COMPONENT POSITION
				if (vertexForce != MVector(0, 0, 0))
				{
					vertexForce = vertexForce / connectingEdges.length();
					predictedPosition = allPoints[index] + vertexForce;
					MPoint offset_pos;
					offset_pos = (predictedPosition - pointPosition) * inputEnvelope;
					newMeshVertex[index] = pointPosition + offset_pos / (inSteps - step);
					
				}

				////////////////////////////////	PREDICTED POSITION	  ////////////////////////////////
				//////////////////////////////////////////////////////////////////////////////////////////				
				newMeshVertex[index] = vertexHandleDisplacement;
				cerr << "----------> VERTEX HANDLE DISPLACEMENT:  " << vertexHandleDisplacement.x << " " << vertexHandleDisplacement.y << " " << vertexHandleDisplacement.z << "\n ";
			}
			// UPDATE POSITION
			allPoints.copy(newMeshVertex);
			

		}

	}
	pGeoIterator.setAllPositions(allPoints);
	//restEdgeLength.copy(currentEdgeLength);

	//fnInputMesh.setPoints(deformedPointArray);

		
	return MStatus::kSuccess;

	

}

MStatus asSlidingSkinning::nodeInitializer()
{
	
	// INITIALIZATIONS
	as_restPoseEvaluated = 0;

	MFnNumericAttribute numericAttributeFn;


	// INPUTS
	// Strength
	as_strength = numericAttributeFn.create("strength", "strength", MFnNumericData::kDouble, 0.5);
	numericAttributeFn.setKeyable(1);
	numericAttributeFn.setMin(0.0);
	//numericAttributeFn.setMax(1.0);
	addAttribute(as_strength);
	// Iterations
	as_iterations = numericAttributeFn.create("simulationIterations", "itr", MFnNumericData::kInt, 1);
	numericAttributeFn.setMin(0.0);
	numericAttributeFn.setKeyable(1);
	addAttribute(as_iterations);
	// Steps
	as_steps = numericAttributeFn.create("steps", "stp", MFnNumericData::kInt, 3);
	numericAttributeFn.setMin(0.0);
	//numericAttributeFn.setMax(1.0);
	numericAttributeFn.setKeyable(1);
	addAttribute(as_steps);
	// Elasticuty
	as_elasticity = numericAttributeFn.create("elasticuty", "elas", MFnNumericData::kDouble, 0.2);
	numericAttributeFn.setMin(0.0);
	//numericAttributeFn.setMax(1.0);
	numericAttributeFn.setKeyable(1);
	addAttribute(as_elasticity);

	// vtxHandle
	as_vertexHandle = numericAttributeFn.create("vertexId", "vtx", MFnNumericData::kInt, 0);
	numericAttributeFn.setMin(0.0);
	numericAttributeFn.setKeyable(1);
	addAttribute(as_vertexHandle);
	// Radius
	as_radius = numericAttributeFn.create("radius", "radius", MFnNumericData::kFloat, 0);
	//numericAttributeFn.setMin(0.0);
	//numericAttributeFn.setMax(10.0);
	numericAttributeFn.setKeyable(1);
	addAttribute(as_radius);
	// Elasticuty
	as_elasticity = numericAttributeFn.create("elasticity", "elasticity", MFnNumericData::kFloat, 0);
	numericAttributeFn.setMin(0.0);
	numericAttributeFn.setMax(1.0);
	numericAttributeFn.setKeyable(1);
	addAttribute(as_elasticity);
	// Displacement
	as_displacement = numericAttributeFn.create("displacement", "disp", MFnNumericData::kFloat, 1);
	numericAttributeFn.setMin(0.0);
	numericAttributeFn.setMax(10.0);
	numericAttributeFn.setKeyable(1);
	addAttribute(as_displacement);
	// Transformationm_tx = numericAttributeFn.create("outTranslateX", "tx", MFnNumericData::kDouble);
	as_tx = numericAttributeFn.create("translateX", "tx", MFnNumericData::kDouble);
	as_ty = numericAttributeFn.create("translateY", "ty", MFnNumericData::kDouble);
	as_tz = numericAttributeFn.create("translateZ", "tz", MFnNumericData::kDouble);

	as_transformation = numericAttributeFn.create("translate", "translate", as_tx, as_ty, as_tz);
	addAttribute(as_transformation);


	// ATTRIBUTE AFFECTS
	attributeAffects(as_strength, outputGeom);
	attributeAffects(as_iterations, outputGeom);
	attributeAffects(as_steps, outputGeom);
	attributeAffects(as_elasticity, outputGeom);
	attributeAffects(as_vertexHandle, outputGeom);
	attributeAffects(as_radius, outputGeom);
	attributeAffects(as_elasticity, outputGeom);
	attributeAffects(as_displacement, outputGeom);
	attributeAffects(as_transformation, outputGeom);	
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

double asSlidingSkinning::clamp(double value, double min, double max)
{
	if (value < min)
	{
		return min;
	}
	if (value > max)
	{
		return max;
	}
	return value;

}
double asSlidingSkinning::smoothStep(double value, double edge0, double edge1)
{
	if (edge0 == edge1)
	{
		return 1.0;
	}
	value = clamp((edge1 - value) / edge1, 0.1, 1.0);
	return value * value*(3 - (2 * value));

}

MVector asSlidingSkinning::displacementVector(MVector startPose, MVector transformation, MPoint vertexPoz, MVector vertexNormal)
{
	double mp, d;
	MVector dTransform, displaceVect, qPositionVect;
	dTransform = transformation - startPose;
	dTransform = vertexPoz + dTransform;
	mp = dTransform * vertexNormal;
	d = MVector(vertexPoz) * vertexNormal;
	//MVector qPositionVect, displaceVect;

	qPositionVect = dTransform + (d - mp)*vertexNormal;
	displaceVect = qPositionVect - vertexPoz;

	return displaceVect;


}