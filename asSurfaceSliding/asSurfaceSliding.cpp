#include "asSurfaceSliding.h"

#include <cstdlib>

MTypeId asSurfaceSliding::as_id(0x09465);
MObject asSurfaceSliding::as_vertexHandle;
MObject asSurfaceSliding::as_radius;
MObject asSurfaceSliding::as_displacement;
MObject asSurfaceSliding::as_elasticity;

MObject asSurfaceSliding::as_transformation;
MObject asSurfaceSliding::as_tx;
MObject asSurfaceSliding::as_ty;
MObject asSurfaceSliding::as_tz;
int asSurfaceSliding::as_restPoseEvaluated;


void* asSurfaceSliding::nodeCreator() { return new asSurfaceSliding; }

MStatus asSurfaceSliding::deform(MDataBlock& pDataBlock, MItGeometry& pGeoIterator, const MMatrix& pMatrix, unsigned int intGeometryIndex)
{
	MStatus status;
	// GETTING DATA
	MDataHandle envData = pDataBlock.inputValue(envelope, &status);
	if (status != MS::kSuccess) return status;

	float inputEnvelope = envData.asFloat();
	int vertexHandle = pDataBlock.inputValue(as_vertexHandle, &status).asInt();
	float radius = pDataBlock.inputValue(as_radius, &status).asFloat();
	float elasticity = pDataBlock.inputValue(as_elasticity, &status).asFloat();
	float displacement = pDataBlock.inputValue(as_displacement, &status).asFloat();
	MVector inTransformation = pDataBlock.inputValue(as_transformation).asVector();
	MVector startPose, dTransform;

	MPointArray deformedPointArray;

	// Undeformed Mesh
	MArrayDataHandle inputMeshHandle = pDataBlock.outputArrayValue(input, &status);
	inputMeshHandle.jumpToElement(0);
	MDataHandle inputMeshElementHandle = inputMeshHandle.outputValue(&status);
	MDataHandle inputMeshGeomHandle = inputMeshElementHandle.child(outputGeom);
	MObject inputMeshObj = inputMeshGeomHandle.data();
	MFnMesh fnInputMesh(inputMeshObj, &status);

	// EDGE LENGTH
	int numEdges = fnInputMesh.numEdges();
	static MDoubleArray restEdgeLength(numEdges, 0.0), currentEdgeLength(numEdges, 0.0);
		   
	// GET MESH POINTS
	MPointArray meshVertex;
	fnInputMesh.getPoints(meshVertex, MSpace::kObject);
	MVectorArray vertexForce(meshVertex.length());
	int numVertex = meshVertex.length();
	MVectorArray restVertexArray(numVertex);
	static MVectorArray previousVertexArray(numVertex);

	// INITIALIZATION STAGE
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
			restEdgeLength[i] = meshVertex[vertID[1]].distanceTo(meshVertex[vertID[0]]);
		}
		// GETTING REST VERTEX
		for (int i = 0; i < numVertex; i++)
		{
			restVertexArray[i] = meshVertex[i];
			
			previousVertexArray[i] = meshVertex[i];
		}
		// GET NEIBOURING POINTS		

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
		cerr << "vtx Normal: " << vertexNormals[i].x << " " << vertexNormals[i].y << " " << vertexNormals[i].z << "\n";
		// Checking if the normal is different than the arbitray chosen vects
		if (tg1 != MVector(0, 0, 0))
		{
			uTangents.append(tg1);
			cerr << "uTangents : " << uTangents[i].x << " " << uTangents[i].y << " " << uTangents[i].z << "\n";
		}
		else
		{
			uTangents.append(tg2);
			cerr << "uTangents : " << uTangents[i].x << " " << uTangents[i].y << " " << uTangents[i].z << "\n";
		}

		vTangents.append(vertexNormals[i] ^ uTangents[i]);
	}

	// GET TRANSFORM PROJECTION ON SURFACE
	// Calculate displacement for vtxHandle

	MVector displaceVect = displacementVector(startPose, inTransformation, meshVertex[vertexHandle], vertexNormals[vertexHandle]);


	// SKIN SLIDING
	// GETTING CURRENT EDGE LENGTH
	for (int i = 0; i < numEdges; i++)
	{
		// GET VERT ID FOR EDGE I
		int2 vertID;
		fnInputMesh.getEdgeVertices(i, vertID);
		currentEdgeLength[i] = meshVertex[vertID[1]].distanceTo(meshVertex[vertID[0]]);
		if (currentEdgeLength[i] != restEdgeLength[i])
		{
			double deltaLen = currentEdgeLength[i] - restEdgeLength[i];
			double elasticForce = elasticity * deltaLen;
			MVector direction;

			// CHECKING WHICH VERTEX HAS BEEN MANIPULATED
			if (meshVertex[vertID[0]].distanceTo(restVertexArray[vertID[0]]) > meshVertex[vertID[1]].distanceTo(restVertexArray[vertID[1]]))
			{
				direction = meshVertex[vertID[1]] - meshVertex[vertID[0]];
				vertexForce[vertID[0]] += direction.normal()*elasticForce;				

			}
			if (meshVertex[vertID[0]].distanceTo(restVertexArray[vertID[0]]) <= meshVertex[vertID[1]].distanceTo(restVertexArray[vertID[1]]))
			{
				direction = meshVertex[vertID[0]] - meshVertex[vertID[1]];
				vertexForce[vertID[1]] += direction.normal()*elasticForce;
		

			}

		}
	}
	
	// Get transformation on u and v tangents
	// Apply transformations to vtxHandle

	for (; !pGeoIterator.isDone(); pGeoIterator.next())
	{
		// GETTING WEIGHTS
		float weight = weightValue(pDataBlock, intGeometryIndex, pGeoIterator.index());
		MPoint pointPosition = pGeoIterator.position(MSpace::kObject);
		float weightVal = 0.1;
		//if (pGeoIterator.index()!=vertexHandle)
		pointPosition += vertexForce[pGeoIterator.index()] * inputEnvelope;

		if (pointPosition.distanceTo(meshVertex[vertexHandle]) <= radius)
		{
			double falloff, uCoef, vCoef;
			falloff = smoothStep(pointPosition.distanceTo(meshVertex[vertexHandle]), 0.0, radius);
			// GET U AND V COMPONENTS
			// POJECT DISPLACEMENT VECT ON U AND V
			uCoef = displaceVect.length()*cos(displaceVect.angle(uTangents[pGeoIterator.index()]));
			vCoef = displaceVect.length()*cos(displaceVect.angle(vTangents[pGeoIterator.index()]));
			
			pointPosition +=  ((vCoef*vTangents[pGeoIterator.index()].normal()) + (uCoef*uTangents[pGeoIterator.index()].normal()))*falloff*inputEnvelope*displacement;

		}

		deformedPointArray.append(pointPosition);
	}


	pGeoIterator.setAllPositions(deformedPointArray);
	fnInputMesh.setPoints(deformedPointArray);
	

	return MStatus::kSuccess;
}
MStatus asSurfaceSliding::nodeInitializer()
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
	// INITIALIZATIONS
	as_restPoseEvaluated = 0;

	MFnNumericAttribute numericAttributeFn;


	// INPUTS
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
	attributeAffects(as_vertexHandle, outputGeom);
	attributeAffects(as_radius, outputGeom);
	attributeAffects(as_elasticity, outputGeom);
	attributeAffects(as_displacement, outputGeom);
	attributeAffects(as_transformation, outputGeom);
	return MStatus::kSuccess;
}

double asSurfaceSliding::clamp(double value, double min, double max)
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
double asSurfaceSliding::smoothStep(double value, double edge0, double edge1)
{
	if (edge0 == edge1)
	{
		return 1.0;
	}
	value = clamp((edge1-value) / edge1 , 0.1, 1.0);
	return value*value*(3-(2*value));

}

MVector asSurfaceSliding::displacementVector(MVector startPose, MVector transformation, MPoint vertexPoz, MVector vertexNormal)
{
	double mp, d;
	MVector dTransform, displaceVect, qPositionVect;

	dTransform = transformation - startPose;
	dTransform = vertexPoz + dTransform;
	//cerr << "dTransform: " << dTransform.x << " " << dTransform.y << " " << dTransform.z << "\n ";
	//cerr << "vertexNormal: " << vertexNormal.x << " " << vertexNormal.y << " " << vertexNormal.z << "\n ";
	mp = dTransform * vertexNormal;
	//cerr << "mp: " << mp << "\n";
	d = MVector(vertexPoz) * vertexNormal;
	//MVector qPositionVect, displaceVect;

	qPositionVect = dTransform + (d - mp)*vertexNormal;
	displaceVect = qPositionVect - vertexPoz;
	


	return displaceVect;


}