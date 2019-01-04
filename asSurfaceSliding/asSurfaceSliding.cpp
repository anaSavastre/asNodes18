#include "asSurfaceSliding.h"

#include <cstdlib>

MTypeId asSurfaceSliding::as_id(0x09465);
MObject asSurfaceSliding::as_vertexHandle;
MObject asSurfaceSliding::as_radius;
MObject asSurfaceSliding::as_displacement;
MObject asSurfaceSliding::as_transformation;
MObject asSurfaceSliding::as_tx;
MObject asSurfaceSliding::as_ty;
MObject asSurfaceSliding::as_tz;
int asSurfaceSliding::as_restPoseEvaluated;



MObject asSurfaceSliding::as_outTranslation;
MObject asSurfaceSliding::as_outTx;
MObject asSurfaceSliding::as_outTy;
MObject asSurfaceSliding::as_outTz;

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
	float displacement = pDataBlock.inputValue(as_displacement, &status).asFloat();
	MVector inTransformation = pDataBlock.inputValue(as_transformation).asVector();
	MVector startPose, dTransform;

	MPointArray deformedPointArray;

	// Undeformed Mesh
	MArrayDataHandle inputMeshHandle = pDataBlock.outputArrayValue(input, &status);
	inputMeshHandle.jumpToElement(0);
	MDataHandle inputMeshElementHandle = inputMeshHandle.outputValue(&status);
	MDataHandle inputMeshGeomHandle = inputMeshElementHandle.child(inputGeom);
	MObject inputMeshObj = inputMeshGeomHandle.data();
	MFnMesh fnInputMesh(inputMeshObj, &status);
	   
	// Output Mesh
	// MArrayDataHandle outputMeshHandle = pDataBlock.outputArrayValue(output, )

	// GET MESH POINTS
	MPointArray meshVertex;
	fnInputMesh.getPoints(meshVertex, MSpace::kWorld);


	// INITIALIZATION STAGE
	if (as_restPoseEvaluated == 0)
	{
		as_restPoseEvaluated = 1;
		startPose = inTransformation;

	}
	dTransform = inTransformation - startPose;
	dTransform = meshVertex[vertexHandle] + dTransform;

	// CREATE PLANE 
	// Normals
	MFloatVectorArray uTangents, vTangents, vertexNormals;
	status = fnInputMesh.getVertexNormals(true, vertexNormals, MSpace::kWorld);
	if (!status)
	{
		MGlobal::displayError("Failed to get narmals");
		return MStatus::kFailure;

	}
	// U Tangents
	//status = fnInputMesh.getTangents(uTangents, MSpace::kObject);
	//status = fnInputMesh.getNormals(uTangents, MSpace::kObject);

	if (!status)
	{
		MGlobal::displayError("Failed to get u Tangents");
		return MStatus::kFailure;

	}
	
	// V Tangents
	for (unsigned int i = 0; i < vertexNormals.length(); i++)
	{
		//Rotation about x axis
		uTangents.append(vertexNormals[i]);
		double aux;
		aux = uTangents[i].y;
		uTangents[i].y = -uTangents[i].z;
		uTangents[i].z = aux;
		vTangents.append(vertexNormals[i] ^ uTangents[i]);
	}

	// GET TRANSFORM PROJECTION ON SURFACE
	//MMeshIntersector meshIntresection;
    //status = meshIntresection.create(inputMeshObj, pMatrix);
	//MVector projectionOnMesh;
	// Calculate displacement for vtxHandle
	double mp, d, uCoef, vCoef;
	mp = dTransform * vertexNormals[vertexHandle];
	d = MVector(meshVertex[vertexHandle]) * vertexNormals[vertexHandle];
	MVector qPositionVect, displaceVect;
	qPositionVect = dTransform + (d - mp)*vertexNormals[vertexHandle];
	displaceVect = qPositionVect - meshVertex[vertexHandle];




	/*float u, v;
	if (status)
	{
		// Intersection Point
		MPointOnMesh pointOnMesh;
		status = meshIntresection.getClosestPoint(MPoint(inTransformation), pointOnMesh);
		if (status)
		{
			// GET VECT(vtxHandle, projectionPoint)
			projectionOnMesh = pointOnMesh.getPoint();
			pointOnMesh.getBarycentricCoords(u, v);
			
		}
		else
		{
			MGlobal::displayError("Failed to get closest point");

		}
	}
	else
	{
		MGlobal::displayError("Failed to create intersector");
	}*/

	
	// Get transformation on u and v tangents
	// Apply transformations to vtxHandle

	for (; !pGeoIterator.isDone(); pGeoIterator.next())
	{
		// GETTING WEIGHTS
		float weight = weightValue(pDataBlock, intGeometryIndex, pGeoIterator.index());
		MPoint pointPosition = pGeoIterator.position(MSpace::kWorld);
		float weightVal = 0.1;
		if (pointPosition.distanceTo(meshVertex[vertexHandle]) <= radius)
		{
			double falloff;
			falloff = smoothStep(pointPosition.distanceTo(meshVertex[vertexHandle]), 0.0, radius);
			// GET U AND V COMPONENTS
			// POJECT DISPLACEMENT VECT ON U AND V
			uCoef = displaceVect.length()*cos(displaceVect.angle(uTangents[pGeoIterator.index()]));

			vCoef = displaceVect.length()*cos(displaceVect.angle(vTangents[pGeoIterator.index()]));
			//pointPosition += MPoint(displaceVect) + (pointPosition-meshVertex[vertexHandle]);
			pointPosition +=  ((vCoef*vTangents[pGeoIterator.index()].normal()) + (uCoef*uTangents[pGeoIterator.index()].normal()))*falloff;

		}
		deformedPointArray.append(pointPosition);
	}


	pGeoIterator.setAllPositions(deformedPointArray);



	//OUTPUTS
	//Translation Handle
	/*outTranslationHandle = pDataBlock.outputValue(as_outTranslation);
	outTranslationHandle.setMVector(outTranslation);
	outTranslationHandle.setClean();
	*/

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
	// Displacement
	as_displacement = numericAttributeFn.create("displacement", "disp", MFnNumericData::kFloat, 0);
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
	// OUTPUTS	
	// outTranslation = numericAttributeFn.create("outTranslateX", "tx", MFnNumericData::kDouble);
	/*
	as_outTx = numericAttributeFn.create("outTranslateX", "outTx", MFnNumericData::kDouble);
	as_outTy = numericAttributeFn.create("outTranslateY", "outTy", MFnNumericData::kDouble);
	as_outTz = numericAttributeFn.create("outTranslateZ", "outTz", MFnNumericData::kDouble);

	as_outTranslation = numericAttributeFn.create("outTranslate", "outTranslate", as_tx, as_ty, as_tz);
	addAttribute(as_outTranslation);
	
	*/
	// ATTRIBUTE AFFECTS
	attributeAffects(as_vertexHandle, outputGeom);
	attributeAffects(as_radius, outputGeom);
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
