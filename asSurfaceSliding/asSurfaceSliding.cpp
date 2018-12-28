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

	// OUTPUTS
	//MDataHandle outTranslationHandle;



	// Undeformed Mesh
	MArrayDataHandle inputMeshHandle = pDataBlock.inputArrayValue(input, &status);
	inputMeshHandle.jumpToElement(0);
	MDataHandle inputMeshElementHandle = inputMeshHandle.inputValue(&status);
	MDataHandle inputMeshGeomHandle = inputMeshElementHandle.child(inputGeom);
	MObject inputMeshObj = inputMeshGeomHandle.data();
	MFnMesh fnInputMesh(inputMeshObj, &status);

	// GET MESH POINTS
	MPointArray meshVertex;
	fnInputMesh.getPoints(meshVertex, MSpace::kWorld);

	// CREATE PLANE 
	// Normal
	MVector vertexNormal, mp, d;
	status = fnInputMesh.getVertexNormal(vertexHandle, false, vertexNormal, MSpace::kWorld);
	MPlane plane;
	plane.setPlane(vertexNormal, 0.0);


	// GET TRANSFORM PROJECTION ON SURFACE
	MMeshIntersector meshIntresection;
	status = meshIntresection.create(inputMeshObj, pMatrix);
	MVector projectionOnMesh, vertexProjectionVec;
	if (status)
	{
		// Intersection Point
		MPoint projectionPoint(inTransformation.x, inTransformation.y, inTransformation.z, inTransformation.kWaxis);
		MPointOnMesh pointOnMesh;
		status = meshIntresection.getClosestPoint(projectionPoint, pointOnMesh);
		if (status)
		{
			// GET VECT(vtxHandle, projectionPoint)
			projectionOnMesh = pointOnMesh.getPoint();
			mp = MVector(projectionOnMesh) * vertexNormal.normalize();
			d = MVector(meshVertex[vertexHandle]) * vertexNormal.normalize();
			vertexProjectionVec = projectionOnMesh - meshVertex[vertexHandle];
			


		}
		else
		{
			MGlobal::displayError("Failed to get closest point");

		}
	}
	else
	{
		MGlobal::displayError("Failed to create intersector");
	}

	
	// Get transformation on u and v tangents
	// Apply transformations to vtxHandle

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





