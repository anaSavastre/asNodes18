#include "asTrig.h"

MTypeId asTrig::id(0x09942);

MObject asTrig::inFunctionAttr;
MObject asTrig::inputAngleAttr;
MObject asTrig::outputAttr;

void* asTrig::nodeCreator() { return new asTrig; }

MStatus asTrig::compute(const MPlug& pPlug, MDataBlock& pDataBlock)
{
	if (pPlug == outputAttr)
	{
		MDataHandle inFunctionHandle = pDataBlock.inputValue(inFunctionAttr);
		MDataHandle inputAngleHandle = pDataBlock.inputValue(inputAngleAttr);

		MAngle inputValue = inputAngleHandle.asAngle();
		short inputFunction = inFunctionHandle.asShort();

		double outValue;
		switch (inputFunction)
		{
		case 1:
			outValue = sin(inputValue.asRadians());
			break;
		case 2:
			outValue = cos(inputValue.asRadians());
			break;
		case 3:
			outValue = tan(inputValue.asRadians());
			break;
		default:
			outValue = 0.0f;
		}


		MDataHandle outputHandle = pDataBlock.outputValue(outputAttr);
		outputHandle.setDouble(outValue);

		outputHandle.setClean();
	}
	return MStatus::kSuccess;
}

MStatus asTrig::nodeInitializer()
{
	MFnUnitAttribute unitAttributeFn;
	MFnNumericAttribute numericAttributeFn;
	MFnEnumAttribute enumAttributeFn;

	inFunctionAttr = enumAttributeFn.create("function", "function");
	enumAttributeFn.addField("noOperatin", 0);
	enumAttributeFn.addField("sin", 1);
	enumAttributeFn.addField("cos", 2);
	enumAttributeFn.addField("tan", 3);
	addAttribute(inFunctionAttr);

	inputAngleAttr = unitAttributeFn.create("inputAngle",
		"inputAngle", MFnUnitAttribute::kAngle, 0);
	addAttribute(inputAngleAttr);

	outputAttr = numericAttributeFn.create("output",
		"output",
		MFnNumericData::kDouble);
	addAttribute(outputAttr);

	attributeAffects(inFunctionAttr, outputAttr);
	attributeAffects(inputAngleAttr, outputAttr);

	return MStatus::kSuccess;
}