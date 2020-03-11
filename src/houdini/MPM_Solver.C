#include "MPM_Solver.h"

#include <SIM/SIM_Engine.h>
#include <SIM/SIM_Options.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Random.h>
#include <SIM/SIM_RandomTwister.h>
#include <SIM/SIM_Position.h>
#include <SIM/SIM_PRMShared.h>
#include <SIM/SIM_Guide.h>
#include <SIM/SIM_GuideShared.h>
#include <GU/GU_Detail.h>
#include <GU/GU_PrimPart.h>
#include <GU/GU_RayIntersect.h>
#include <GEO/GEO_PrimPoly.h>
#include <GA/GA_Handle.h>
#include <GA/GA_Types.h>
#include <PRM/PRM_Include.h>
#include <UT/UT_DSOVersion.h>
#include <UT/UT_Map.h>
#include <UT/UT_StringStream.h>
#include <UT/UT_Vector3.h>
#include <UT/UT_WorkBuffer.h>
#include <SYS/SYS_Floor.h>
#include <SYS/SYS_Math.h>

void initializeSIM(void *)
{
	IMPLEMENT_DATAFACTORY(SIM_MPMSolver);
}

SIM_MPMSolver::SIM_MPMSolver(const SIM_DataFactory * factory) 
	: BaseClass(factory), 
	  SIM_OptionsUser(this) 
{
}

SIM_MPMSolver::~SIM_MPMSolver() 
{
}

SIM_Solver::SIM_Result SIM_MPMSolver::solveSingleObjectSubclass(SIM_Engine & engine, 
	SIM_Object & object, SIM_ObjectArray & feedbackToObjects, const SIM_Time & timeStep, bool objectIsNew)
{
	SIM_Result result = SIM_SOLVER_SUCCESS;

	return result;
}

const SIM_DopDescription * SIM_MPMSolver::getMyOwnSolverDescription()
{
	static PRM_Name	 density(MPM_DENSITY, "Density");
	static PRM_Template theTemplates[] =
	{
		PRM_Template(PRM_FLT_J,		1, &density, PRMpointOneDefaults),
		PRM_Template()
	};

	static SIM_DopDescription theDopDescription(
		true,					// true, to make this node a DOP
		"hdk_MPMSolver",		// internal name
		"MPM Solver",			// node label
		SIM_SOLVER_DATANAME,	// data name (for details view)
		classname(),			// type of this dop
		theTemplates			// input parameters
	);

	return &theDopDescription;
}
