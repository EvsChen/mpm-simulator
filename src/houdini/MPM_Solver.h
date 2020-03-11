#ifndef __MPM_Solver_h__
#define __MPM_Solver_h__

#include <UT/UT_IStream.h>
#include <GA/GA_Types.h>
#include <GU/GU_DetailHandle.h>
#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_SingleSolver.h>
#include <SIM/SIM_Geometry.h>

#include "engine.h"

#define MPM_DENSITY "density"

class SIM_MPMSolver : public SIM_SingleSolver,
	public SIM_OptionsUser
{
public:
	GETSET_DATA_FUNCS_F(MPM_DENSITY, Density);
protected:
	explicit SIM_MPMSolver(const SIM_DataFactory *factory);
	virtual ~SIM_MPMSolver();

	// This implements your own solver step
	SIM_Result solveSingleObjectSubclass(
		SIM_Engine& engine, SIM_Object& object,
		SIM_ObjectArray& feedbackToObjects,
		const SIM_Time& timeStep,
		bool objectIsNew
	);
private:
	static const SIM_DopDescription* getMyOwnSolverDescription();
	DECLARE_STANDARD_GETCASTTOTYPE();
	DECLARE_DATAFACTORY(SIM_MPMSolver, SIM_SingleSolver, "MPM Solver", getMyOwnSolverDescription());
};

#endif

