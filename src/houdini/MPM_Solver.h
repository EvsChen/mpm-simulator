#ifndef __MPM_Solver_h__
#define __MPM_Solver_h__

#include <UT/UT_IStream.h>
#include <GA/GA_Types.h>
#include <GU/GU_DetailHandle.h>
#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_SingleSolver.h>
#include <SIM/SIM_Geometry.h>

#include "../engine.h"
#include "../particle.h"

// Grid
#define MPM_GRIDX "gridx"
#define MPM_GRIDY "gridy"
#define MPM_GRIDZ "gridz"
#define MPM_SPACING "spacing"

// Young's modulus
#define MPM_E "e"
// Poisson's ratio
#define MPM_NU "nu"
// Particle Density
#define MPM_DENSITY "density"

// BoundingBox
#define MPM_BBOXMIN "bboxMin"
#define MPM_BBOXMAX "bboxMax"

// Simulation
#define MPM_TIMESTEP "timestep"

class SIM_MPMSolver : public SIM_SingleSolver,
	public SIM_OptionsUser
{
public:	
	// I: int64
	// F: fpreal64
	// V3: UT_Vector3
	GETSET_DATA_FUNCS_I(MPM_GRIDX, GridX);
	GETSET_DATA_FUNCS_I(MPM_GRIDY, GridY);
	GETSET_DATA_FUNCS_I(MPM_GRIDZ, GridZ);
	GETSET_DATA_FUNCS_F(MPM_SPACING, Spacing);

	GETSET_DATA_FUNCS_F(MPM_E, E);
	GETSET_DATA_FUNCS_F(MPM_NU, Nu);
	GETSET_DATA_FUNCS_F(MPM_DENSITY, Density);

	GETSET_DATA_FUNCS_V3(MPM_BBOXMIN, BBoxMin);
	GETSET_DATA_FUNCS_V3(MPM_BBOXMAX, BBoxMax);

	GETSET_DATA_FUNCS_F(MPM_TIMESTEP, Timestep);

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

	Vec3f worldMin;
	Vec3f worldMax;
	Vec3f worldDiff;
	Vec3f worldDiffInv;	// (1/x, 1/y, 1/z)
	Vec3f localMin;
	Vec3f localMax;
	Vec3f localDiff;
	Vec3f localDiffInv;		// (1/x, 1/y, 1/z)

	inline Vec3f worldToLocal(Vec3f p0);
	inline Vec3f localToWorld(Vec3f p1);

	uPtr<Engine> MPMEngine;
};

#endif

