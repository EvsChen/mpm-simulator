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
#include <SIM/SIM_GeometryCopy.h>
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

#include <cstring>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
#include "direct.h"
#else
#include <sys/stat.h>
#endif

#define PRINT(s) std::cout << s << std::endl;

Params params;
Profiler profiler;

void initializeSIM(void *)
{
	IMPLEMENT_DATAFACTORY(SIM_MPMSolver);
	
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
		_mkdir(params.outFolder.c_str());
#else
		mkdir(params.outFolder.c_str(), 0777);
#endif
		FLAGS_log_dir = params.outFolder;
		google::InitGoogleLogging("Plugin");
		google::InstallFailureSignalHandler();
		google::FlushLogFiles(google::GLOG_INFO);

}

inline Vec3f UTVecToVec3(UT_Vector3 utv) {
	return Vec3f(Float(utv.x()), Float(utv.y()), Float(utv.z()));
}

inline UT_Vector3 VecToUTVec3(Vec3f v) {
	return UT_Vector3(v.x(), v.y(), v.z());
}

SIM_MPMSolver::SIM_MPMSolver(const SIM_DataFactory * factory) 
	: BaseClass(factory), 
	  SIM_OptionsUser(this) 
{
}

SIM_MPMSolver::~SIM_MPMSolver() 
{
}

const SIM_DopDescription * SIM_MPMSolver::getMyOwnSolverDescription()
{
	static PRM_Name	prm_gridx(MPM_GRIDX, "Grid Resolution X");
	static PRM_Name	prm_gridy(MPM_GRIDY, "Grid Resolution Y");
	static PRM_Name	prm_gridz(MPM_GRIDZ, "Grid Resolution Z");
	static PRM_Name prm_spacing(MPM_SPACING, "Grid Spacing");

	static PRM_Name prm_e(MPM_E, "Young's Modulus");
	static PRM_Name prm_nu(MPM_NU, "Poisson's Ratio");
	static PRM_Name prm_density(MPM_DENSITY, "Particle Density");

	static PRM_Name prm_bboxMin(MPM_BBOXMIN, "Bounding Box Min");
	static PRM_Name prm_bboxMax(MPM_BBOXMAX, "Bounding Box Max");
	static PRM_Name prm_timestep(MPM_TIMESTEP, "Timestep");

	static PRM_Default prm_grid_dft(30);
	static PRM_Default prm_spacing_dft(1e-2f);
	static PRM_Default prm_e_dft(5.f);
	static PRM_Default prm_nu_dft(0.2f);
	static PRM_Default prm_density_dft(1e3f);
	static PRM_Default prm_bboxMin_dft[] = { PRM_Default(-1), PRM_Default(-1),PRM_Default(-1) };
	static PRM_Default prm_bboxMax_dft[] = { PRM_Default(1), PRM_Default(1),PRM_Default(1) };
	static PRM_Default prm_timestep_dft(5e-4f);
	static PRM_Template theTemplates[] =
	{
		PRM_Template(PRM_INT_J, 1, &prm_gridx, &prm_grid_dft),
		PRM_Template(PRM_INT_J, 1, &prm_gridy, &prm_grid_dft),
		PRM_Template(PRM_INT_J, 1, &prm_gridz, &prm_grid_dft),
		PRM_Template(PRM_FLT_J, 1, &prm_spacing, &prm_spacing_dft),
		PRM_Template(PRM_FLT_J, 1, &prm_e, &prm_e_dft),
		PRM_Template(PRM_FLT_J, 1, &prm_nu, &prm_nu_dft),
		PRM_Template(PRM_FLT_J, 1, &prm_density, &prm_density_dft),
		PRM_Template(PRM_XYZ_J, 3, &prm_bboxMin, prm_bboxMin_dft),
		PRM_Template(PRM_XYZ_J, 3, &prm_bboxMax, prm_bboxMax_dft),
		PRM_Template(PRM_FLT_J, 1, &prm_timestep, &prm_timestep_dft),
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

inline Vec3f SIM_MPMSolver::worldToLocal(Vec3f p0)
{
	return (p0 - worldMin).cwiseProduct(worldDiffInv).cwiseProduct(localDiff) + localMin;
}

inline Vec3f SIM_MPMSolver::localToWorld(Vec3f p1)
{
	return (p1 - localMin).cwiseProduct(localDiffInv).cwiseProduct(worldDiff) + worldMin;
}


SIM_Solver::SIM_Result SIM_MPMSolver::solveSingleObjectSubclass(SIM_Engine & engine, 
	SIM_Object & object, SIM_ObjectArray & feedbackToObjects, const SIM_Time & timeStep, bool objectIsNew)
{	
	// Initialize

	// Set Params
	params.setMaterial(getE(), getNu(), getDensity());
	params.pType = ParticleType::ELASTIC;
	params.timeStep = getTimestep();
	params.spacing = getSpacing();
	params.gridX = getGridX();
	params.gridY = getGridY();
	params.gridZ = getGridZ();
	params.setOutput(true, false);
	params.log();
	google::FlushLogFiles(google::GLOG_INFO);

	// Set BouningBox
	worldMin = UTVecToVec3(getBBoxMin());
	worldMax = UTVecToVec3(getBBoxMax());
	worldDiff = worldMax - worldMin;
	worldDiffInv = Vec3f(1 / worldDiff.x(), 1 / worldDiff.y(), 1 / worldDiff.z());
	localMin = Vec3f(0, 0, 0);
	localMax = Vec3f(params.gridX * params.spacing, params.gridY * params.spacing, params.gridZ * params.spacing);
	localDiff = localMax - localMin;
	localDiffInv = Vec3f(1 / localDiff.x(), 1 / localDiff.y(), 1 / localDiff.z());

	// Get the object's last state before this time step
	const SIM_Geometry* geometry(object.getGeometry());
	if (!geometry)
	{
		return SIM_SOLVER_FAIL;
	}
	// Extract simulation state from geometry
	GU_ConstDetailHandle gdh = geometry->getOwnGeometry();
	const GU_Detail* gdp = gdh.gdp();
	GA_ROHandleV3 vhnd(gdp->findPointAttribute("v"));
	std::vector<Vec3f> positions;
	std::vector<Vec3f> velocities;
	for (GA_Iterator it(gdp->getPointRange()); !it.atEnd(); ++it)
	{
		Vec3f p = worldToLocal(UTVecToVec3(gdp->getPos3(*it)));
		Vec3f v = vhnd.get(*it);
		positions.push_back(p);
	}

	

	// Initialize Engine
	Engine MPMEngine(positions);

		// Integrate simulation state forward by time step
		//MPMEngine->P2GTransfer();
		//LOG(INFO) << "P2G";
		//google::FlushLogFiles(google::GLOG_INFO);
		//MPMEngine->updateGridState();
		//LOG(INFO) << "UPDATE STATE";
		//google::FlushLogFiles(google::GLOG_INFO);
		//MPMEngine->updateDeformGrad();
		//LOG(INFO) << "UPDATE DEFORM GRAD";
		//google::FlushLogFiles(google::GLOG_INFO);

		//MPMEngine->particleList_.hardening();
		//LOG(INFO) << "HADERDENING";
		//google::FlushLogFiles(google::GLOG_INFO);
		//MPMEngine->G2PTransfer();
		//LOG(INFO) << "G2P";
		//google::FlushLogFiles(google::GLOG_INFO);

		//MPMEngine->grid_.reset();
		//LOG(INFO) << "RESET";
		//google::FlushLogFiles(google::GLOG_INFO);

		//MPMEngine->particleList_.advection();
		//LOG(INFO) << "ADVECT";		
		//google::FlushLogFiles(google::GLOG_INFO);


	// Write Positions Back
	SIM_GeometryCopy* geometryCopy(
		SIM_DATA_CREATE(
			object, SIM_GEOMETRY_DATANAME, SIM_GeometryCopy,
			SIM_DATA_RETURN_EXISTING | SIM_DATA_ADOPT_EXISTING_ON_DELETE
		)
	);

	GU_DetailHandleAutoWriteLock lock(geometryCopy->getOwnGeometry());
	if (lock.isValid()) 
	{
		GU_Detail* gdp = lock.getGdp();
		int idx = 0;
		for (GA_Iterator it(gdp->getPointRange()); !it.atEnd(); ++it)
		{
			UT_Vector3 newpos = VecToUTVec3(localToWorld(MPMEngine.particleList_.getPosition(idx)));
			// const UT_Vector3 newpos = gdp->getPos3(*it) + UT_Vector3(0, 5, 0);
			gdp->setPos3(*it, newpos);
			idx++;
		}
	}

	return SIM_SOLVER_SUCCESS;
	// return SIM_SOLVER_FAIL;
}


