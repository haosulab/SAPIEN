#pragma once
#include <PxPhysicsAPI.h>
#include <vector>
#include <map>
#include <string>

#define APP_ASSERT_FATAL(exp, msg)                                                                \
  if (!(exp)) {                                                                                   \
    std::cerr << msg << std::endl;                                                                \
    exit(1);                                                                                      \
  }

#define APP_ASSERT_WAN(exp, msg)                                                                  \
  if (!(exp)) {                                                                                   \
    std::cerr << msg << std::endl;                                                                \
  }

using namespace physx;
// TODO: proof read and test this struct
struct PxArticulationWrapper {
  PxArticulationReducedCoordinate *articulation = nullptr;
  PxArticulationCache *cache = nullptr;
  std::vector<PxArticulationLink *> links;
  std::map<std::string, PxArticulationLink *> namedLinks;

  std::vector<PxU32> dofStarts;

  /* dof in articulation */
  uint32_t dof() const;

  /* call to update cache with current articulation */
  void updateCache();
  /* call to apply cache into articulation */
  void updateArticulation();

  /* access qpos by index */
  std::vector<PxReal> qpos(PxU32 index); 
  /* access qpos by name */
  std::vector<PxReal> qpos(const std::string &name); 
  /* set qpos by index */
  void set_qpos(PxU32 index, const std::vector<PxReal> &v);
  /* set all qpos in the articulation */
  void set_qpos_unchecked(const std::vector<PxReal> &v); 
  /* set qpos by name */
  void set_qpos(const std::string &name, const std::vector<PxReal> &v); 

  std::vector<PxReal> qvel(PxU32 index); 
  std::vector<PxReal> qvel(const std::string &name); 
  void set_qvel_unchecked(const std::vector<PxReal> &v);

  std::vector<PxReal> qacc(PxU32 index); 
  std::vector<PxReal> qacc(const std::string &name);

  std::vector<PxReal> qf(PxU32 index);
  std::vector<PxReal> qf(const std::string &name);
  void set_qf_unchecked(const std::vector<PxReal> &v);

  std::string summary() const;
};
