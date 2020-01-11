#include "filter_shader.h"

namespace sapien {

// constexpr int max_groups = 128;

// int CollisionGroupManager::NewExclusiveGroup() {
//   if (count == max_groups) {
//     std::cerr << "Collision group exhausted" << std::endl;
//     return -1;
//   }
//   exclusiveGroups.insert(count);
//   return count++;
// }

// int CollisionGroupManager::NewReusableGroup(int id) {
//   std::set<int> used;
//   for (int i = 0; i < count; ++i) {
//     used.insert(i);
//   }
//   std::set<int> reusable;
//   std::set_difference(used.begin(), used.end(), exclusiveGroups.begin(), exclusiveGroups.end(),
//                       std::inserter(reusable, reusable.end()));
//   std::set<int> canUse;
//   std::set_difference(reusable.begin(), reusable.end(), reusableGroups[id].begin(),
//                       reusableGroups[id].end(), std::inserter(canUse, canUse.end()));
//   if (canUse.empty()) {
//     if (count == max_groups) {
//       std::cerr << "Collision group exhausted" << std::endl;
//       return -1;
//     }
//     reusableGroups[id].insert(count);
//     return count++;
//   }
//   int group = *canUse.begin();
//   reusableGroups[id].insert(group);
//   return group;
// }

// void CollisionGroupManager::addGroupToData(PxFilterData &data, int group) {
//   int word = group / 32;
//   int bit = group % 32;
//   switch (word) {
//     case 0:
//       data.word0 |= 1 << bit;
//       return;
//     case 1:
//       data.word1 |= 1 << bit;
//       return;
//     case 2:
//       data.word2 |= 1 << bit;
//       return;
//     case 3:
//       data.word3 |= 1 << bit;
//       return;
//   }
//   return;
// }

}
