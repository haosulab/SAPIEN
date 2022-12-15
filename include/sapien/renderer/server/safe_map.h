#pragma once
#include <shared_mutex>
#include <unordered_map>
#include <vector>

namespace sapien {
namespace Renderer {
namespace server {

typedef std::unique_lock<std::shared_mutex> WriteLock;
typedef std::shared_lock<std::shared_mutex> ReadLock;

template <typename Key, typename Tp> class ts_unordered_map {
public:
  Tp get(Key key, Tp empty) {
    ReadLock lock(mMutex);
    auto it = mMap.find(key);
    if (it != mMap.end()) {
      return it->second;
    }
    return empty;
  }

  Tp get(Key key) {
    ReadLock lock(mMutex);
    return mMap.at(key);
  }

  void set(Key key, Tp value) {
    WriteLock lock(mMutex);
    mMap[key] = std::move(value);
  }

  void erase(Key key) {
    WriteLock lock(mMutex);
    mMap.erase(key);
  }

  std::vector<std::pair<Key, Tp>> flat() const {
    std::vector<std::pair<Key, Tp>> result;
    ReadLock lock(mMutex);
    for (auto &it : mMap) {
      result.push_back(it);
    }
    return result;
  }

  ReadLock lockRead() { return ReadLock(mMutex); }
  WriteLock lockWrite() { return WriteLock(mMutex); }

  std::unordered_map<Key, Tp> const &getMap() const { return mMap; }
  std::unordered_map<Key, Tp> &getMap() { return mMap; }

private:
  mutable std::shared_mutex mMutex;
  std::unordered_map<Key, Tp> mMap;
};

} // namespace server
} // namespace Renderer
} // namespace sapien
