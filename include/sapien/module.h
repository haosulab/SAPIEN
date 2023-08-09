// #pragma once
// #include <memory>
// #include <vector>

// namespace sapien {
// class Entity;
// class Scene;

// class Module {
// public:
//   Module() {}

//   std::vector<std::shared_ptr<Entity>> getEntities() const { return mEntities; }

//   std::shared_ptr<Scene> getScene() const;
//   void internalSetScene(Scene *scene);

//   // TODO: make sure each entity can only be added to 1 module

//   void addEntity(std::shared_ptr<Entity> entity);
//   void removeEntity(std::shared_ptr<Entity> entity);

//   void setName(std::string name) { mName = name; };
//   std::string getName() const { return mName; };

//   template <class Archive> void save(Archive &ar) const { ar(mName, mEntities); }
//   template <class Archive> void load(Archive &ar) const { ar(mName, mEntities); }

// private:
//   std::string mName;
//   std::vector<std::shared_ptr<Entity>> mEntities;
//   Scene *mScene{};
// };

// }; // namespace sapien
