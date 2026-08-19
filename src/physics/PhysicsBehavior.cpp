#include "PhysicsBehavior.h"

namespace Velox {

PhysicsBehaviorRegistry& PhysicsBehaviorRegistry::Get() {
    static PhysicsBehaviorRegistry instance;
    return instance;
}

void PhysicsBehaviorRegistry::Register(std::unique_ptr<IPhysicsBehavior> behavior) {
    m_behaviors.push_back(std::move(behavior));
}

const std::vector<std::unique_ptr<IPhysicsBehavior>>& PhysicsBehaviorRegistry::GetBehaviors() const {
    return m_behaviors;
}

} // namespace Velox
