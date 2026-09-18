#include "../src/core/VelcoxECS.h"
#include <iostream>
#include <cassert>
#include <vector>

struct Position {
    float x = 0;
    float y = 0;
};

struct Velocity {
    float vx = 0;
    float vy = 0;
};

struct Tag {
    int value = 0;
};

void TestEntityLifecycle() {
    std::cout << "[Test ECS: Lifecycle] Testing CreateEntity, DestroyEntity, and ID management...\n";
    Velox::EntityManager em;

    Velox::EntityID e0 = em.CreateEntity();
    Velox::EntityID e1 = em.CreateEntity();
    Velox::EntityID e2 = em.CreateEntity();

    assert(e0 != e1 && e1 != e2);

    // Destroy middle entity
    em.DestroyEntity(e1);

    // Create a new entity
    Velox::EntityID eNew = em.CreateEntity();
    assert(eNew != e0 && eNew != e2);

    std::cout << "  -> PASSED: Entity creation, destruction, and ID management verified.\n";
}

void TestComponentRegistrationAndAccess() {
    std::cout << "[Test ECS: Components] Testing Component Registration, Addition, Removal, Queries...\n";
    Velox::EntityManager em;

    em.RegisterComponent<Position>();
    em.RegisterComponent<Velocity>();
    em.RegisterComponent<Tag>();

    Velox::EntityID e1 = em.CreateEntity();
    Velox::EntityID e2 = em.CreateEntity();
    Velox::EntityID e3 = em.CreateEntity();

    // Add components
    em.AddComponent(e1, Position{10.0f, 20.0f});
    em.AddComponent(e1, Velocity{1.0f, 2.0f});

    em.AddComponent(e2, Position{30.0f, 40.0f});

    em.AddComponent(e3, Tag{99});

    // HasComponent check
    assert(em.HasComponent<Position>(e1) == true);
    assert(em.HasComponent<Velocity>(e1) == true);
    assert(em.HasComponent<Tag>(e1) == false);

    assert(em.HasComponent<Position>(e2) == true);
    assert(em.HasComponent<Velocity>(e2) == false);

    // GetComponent check & modification
    auto& pos = em.GetComponent<Position>(e1);
    assert(pos.x == 10.0f && pos.y == 20.0f);
    pos.x = 50.0f;
    assert(em.GetComponent<Position>(e1).x == 50.0f);

    // Query entities with Position
    auto posEntities = em.GetEntitiesWithComponent<Position>();
    assert(posEntities.size() == 2);

    // Remove component
    em.RemoveComponent<Position>(e1);
    assert(em.HasComponent<Position>(e1) == false);

    posEntities = em.GetEntitiesWithComponent<Position>();
    assert(posEntities.size() == 1);
    assert(posEntities[0] == e2);

    // Entity destroyed should automatically clear all attached components
    em.DestroyEntity(e2);
    posEntities = em.GetEntitiesWithComponent<Position>();
    assert(posEntities.empty());

    std::cout << "  -> PASSED: Component attach, mutate, remove, and auto-cleanup verified.\n";
}

int main() {
    std::cout << "=== Running Velox ECS Unit Tests ===\n";
    TestEntityLifecycle();
    TestComponentRegistrationAndAccess();
    std::cout << "=== All ECS Unit Tests Passed Successfully! ===\n";
    return 0;
}
