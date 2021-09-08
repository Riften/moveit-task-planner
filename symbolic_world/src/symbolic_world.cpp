//
// Created by yongxi on 9/2/21.
//

#include <symbolic_world/symbolic_world.h>

#include <utility>

SymbolicWorld::SymbolicWorld(collision_detection::WorldPtr world) {
    moveit_world_ = std::move(world);
}

void SymbolicWorld::init() {
    // moveit_world_->addObserver();
}