#pragma once

#include "../math/Vec2.h"
#include "Containers.h"
#include <cstdint>
#include <algorithm>

namespace Velox {

struct AABB {
    Vec2 min;
    Vec2 max;

    AABB() : min(0.0f, 0.0f), max(0.0f, 0.0f) {}
    AABB(const Vec2& minVal, const Vec2& maxVal) : min(minVal), max(maxVal) {}

    bool Overlaps(const AABB& other) const {
        if (max.x < other.min.x || min.x > other.max.x) return false;
        if (max.y < other.min.y || min.y > other.max.y) return false;
        return true;
    }

    bool Contains(const AABB& other) const {
        return min.x <= other.min.x && min.y <= other.min.y &&
               max.x >= other.max.x && max.y >= other.max.y;
    }

    float GetPerimeter() const {
        float wx = max.x - min.x;
        float wy = max.y - min.y;
        return 2.0f * (wx + wy);
    }

    static AABB Combine(const AABB& a, const AABB& b) {
        return AABB(
            Vec2(std::min(a.min.x, b.min.x), std::min(a.min.y, b.min.y)),
            Vec2(std::max(a.max.x, b.max.x), std::max(a.max.y, b.max.y))
        );
    }
};

struct DynamicTreeNode {
    AABB aabb;
    uint32_t userData; // EntityID
    int32_t parent;
    int32_t left;
    int32_t right;
    int32_t height;

    bool IsLeaf() const { return left == -1; }
};

// ============================================================================
// DynamicTree: Cache-Aligned Flat Array Bounding Volume Hierarchy (BVH)
// ============================================================================
class DynamicTree {
public:
    static constexpr int32_t NullNode = -1;

    DynamicTree(int32_t initialCapacity = 256) 
        : m_root(NullNode), m_freeList(NullNode), m_nodeCount(0) {
        m_nodes.Reserve(initialCapacity);
        m_stack.Reserve(256);
        ExpandPool(initialCapacity);
    }

    int32_t InsertLeaf(uint32_t userData, const AABB& aabb) {
        int32_t leaf = AllocateNode();
        // Fat AABB Margin (2.0f px)
        const float margin = 2.0f;
        m_nodes[leaf].aabb.min = Vec2(aabb.min.x - margin, aabb.min.y - margin);
        m_nodes[leaf].aabb.max = Vec2(aabb.max.x + margin, aabb.max.y + margin);
        m_nodes[leaf].userData = userData;
        m_nodes[leaf].height = 0;
        m_nodes[leaf].left = NullNode;
        m_nodes[leaf].right = NullNode;

        InsertLeafInternal(leaf);
        return leaf;
    }

    void RemoveLeaf(int32_t leaf) {
        assert(leaf >= 0 && static_cast<size_t>(leaf) < m_nodes.Size());
        assert(m_nodes[leaf].IsLeaf());

        RemoveLeafInternal(leaf);
        FreeNode(leaf);
    }

    bool MoveLeaf(int32_t leaf, const AABB& aabb, const Vec2& displacement) {
        assert(leaf >= 0 && static_cast<size_t>(leaf) < m_nodes.Size());
        assert(m_nodes[leaf].IsLeaf());

        if (m_nodes[leaf].aabb.Contains(aabb)) {
            return false; // Still inside fat AABB, 0 tree modifications required
        }

        RemoveLeafInternal(leaf);

        // Expand fat AABB predicting motion displacement
        const float margin = 2.0f;
        AABB fatAABB;
        fatAABB.min = Vec2(aabb.min.x - margin, aabb.min.y - margin);
        fatAABB.max = Vec2(aabb.max.x + margin, aabb.max.y + margin);

        Vec2 d = displacement * 2.0f;
        if (d.x < 0.0f) fatAABB.min.x += d.x;
        else fatAABB.max.x += d.x;
        if (d.y < 0.0f) fatAABB.min.y += d.y;
        else fatAABB.max.y += d.y;

        m_nodes[leaf].aabb = fatAABB;
        InsertLeafInternal(leaf);
        return true;
    }

    template <typename Callback>
    void Query(const AABB& aabb, Callback&& callback) const {
        if (m_root == NullNode) return;

        FixedArray<int32_t, 1024> stack;
        stack.PushBack(m_root);

        while (!stack.IsEmpty()) {
            int32_t nodeId = stack.Back();
            stack.PopBack();

            if (nodeId == NullNode) continue;
            const DynamicTreeNode& node = m_nodes[nodeId];

            if (node.aabb.Overlaps(aabb)) {
                if (node.IsLeaf()) {
                    bool proceed = callback(node.userData);
                    if (!proceed) return;
                } else {
                    if (node.left != NullNode && !stack.IsFull()) stack.PushBack(node.left);
                    if (node.right != NullNode && !stack.IsFull()) stack.PushBack(node.right);
                }
            }
        }
    }

    const AABB& GetFatAABB(int32_t leaf) const {
        assert(leaf >= 0 && static_cast<size_t>(leaf) < m_nodes.Size());
        return m_nodes[leaf].aabb;
    }

    uint32_t GetUserData(int32_t leaf) const {
        assert(leaf >= 0 && static_cast<size_t>(leaf) < m_nodes.Size());
        return m_nodes[leaf].userData;
    }

    void Clear() {
        m_root = NullNode;
        m_freeList = NullNode;
        m_nodeCount = 0;
        m_nodes.Clear();
    }

private:
    void ExpandPool(int32_t count) {
        size_t start = m_nodes.Size();
        m_nodes.Resize(start + count);
        for (size_t i = start; i < m_nodes.Size() - 1; ++i) {
            m_nodes[i].parent = static_cast<int32_t>(i + 1); // Free list linked chain
            m_nodes[i].height = -1;
        }
        m_nodes[m_nodes.Size() - 1].parent = m_freeList;
        m_nodes[m_nodes.Size() - 1].height = -1;
        m_freeList = static_cast<int32_t>(start);
    }

    int32_t AllocateNode() {
        if (m_freeList == NullNode) {
            ExpandPool(m_nodes.Size() == 0 ? 64 : static_cast<int32_t>(m_nodes.Size()));
        }
        int32_t nodeId = m_freeList;
        m_freeList = m_nodes[nodeId].parent;
        m_nodes[nodeId].parent = NullNode;
        m_nodes[nodeId].left = NullNode;
        m_nodes[nodeId].right = NullNode;
        m_nodes[nodeId].height = 0;
        ++m_nodeCount;
        return nodeId;
    }

    void FreeNode(int32_t nodeId) {
        assert(nodeId >= 0 && static_cast<size_t>(nodeId) < m_nodes.Size());
        m_nodes[nodeId].parent = m_freeList;
        m_nodes[nodeId].height = -1;
        m_freeList = nodeId;
        --m_nodeCount;
    }

    void InsertLeafInternal(int32_t leaf) {
        if (m_root == NullNode) {
            m_root = leaf;
            m_nodes[m_root].parent = NullNode;
            return;
        }

        // Surface Area Heuristic (SAH) greedy traversal
        AABB leafAABB = m_nodes[leaf].aabb;
        int32_t index = m_root;
        while (!m_nodes[index].IsLeaf()) {
            int32_t left = m_nodes[index].left;
            int32_t right = m_nodes[index].right;

            float area = m_nodes[index].aabb.GetPerimeter();
            AABB combinedAABB = AABB::Combine(m_nodes[index].aabb, leafAABB);
            float combinedArea = combinedAABB.GetPerimeter();

            float cost = 2.0f * combinedArea;
            float inheritanceCost = 2.0f * (combinedArea - area);

            float costLeft;
            if (m_nodes[left].IsLeaf()) {
                AABB aabb = AABB::Combine(leafAABB, m_nodes[left].aabb);
                costLeft = aabb.GetPerimeter() + inheritanceCost;
            } else {
                AABB aabb = AABB::Combine(leafAABB, m_nodes[left].aabb);
                float oldArea = m_nodes[left].aabb.GetPerimeter();
                float newArea = aabb.GetPerimeter();
                costLeft = (newArea - oldArea) + inheritanceCost;
            }

            float costRight;
            if (m_nodes[right].IsLeaf()) {
                AABB aabb = AABB::Combine(leafAABB, m_nodes[right].aabb);
                costRight = aabb.GetPerimeter() + inheritanceCost;
            } else {
                AABB aabb = AABB::Combine(leafAABB, m_nodes[right].aabb);
                float oldArea = m_nodes[right].aabb.GetPerimeter();
                float newArea = aabb.GetPerimeter();
                costRight = (newArea - oldArea) + inheritanceCost;
            }

            if (cost < costLeft && cost < costRight) break;

            if (costLeft < costRight) index = left;
            else index = right;
        }

        int32_t sibling = index;
        int32_t oldParent = m_nodes[sibling].parent;
        int32_t newParent = AllocateNode();
        m_nodes[newParent].parent = oldParent;
        m_nodes[newParent].userData = 0;
        m_nodes[newParent].aabb = AABB::Combine(leafAABB, m_nodes[sibling].aabb);
        m_nodes[newParent].height = m_nodes[sibling].height + 1;

        if (oldParent != NullNode) {
            if (m_nodes[oldParent].left == sibling) {
                m_nodes[oldParent].left = newParent;
            } else {
                m_nodes[oldParent].right = newParent;
            }
            m_nodes[newParent].left = sibling;
            m_nodes[newParent].right = leaf;
            m_nodes[sibling].parent = newParent;
            m_nodes[leaf].parent = newParent;
        } else {
            m_nodes[newParent].left = sibling;
            m_nodes[newParent].right = leaf;
            m_nodes[sibling].parent = newParent;
            m_nodes[leaf].parent = newParent;
            m_root = newParent;
        }

        // Walk back up the tree refitting AABBs
        index = m_nodes[leaf].parent;
        while (index != NullNode) {
            index = Balance(index);
            int32_t left = m_nodes[index].left;
            int32_t right = m_nodes[index].right;

            assert(left != NullNode && right != NullNode);
            m_nodes[index].height = 1 + std::max(m_nodes[left].height, m_nodes[right].height);
            m_nodes[index].aabb = AABB::Combine(m_nodes[left].aabb, m_nodes[right].aabb);
            index = m_nodes[index].parent;
        }
    }

    void RemoveLeafInternal(int32_t leaf) {
        if (leaf == m_root) {
            m_root = NullNode;
            return;
        }

        int32_t parent = m_nodes[leaf].parent;
        int32_t grandParent = m_nodes[parent].parent;
        int32_t sibling = (m_nodes[parent].left == leaf) ? m_nodes[parent].right : m_nodes[parent].left;

        if (grandParent != NullNode) {
            if (m_nodes[grandParent].left == parent) {
                m_nodes[grandParent].left = sibling;
            } else {
                m_nodes[grandParent].right = sibling;
            }
            m_nodes[sibling].parent = grandParent;
            FreeNode(parent);

            int32_t index = grandParent;
            while (index != NullNode) {
                index = Balance(index);
                int32_t left = m_nodes[index].left;
                int32_t right = m_nodes[index].right;

                m_nodes[index].aabb = AABB::Combine(m_nodes[left].aabb, m_nodes[right].aabb);
                m_nodes[index].height = 1 + std::max(m_nodes[left].height, m_nodes[right].height);
                index = m_nodes[index].parent;
            }
        } else {
            m_root = sibling;
            m_nodes[sibling].parent = NullNode;
            FreeNode(parent);
        }
    }

    int32_t Balance(int32_t iA) {
        DynamicTreeNode* A = &m_nodes[iA];
        if (A->IsLeaf() || A->height < 2) return iA;

        int32_t iB = A->left;
        int32_t iC = A->right;
        DynamicTreeNode* B = &m_nodes[iB];
        DynamicTreeNode* C = &m_nodes[iC];

        int32_t balance = C->height - B->height;

        // Rotate C up
        if (balance > 1) {
            int32_t iF = C->left;
            int32_t iG = C->right;
            DynamicTreeNode* F = &m_nodes[iF];
            DynamicTreeNode* G = &m_nodes[iG];

            C->left = iA;
            C->parent = A->parent;
            A->parent = iC;

            if (C->parent != NullNode) {
                if (m_nodes[C->parent].left == iA) m_nodes[C->parent].left = iC;
                else m_nodes[C->parent].right = iC;
            } else {
                m_root = iC;
            }

            if (F->height > G->height) {
                C->right = iF;
                A->right = iG;
                G->parent = iA;
                A->aabb = AABB::Combine(B->aabb, G->aabb);
                C->aabb = AABB::Combine(A->aabb, F->aabb);
                A->height = 1 + std::max(B->height, G->height);
                C->height = 1 + std::max(A->height, F->height);
            } else {
                C->right = iG;
                A->right = iF;
                F->parent = iA;
                A->aabb = AABB::Combine(B->aabb, F->aabb);
                C->aabb = AABB::Combine(A->aabb, G->aabb);
                A->height = 1 + std::max(B->height, F->height);
                C->height = 1 + std::max(A->height, G->height);
            }
            return iC;
        }

        // Rotate B up
        if (balance < -1) {
            int32_t iD = B->left;
            int32_t iE = B->right;
            DynamicTreeNode* D = &m_nodes[iD];
            DynamicTreeNode* E = &m_nodes[iE];

            B->left = iA;
            B->parent = A->parent;
            A->parent = iB;

            if (B->parent != NullNode) {
                if (m_nodes[B->parent].left == iA) m_nodes[B->parent].left = iB;
                else m_nodes[B->parent].right = iB;
            } else {
                m_root = iB;
            }

            if (D->height > E->height) {
                B->right = iD;
                A->left = iE;
                E->parent = iA;
                A->aabb = AABB::Combine(C->aabb, E->aabb);
                B->aabb = AABB::Combine(A->aabb, D->aabb);
                A->height = 1 + std::max(C->height, E->height);
                B->height = 1 + std::max(A->height, D->height);
            } else {
                B->right = iE;
                A->left = iD;
                D->parent = iA;
                A->aabb = AABB::Combine(C->aabb, D->aabb);
                B->aabb = AABB::Combine(A->aabb, E->aabb);
                A->height = 1 + std::max(C->height, D->height);
                B->height = 1 + std::max(A->height, E->height);
            }
            return iB;
        }

        return iA;
    }

    PodVector<DynamicTreeNode> m_nodes;
    int32_t m_root;
    int32_t m_freeList;
    int32_t m_nodeCount;
    mutable PodVector<int32_t> m_stack;
};

} // namespace Velox
