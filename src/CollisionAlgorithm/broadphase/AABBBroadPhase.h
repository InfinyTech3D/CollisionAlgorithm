#pragma once

#include <sofa/helper/AdvancedTimer.h>
#include <CollisionAlgorithm/BaseAABBBroadPhase.h>
#include <CollisionAlgorithm/BaseAlgorithm.h>
#include <CollisionAlgorithm/BaseElement.h>

namespace sofa::collisionalgorithm {

/**
 * @brief A class for broad-phase collision detection using Axis-Aligned Bounding Boxes (AABB).
 *
 * The `AABBBroadPhase` class implements a broad-phase collision detection algorithm that
 * organizes elements in a 3D spatial grid. The grid is used to index elements, and each
 * element is associated with a specific location in the grid, facilitating efficient collision checks.
 * The class is part of the broader collision detection system and is used to quickly narrow down
 * potential collisions before more detailed, computationally expensive checks are performed.
 */
class AABBBroadPhase : public BaseAABBBroadPhase {
public:

    /**
     * @brief The macro used to register the class in the Sofa framework.
     *
     * This macro registers the `AABBBroadPhase` class as a subclass of `BaseAABBBroadPhase`.
     * It is required by the Sofa framework for managing and interacting with objects.
     */
    SOFA_CLASS(AABBBroadPhase, BaseAABBBroadPhase);

    /**
     * @brief Constructor for the AABBBroadPhase class.
     *
     * Initializes the `AABBBroadPhase` object. Currently, the constructor does not perform
     * any specific initialization beyond calling the base class constructor.
     */
    AABBBroadPhase() {}

    /**
     * @brief Updates the offset values used for indexing the elements in the grid.
     *
     * This method calculates and updates the offset values that are used to generate the keys
     * for the elements in the 3D grid. The offsets are based on the number of boxes along each
     * dimension (nbox[1] and nbox[2]).
     */
    void updateData() override {
        m_offset[0] = m_nbox[1] * m_nbox[2];
        m_offset[1] = m_nbox[2];
    }

    /**
     * @brief Retrieves the set of elements at a given 3D grid position.
     *
     * This method returns a set of elements that are located at the grid position (i, j, k).
     * It performs a lookup in the indexed element map and returns the corresponding set of elements.
     * If no elements are found, an empty set is returned.
     *
     * @param i The x-coordinate of the grid.
     * @param j The y-coordinate of the grid.
     * @param k The z-coordinate of the grid.
     * @return A set of elements at the given grid position.
     */
    const std::set<BaseElement::SPtr>& getElementSet(unsigned i, unsigned j, unsigned k) const override {
        auto it = m_indexedElement.find(getKey(i, j, k));
        if (it == m_indexedElement.end()) {
            static std::set<BaseElement::SPtr> empty;
            return empty;
        } else {
            return it->second;
        }
    }

    /**
     * @brief Clears the indexed element container.
     *
     * This method clears the map of indexed elements, effectively removing all elements from the grid.
     */
    void newContainer() override {
        m_indexedElement.clear();
    }

    /**
     * @brief Adds a new element to the grid at the specified position.
     *
     * This method calculates the key corresponding to the grid position (i, j, k) and inserts the
     * given element into the map at that key. The offsets for the grid are recalculated before
     * inserting the element.
     *
     * @param i The x-coordinate of the grid.
     * @param j The y-coordinate of the grid.
     * @param k The z-coordinate of the grid.
     * @param elmt A smart pointer to the element to be added.
     */
    void addElement(int i, int j, int k, BaseElement::SPtr elmt) override {
        m_offset[0] = m_nbox[1] * m_nbox[2];
        m_offset[1] = m_nbox[2];

        unsigned key_i = i * m_offset[0];
        unsigned key_j = j * m_offset[1];
        unsigned key = key_i + key_j + k;

        m_indexedElement[key].insert(elmt);
    }

    /**
     * @brief Generates a unique key for the grid position (i, j, k).
     *
     * This method calculates a unique key for the element located at the grid position (i, j, k)
     * using the current offsets. The key is used to efficiently look up elements in the grid.
     *
     * @param i The x-coordinate of the grid.
     * @param j The y-coordinate of the grid.
     * @param k The z-coordinate of the grid.
     * @return A unique key representing the grid position.
     */
    inline Index getKey(size_t i, size_t j, size_t k) const {
        return i * m_offset[0] + j * m_offset[1] + k;
    }

    /**
     * @brief Retrieves the x-coordinate of the grid from the key.
     *
     * This method calculates and returns the x-coordinate (i) from the given key.
     *
     * @param key The key representing the grid position.
     * @return The x-coordinate (i) corresponding to the key.
     */
    inline unsigned getIKey(unsigned key) {
        return key / m_offset[0];
    }

    /**
     * @brief Retrieves the y-coordinate of the grid from the key.
     *
     * This method calculates and returns the y-coordinate (j) from the given key.
     *
     * @param key The key representing the grid position.
     * @return The y-coordinate (j) corresponding to the key.
     */
    inline unsigned getJKey(unsigned key) {
        return (key - getIKey(key) * m_offset[0]) / m_offset[1];
    }

    /**
     * @brief Retrieves the z-coordinate of the grid from the key.
     *
     * This method calculates and returns the z-coordinate (k) from the given key.
     *
     * @param key The key representing the grid position.
     * @return The z-coordinate (k) corresponding to the key.
     */
    inline unsigned getKKey(unsigned key) {
        return key - getIKey(key) * m_offset[0] - getJKey(key) * m_offset[1];
    }

protected:
    /**
     * @brief The map storing elements indexed by their grid position key.
     *
     * This map associates each key (representing a grid position) with a set of elements located
     * at that position. It is used to efficiently retrieve elements during collision checks.
     */
    std::map<unsigned, std::set<BaseElement::SPtr>> m_indexedElement;

    /**
     * @brief The offset values used for indexing the grid.
     *
     * The `m_offset` array contains the offset values for the grid in each dimension. These values
     * are used to calculate unique keys for grid positions.
     */
    type::Vec<2, size_t> m_offset;
};

}  // namespace sofa::collisionalgorithm
