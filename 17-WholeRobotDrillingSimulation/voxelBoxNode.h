//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Chris Sewell
    \author    Charity Lu
    \author    Francois Conti
    \version   3.2.0 $Rev: 1869 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef VOXEL_BOX_NODE_H
#define VOXEL_BOX_NODE_H
//------------------------------------------------------------------------------
// #include "collisions/CCollisionBasics.h"
// #include "collisions/CCollisionAABBBox.h"
// #include "collisions/CCollisionAABBTree.h"
#include "chai3d.h"
#include "voxelBox.h"

namespace chai3d{
//==============================================================================
/*!
    \file       voxelBoxTree.h

    \brief
    Implements an axis-aligned bounding box collision tree (AABB)
*/
//==============================================================================

//------------------------------------------------------------------------------
//! Internal AABB Node Types.
typedef enum
{
    BOX_NODE_INTERNAL,
    BOX_NODE_LEAF,
    BOX_NOT_DEFINED
} boxNodeType;

//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      voxelBoxNode
    \ingroup    collisions

    \brief
    This structure implements a tree node inside an AABB collision tree.
*/
//==============================================================================
struct voxelBoxNode
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of voxelBoxNode.
    //voxelBoxNode(int index);
    inline voxelBoxNode(int index = -1)
    {
        m_bbox.setEmpty();
        m_depth        = -1;
        m_nodeType     = BOX_NOT_DEFINED;
        m_leftSubTree  = -1;
        m_rightSubTree = -1;
        m_index = index; //m_index is critical if you want to render sth back to obstacles in the world, like changing its color when touched by the drill, which helps with debugging your algorithm
    }

    //! Destructor of voxelBoxNode.
    virtual ~voxelBoxNode() {}
    
    
    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    // //! This method creates a boundary box for a voxel (i.e. a sphere).
    // void fitBBox(double a_radius,
    //     cVector3d& a_center);

    // //! This method draws the edges of the boundary box for this node, if at the given depth.
    // void render(int a_depth = -1);


    // //! This method determines whether another voxelBox intersects any elements covered by this node.
    // bool computeCollision(const voxelBox& a_box);

    //==============================================================================
    /*!
          This method creates a boundary box to enclose a sphere belonging to the 
          leaf node.

          \param  a_radius   Radius of the sphere.
          \param  a_center   Center of the sphere.
    */
    //==============================================================================
    inline void fitBBox(double a_radius,
        cVector3d& a_center)
    {
        // empty box
        m_bbox.setEmpty();

        // enclose vertex
        // m_bbox.enclose(a_center);

        // retrieve boundary box min and max values
        // cVector3d min = m_bbox.m_min;
        // cVector3d max = m_bbox.m_max;

        cVector3d min = a_center;
        cVector3d max = a_center;
        // add radius envelope
        min.sub(a_radius, a_radius, a_radius);
        max.add(a_radius, a_radius, a_radius);

        // store new values
        m_bbox.setValue(min, max);
    }



    //==============================================================================
    /*!
        This method draws the edges of the boundary box for an internal tree node 
        if it is at depth a_depth in the tree, and calls the draw function for its 
        children.

        \param  a_depth  If a_depth > 0, then only draw nodes at this level in the tree.
                         If a_depth < 0 render all nodes up to this level.
    */
    //==============================================================================
    inline void render(cVector3d &min, cVector3d &max)//(int a_depth = -1)
    {
        m_bbox.render(min, max);
    // #ifdef C_USE_OPENGL
    //     if ( ((a_depth < 0) && (abs(a_depth) >= m_depth)) || (a_depth == m_depth))
    //     {
    //         m_bbox.render();
    //     }
    // #endif
    }

    //==============================================================================
    /*!
        This method draws the edges of the boundary box for an internal tree node 
        if it is at depth a_depth in the tree, and calls the draw function for its 
        children.

        \param  a_box    Input Box. 
    */
    //==============================================================================

    inline bool computeCollision(const voxelBox& a_box)
    {
        return m_bbox.intersect(a_box);
    }
    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! Bounding box for this node.
    voxelBox m_bbox;

    //! Depth of this node in the collision tree.
    int m_depth;

    //! Node type.
    boxNodeType m_nodeType;

    //! Index of this node.
    int m_index;

    //! Left child node index.
    int m_leftSubTree;

    //! Right child node index.
    int m_rightSubTree;
};


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

