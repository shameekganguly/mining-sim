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
    \version   3.2.0 $Rev: 2167 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef VOXEL_COLLISION_H
#define VOXEL_COLLISION_H
//------------------------------------------------------------------------------
// #include "math/CMaths.h"
// #include "collisions/CGenericCollision.h"
// #include "collisions/CCollisionAABBTree.h"

#include "chai3d.h"
#include "voxelBoxNode.h"
#include "drill.h"
//------------------------------------------------------------------------------
#include <vector>
#include <iostream>
//------------------------------------------------------------------------------
namespace chai3d {
//==============================================================================
/*!
    \file       CCollisionAABB.h

    \brief
    Implements an axis-aligned bounding box collision tree (AABB)
*/
//==============================================================================

//==============================================================================
/*!
    \class      cCollisionAABB
    \ingroup    collisions

    \brief
    This class implements an axis-aligned bounding box collision detector.

    \details
    This class implements an axis-aligned bounding box collision detection
    tree to efficiently detect for any collision between a line segment and 
    a collection of elements (point, segment, triangle) that compose an object.
*/
//==============================================================================
class voxelCollision : public cGenericCollision
{
    enum cCollisionAABBState
    {
        C_AABB_STATE_TEST_CURRENT_NODE,
        C_AABB_STATE_TEST_LEFT_NODE,
        C_AABB_STATE_TEST_RIGHT_NODE,
        C_AABB_STATE_POP_STACK
    };

    struct cCollisionAABBStack
    {
        int m_index;
        cCollisionAABBState m_state;
    };

    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of voxelCollision.
    inline voxelCollision()
    {
        // radius padding around elements
        //m_radiusAroundElements = 0.0;

        // // list of elements
        m_elements.clear();

        // number of elements
        m_numElements = 0;

        // clear nodes
        m_nodes.clear();

        // initialize variables
        m_rootIndex = -1;
        m_maxDepth = 0;
        m_radius = 0.0;
    }

    //! Destructor of voxelCollision.
    virtual ~voxelCollision()
    {
        // clear all nodes
        m_nodes.clear();
    }
    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:



    //! This method computes all collisions between a segment passed as argument and the attributed 3D object.
    // virtual bool computeCollision(cGenericObject* a_object,
    //                               cVector3d& a_segmentPointA,
    //                               cVector3d& a_segmentPointB,
    //                               cCollisionRecorder& a_recorder,
    //                               cCollisionSettings& a_settings);


    //     inline bool computeCollision(const voxelBox& a_box)
    // {
    //     return m_bbox.intersect(a_box);
    // }

    //==============================================================================
    /*!
        This method builds an axis-aligned bounding box collision-detection tree for
        a collection of elements passed as argument. \n\n

        Each leaf is associated with one element and with a boundary box of minimal
        dimensions such that it fully encloses the element and is aligned with
        the coordinate axes (no rotations).  Each internal node is associated
        with a boundary box of minimal dimensions such that it fully encloses
        the boundary boxes of its two children and is aligned with the axes.

        \param  a_elements  Pointer to element array.
        \param  a_radius    Bounding radius to add around each elements.
    */
    //==============================================================================
    inline void initialize(std::map<int, cShapeSphere*> & voxel, const int numOfVoxels)//(const cGenericArrayPtr a_elements, const double a_radius)
    {
        ////////////////////////////////////////////////////////////////////////////
        // INITIALIZATION
        ////////////////////////////////////////////////////////////////////////////


        // sanity check
        if (voxel.empty())
        {
            m_rootIndex = -1;
            return;
        }

        m_elements = voxel;

        // store radius
        m_radius = voxel[0]->getRadius();

        // clear previous tree
        m_nodes.clear();

        // get number of elements
        m_numElements = numOfVoxels;

        // init variables
        m_maxDepth = 0;

        // if zero elements, then exit
        if (m_numElements == 0)
        {
            m_rootIndex = -1;
            return;
        }


        ////////////////////////////////////////////////////////////////////////////
        // CREATE LEAF NODES
        ////////////////////////////////////////////////////////////////////////////


         // create leaf node for each element
        for (int i=0; i<m_numElements; ++i)
        {


            // if (m_elements[i]->getEnabled()){
            //     // get position of vertices
            //     cVector3d voxelCenter = m_elements[i]->getLocalPos();


            //     // create leaf node
            //     voxelBoxNode leaf;

            //     leaf.fitBBox(m_radius, voxelCenter);
            //     leaf.m_index = i;
            //     //leaf.m_leftSubTree = i;
            //     leaf.m_nodeType = BOX_NODE_LEAF;

            //     // add leaf to list
            //     m_nodes.push_back(leaf);
            // }

            // get position of vertices
            cVector3d voxelCenter = m_elements[i]->getLocalPos();


            // create leaf node
            voxelBoxNode leaf;

            leaf.fitBBox(m_radius, voxelCenter);
            leaf.m_index = i;
            //leaf.m_leftSubTree = i;
            leaf.m_nodeType = BOX_NODE_LEAF;

            // add leaf to list
            m_nodes.push_back(leaf);
        }


        ////////////////////////////////////////////////////////////////////////////
        // CREATE TREE
        ////////////////////////////////////////////////////////////////////////////
        int indexFirst = 0;
        int indexLast = m_numElements - 1;
        int depth = 0;

        if (m_numElements > 1)
        {
            m_rootIndex = buildTree(indexFirst, indexLast, depth);
        }
        else
        {
            m_rootIndex = 0;
        }
    }

    //==============================================================================
    /*!
        This methods updates the collision detector and should be called if the 
        3D model it represents is modified.
    */
    //==============================================================================
    virtual void update()
    {
        initialize(m_elements, m_radius);
    }


    //! This method renders a visual representation of the collision tree.
    virtual void render(const int depth, int *order,cVector3d & min, cVector3d & max, bool printInfo = false){
        if (depth > m_maxDepth){
            if (printInfo){
                std::cout << "The depth is larger than the max depth of the tree." << std::endl;
            }
            return;
        }

        int tmpIdx = m_rootIndex;
        int currDepth = 0;
        while (currDepth < depth){
            if (order[currDepth] == 0){
                tmpIdx = m_nodes[tmpIdx].m_leftSubTree;
            }else if (order[currDepth] == 1){//1
                tmpIdx = m_nodes[tmpIdx].m_rightSubTree;
            }
            currDepth++;
        }

        if (tmpIdx == -1){
            if (printInfo){
                std::cout << "No existing node is at this branch." << std::endl;   
            }
            return;
        }else{
            m_nodes[tmpIdx].render(min, max);
            if (printInfo){
                std::cout << "Center of the BBox: " << m_nodes[tmpIdx].m_bbox.getCenter() << std::endl; 
            }
            return;
        }

        // if (depth == 0){
            
        // }
        // int leftLargest = m_nodes[m_rootIndex].m_rightSubTree;
        // m_nodes[leftLargest].render(min, max);
    }


    virtual void render(const int idx, cVector3d & min, cVector3d & max){
        if (idx > m_nodes.size()){
            std::cout << "The index exceeds the size of the m_nodes." << std::endl;
            return;
        }
        if (idx < 0){
            std::cout << "The index must be larger than 0." << std::endl;
            return;
        }  
        m_nodes[idx].render(min, max);
        std::cout << "Center of the BBox: " << m_nodes[idx].m_bbox.getCenter() << std::endl;
        return;
    }



    //! This method computes all nodes that collides with the drill dr.
    //! If we have multiple drills, plug in this function with different dr.
    virtual int computeCollision(drill* dr, cVector3d & lastLinearVel, bool &stickyFlag){

        int count = 0;
        std::vector<int> collideIdx;
        int result = computeNodeCollision(m_rootIndex, dr, lastLinearVel, stickyFlag, count, collideIdx);
        //std::cout << count << std::endl << std::endl;
        // for (int i = 0; i < collideIdx.size(); i++){
        //     std::cout << collideIdx[i] << " ";
        // }
        // std::cout << std::endl << std::endl;

        return result;
        //return computeNodeCollision(m_rootIndex, dr, lastLinearVel, stickyFlag, count);


    }




    inline void printTree(){ // help with debugging
        int startIdx = m_rootIndex;
        printTreeRecursive(startIdx);
        std::cout << std::endl << std::endl;
    }


    inline void printTreeRecursive(int idx){
        if (m_nodes[idx].m_nodeType == BOX_NODE_LEAF){
            std::cout << idx << " ";
            return;
        }else{
            printTreeRecursive(m_nodes[idx].m_leftSubTree);
            std::cout << idx << " ";
            printTreeRecursive(m_nodes[idx].m_rightSubTree);

        }
    }

    //--------------------------------------------------------------------------
    // PROTECTED METHODS:
    //--------------------------------------------------------------------------

protected:

    //==============================================================================
    /*!
        Given a __start__ and __end__ index value of leaf nodes, this method creates
        a collision tree.

        \param  a_indexFirstNode  Lower index value of leaf node.
        \param  a_indexLastNode   Upper index value of leaf node
        \param  a_depth           Current depth of the tree. Root starts at 0.
    */
    //==============================================================================
    inline int buildTree(const int a_indexFirstNode, const int a_indexLastNode, const int a_depth)
    {
        // create new node
        voxelBoxNode node;

        // set depth of this node.
        node.m_depth = a_depth;
        node.m_nodeType = BOX_NODE_INTERNAL;

        // create a box to enclose all the leafs below this internal node


        node.m_bbox.setEmpty();
        for (int i=a_indexFirstNode; i<=a_indexLastNode; i++)
        {
            node.m_bbox.enclose(m_nodes[i].m_bbox);
        }

        // move leafs with smaller coordinates (on the longest axis) towards the
        // beginning of the array and leaves with larger coordinates towards the
        // end of the array
        int axis = node.m_bbox.getLongestAxis();
        int i = a_indexFirstNode;
        int mid = a_indexLastNode;

        double center = node.m_bbox.getCenter().get(axis);
        while (i < mid)
        {
            if (m_nodes[i].m_bbox.getCenter().get(axis) < center)
            {
                i++;
            }
            else
            {
                // swap nodes. For efficiency, we swap the minimum amount of information necessary.
                int t_index               = m_nodes[i].m_index;
                voxelBox t_bbox           = m_nodes[i].m_bbox;

                m_nodes[i].m_index        = m_nodes[mid].m_index;
                m_nodes[i].m_bbox         = m_nodes[mid].m_bbox;

                m_nodes[mid].m_index      = t_index;
                m_nodes[mid].m_bbox       = t_bbox;

                //cSwap(m_nodes[i], m_nodes[mid]);
                mid--;
            }
        }

        // increment depth for child nodes
        int depth = a_depth + 1;
        m_maxDepth = cMax(m_maxDepth, depth);

        // we expect mid, used as the right iterator in the "insertion sort" style
        // rearrangement above, to have moved roughly to the middle of the array;
        // however, if it never moved left or moved all the way left, set it to
        // the middle of the array so that neither the left nor right subtree will
        // be empty





        //!!!!!!!!!!!Huge breakthrough!!!!!!!!!!!
        // if ((mid == a_indexFirstNode) || (mid == a_indexLastNode))
        // {
        //     mid = (int)((a_indexLastNode + a_indexFirstNode) / 2);
        // }
        //!!!!!!!!!!!Huge breakthrough!!!!!!!!!!!
        mid = (int)((a_indexLastNode + a_indexFirstNode) / 2);



        // if there are only two nodes then assign both child nodes as leaves
        if ((a_indexLastNode - a_indexFirstNode) == 1)
        {
            // set left leaf
            node.m_leftSubTree = a_indexFirstNode;
            m_nodes[a_indexFirstNode].m_depth = depth;

            // set right leaf
            node.m_rightSubTree = a_indexLastNode;
            m_nodes[a_indexLastNode].m_depth = depth;
        }

        // there are more than 2 nodes
        else
        {




            // if the left subtree contains multiple elements, create new internal node
            if (mid > a_indexFirstNode)
            {
                node.m_leftSubTree = buildTree(a_indexFirstNode, mid, depth);
            }

            // if there is only one element in the right subtree, the right subtree
            // pointer should just point to the leaf node
            else
            {
                node.m_leftSubTree = a_indexFirstNode;
                m_nodes[a_indexFirstNode].m_depth = depth;
            }

            // if the right subtree contains multiple elements, create new internal node
            if ((mid+1) < a_indexLastNode)
            {
                node.m_rightSubTree = buildTree((mid+1), a_indexLastNode, depth);
            }

            // if there is only one element in the left subtree, the left subtree
            // pointer should just point to the leaf node
            else
            {
                node.m_rightSubTree = a_indexLastNode;
                m_nodes[a_indexLastNode].m_depth = depth;
            }
        }

        // insert node
        //node.m_index = (int)(m_nodes.size()-1);
        node.m_index = m_nodes.size();
        m_nodes.push_back(node);
        //return (int)(m_nodes.size()-1);
        return node.m_index;
    }











    //! This method computes node to bbox collision, but it doesn't care about if the node is disabled or not
    // we can set disabled type for the node. Thus don't have to search for all disabled ones
    


    //1->collision          No change to the tree

    //2->crush              Change to the tree: if not leaf and two subtrees are -1, return 2, 
    //                      disable the internal node. But not change it to leaf node. 
    //                      Keep it as internal node.
    //                      If the subtree returns 2, set that m_(left or right)SubTree as -1
    //                      The object is already disabled in drill function.

    //3->no collision       No change to the tree



    inline int computeNodeCollision(int currIdx, drill* dr, cVector3d & lastLinearVel, bool &stickyFlag, int & count, std::vector<int> & collideIdx){//this is only for debugging --> std::vector<int> & collisionIdx){
        count++;
        collideIdx.push_back(currIdx);
        //m_elements[m_nodes[3].m_index]->m_material->setYellowGold();
        if (m_nodes[currIdx].computeCollision(dr->getBBox())){//this node collides with the drillbox
            
            if (m_nodes[currIdx].m_nodeType == BOX_NODE_LEAF){

                //now we want to integrate the specific metric on the leaf node.
                // if the voxel is truly in collision and eventually crushed, disable the voxel
                //and don't have to look at this branch again
                
                //remember now the left->-1 and right->-1 never means it is a leaf node, 
                //maybe it is just a pruned internal node

                cShapeSphere* leafVoxel = m_elements[m_nodes[currIdx].m_index];
                int result = dr->isInCollision(leafVoxel, lastLinearVel, stickyFlag);
                if (result == 1){
                    m_elements[m_nodes[currIdx].m_index]->m_material->setGreenMediumAquamarine();//change color helps with debug
                    
                }else{
                    m_elements[m_nodes[currIdx].m_index]->m_material->setWhite();
                }
                return result;
            }
            else{
                int leftCollisionType, rightCollisionType;
                if (m_nodes[currIdx].m_leftSubTree != -1){
                    leftCollisionType =  computeNodeCollision(m_nodes[currIdx].m_leftSubTree, dr, lastLinearVel, stickyFlag, count, collideIdx);//, collisionIdx);
                    if (leftCollisionType == 2){
                        m_nodes[currIdx].m_leftSubTree = -1;//disable this branch
                    }
                }

                if (m_nodes[currIdx].m_rightSubTree != -1){
                    rightCollisionType =  computeNodeCollision(m_nodes[currIdx].m_rightSubTree, dr, lastLinearVel, stickyFlag, count, collideIdx);//, collisionIdx);
                    if (rightCollisionType == 2){
                        m_nodes[currIdx].m_rightSubTree = -1;//disable this branch
                    }
                }   

                if (m_nodes[currIdx].m_leftSubTree == -1 && m_nodes[currIdx].m_rightSubTree == -1){
                    //it is not a leaf node and all subtree indexes are -1
                    //so it is a internal node with all branches pruned

                    //it may be completely pruned just after the ifs above, or it has been completely pruned before
                    //this applies to both cases
                    return 2;
                }else{
                    // if we have any type that returns 1 (collision), then returns collision
                    // since not all of them are 2, (if all are 2 we would end up in the block above)
                    // return 3.
                    if (leftCollisionType == 1 || rightCollisionType == 1){
                        return 1;
                    }else{          
                        return 3;
                    }
                }
            }
        }
        return 3;//if no bbox collision in the first place, return 3 (no collision)

    }
    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------
//To test the code, make them public
//protected:
public:

    //! Collision shell radius around elements.
    double m_radius;

    //! Number of elements inside tree.
    int m_numElements;

    //! Pointer to the list of elements in the object.
    //cGenericArrayPtr m_elements;
    std::map<int, cShapeSphere*> m_elements;

    //! List of nodes.
    std::vector<voxelBoxNode> m_nodes;

    //! Index number of root node.
    int m_rootIndex;

    //! Maximum depth of tree.
    int m_maxDepth;
};
//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif //VOXEL_COLLISION_H
//------------------------------------------------------------------------------
