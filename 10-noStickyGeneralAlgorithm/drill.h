//------------------------------------------------------------------------------
#ifndef DRILL_H
#define DRILL_H
//------------------------------------------------------------------------------
// #include "collisions/CCollisionBasics.h"
// #include "collisions/CCollisionAABBBox.h"
// #include "collisions/CCollisionAABBTree.h"
#include "chai3d.h"
#include "voxelBoxNode.h"
#include <vector>
#define PI 3.14159265
namespace chai3d{
//==============================================================================
/*!
    \file       drill.h

    \brief
    Construct a drill class.
*/
//==============================================================================

// //------------------------------------------------------------------------------
// //! Internal AABB Node Types.
// typedef enum
// {
//     BOX_NODE_INTERNAL,
//     BOX_NODE_LEAF,
//     BOX_NOT_DEFINED
// } boxNodeType;

// //------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      drill
    \ingroup    collisions

    \brief
    This structure implements a drill with two spheres and one cylinder.
*/
//==============================================================================

class drill
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of voxelBoxNode.
    //voxelBoxNode(int index);


    //inline drill(cShapeSphere* cursor, cShapeSphere* cursorS1, cShapeSphere* cursorS2)
    inline drill(cShapeCylinder* cursor, cShapeSphere* cursorS1, cShapeSphere* cursorS2, cMultiMesh* drillObj, cWorld* world)
    {
        m_radius = cursorS1->getRadius();
        m_height = cursor->getHeight();

        // cShapeCylinder* cursor = new cShapeCylinder(Rc, Rc, Hc);
        // cShapeSphere* cursorS1 = new cShapeSphere(Rc);
        // cShapeSphere* cursorS2 = new cShapeSphere(Rc);

        m_spheres.clear();
        m_spheres.push_back(cursorS1);
        m_spheres.push_back(cursorS2);

        m_sphereNodes.clear();
        voxelBoxNode n1;
        voxelBoxNode n2;
        m_sphereNodes.push_back(n1);
        m_sphereNodes.push_back(n2);

        m_cylinder = cursor;

        m_drillObj = drillObj;

        addChildToWorld(world);//this is a custom member function 

        //at the beginning the positions are not determined, 
        //so no specific parameters need to be set for m_node
    }

    virtual ~drill()
    {
        // for (int i = 0; i < m_spheres.size(); i++){
        //     delete m_spheres[i];
        // }
        // // delete m_spheres[0];
        // // delete m_spheres[1];
        // delete m_cylinder;
    }

    
    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:


    // This method sets local positions for the drill 
    // and update the bounding boxes of the spheres and the drill.
    inline void setLocalPos(cVector3d & P1, cVector3d & P2, cVector3d & PObj, double epsForBBox = 0.1){


        //epsForBBox is for changing the color of the voxel that hasn't be drilled back to white
        //Make the bbox slightly larger




        //the cyliner will be set as the same position as the sphere 1
        m_cylinder->setLocalPos(P1);
        m_spheres[0]->setLocalPos(P1);
        m_spheres[1]->setLocalPos(P2);
        m_drillObj->setLocalPos(PObj);


        m_sphereNodes[0].fitBBox(m_radius * (1+epsForBBox), P1);
        m_sphereNodes[1].fitBBox(m_radius * (1+epsForBBox), P2);

        m_node.m_bbox.enclose(m_sphereNodes[0].m_bbox, m_sphereNodes[1].m_bbox);
    }

    // This method sets local rotations for the drill 
    // Here drillObjAngle means the relative angle between drill obj file and the drill capsule
    inline void setLocalRot(cMatrix3d & rot, cMatrix3d & drillObjAngle){
        m_cylinder->setLocalRot(rot);
        cMatrix3d drillObjRealAngle = rot * drillObjAngle;
        //m_drillObj->setLocalRot(drillObjRealAngle);
        m_drillObj->setLocalRot(drillObjRealAngle);
        //std::cout << drillObjAngle.str(3) << std::endl;
    }

    // This method returns the local positions of the two spheres.
    inline std::vector<cVector3d> getLocalPos(){
        
        std::vector<cVector3d> localPos;
        localPos.push_back(m_spheres[0]->getLocalPos());
        localPos.push_back(m_spheres[1]->getLocalPos());

        return localPos;

    }

    // This method add childs of all the objects to world
    inline void addChildToWorld(cWorld* world){
        world->addChild(m_cylinder);
        world->addChild(m_spheres[0]);
        world->addChild(m_spheres[1]); 
        world->addChild(m_drillObj);
    }

    // This method returns the bounding box of the drill
    inline voxelBox getBBox(){
        return m_node.m_bbox;
    }

    inline void setColor(int button){
        switch(button)
        {
            case 0:
                m_cylinder->m_material->setGreenMediumAquamarine();
                m_spheres[0]->m_material->setGreenMediumAquamarine();
                m_spheres[1]->m_material->setGreenMediumAquamarine();
                m_drillObj->m_material->setGreenMediumAquamarine();
                break;
            case 1:
                m_cylinder->m_material->setYellowGold();
                m_spheres[0]->m_material->setYellowGold();
                m_spheres[1]->m_material->setYellowGold();
                m_drillObj->m_material->setYellowGold();               
                break;
            case 2:
                m_cylinder->m_material->setOrangeCoral();
                m_spheres[0]->m_material->setOrangeCoral();
                m_spheres[1]->m_material->setOrangeCoral(); 
                m_drillObj->m_material->setOrangeCoral();
                break;
            case 3:
                m_cylinder->m_material->setPurpleLavender();
                m_spheres[0]->m_material->setPurpleLavender();
                m_spheres[1]->m_material->setPurpleLavender();
                m_drillObj->m_material->setPurpleLavender();
                break;
            default:
                m_cylinder->m_material->setBlueRoyal();
                m_spheres[0]->m_material->setBlueRoyal();
                m_spheres[1]->m_material->setBlueRoyal();
                m_drillObj->m_material->setBlueRoyal();
        }

    }

    inline cVector3d axisDirection(){
        std::vector<cVector3d> localPos = getLocalPos();
        cVector3d unitL = localPos[1] - localPos[0];
        unitL = unitL / unitL.length();
        return unitL;
    }

    inline void setCapsuleShowEnabled(bool flag){
        m_cylinder->setShowEnabled(flag);
        m_spheres[0]->setShowEnabled(flag);
        m_spheres[1]->setShowEnabled(flag);
    }

    inline void setObjShowEnabled(bool flag){
        m_drillObj->setShowEnabled(flag);
    }



    //This method computes the contact force between voxels and the drill
    inline cVector3d contactForce(cVector3d & F_proxy, double proxy_b_no_contact, 
        double proxy_b_contact_dir, double proxy_b_contact_lat){

        cVector3d unitL = axisDirection();
        cVector3d F_proxy_dir, F_proxy_lat, F_contact_dir, F_contact_lat, F_contact;

        F_proxy_dir = unitL.dot(F_proxy) * unitL; //proxy force component along drilling direction 
        F_proxy_lat = F_proxy - F_proxy_dir; //proxy force component along drilling direction   

        F_contact_dir = -(proxy_b_contact_dir - proxy_b_no_contact) / proxy_b_contact_dir * F_proxy_dir;
        F_contact_lat = -(proxy_b_contact_lat - proxy_b_no_contact) / proxy_b_contact_lat * F_proxy_lat;

        F_contact = F_contact_dir + F_contact_lat;

        return F_contact;
    }

    //This method computes the vibration force between voxels and the drill
    inline cVector3d vibrationForce(bool vibrationSwitch, double curr_time){
        cVector3d F_vibration;
        if (vibrationSwitch){

            cVector3d unitL = axisDirection();
            double vibrationFreq = 50; // Hz
            double vibrationMag = 0.001; //0.001
            double deltaT = curr_time - (int)(vibrationFreq * curr_time) / vibrationFreq;
            double vibrationPhase = 2 * PI * deltaT * vibrationFreq;

            F_vibration.x(vibrationMag * cos(vibrationPhase));

            cVector3d latDirection;
            cMatrix3d NormalRotation(1, 0, 0,
                                     0, 0, 1,
                                     0,-1, 0);//rotate -90 degree

            latDirection = NormalRotation * unitL;

            double phaseDifference = PI / 2.0;
            F_vibration = F_vibration + (vibrationMag * cos(vibrationPhase + phaseDifference)) * latDirection;

        }else{
            F_vibration.zero();
        }
        return F_vibration;








        
    }


    //==============================================================================
    //To be done.

    // // This method evaluate the collision through a specific metric rather than simple bbox collision test
    // // voxel contains all voxel objects
    // // possibleLeafIdx contains the idx of objects that may be in collision with the drill
    // inline void drillCollision(std::map<int, cShapeSphere*> & voxel, std::vector<int> possibleLeafIdx){

    // }


    //one node collision test using a specific metric for evaluation
    //1->collision
    //2->crush
    //3->no collision
    //4->already eliminated
    
    //idx: the index of the voxel (obstacleS)

    inline int isInCollision(cShapeSphere* voxel, cVector3d & lastLinearVel, bool & stickyFlag){

    //inline int isInCollision(int idx, cVector3d P1, cVector3d P2, cVector3d yc, cVector3d unitL, double x1c, double x2c, bool & stickyFlag){

        std::vector<cVector3d>  localPos = getLocalPos();
        cVector3d P1 = localPos[0];
        cVector3d P2 = localPos[1];
        cVector3d Pm = (P1 + P2) / 2.0;         // position of mass center (also the cylinder center)
        cVector3d unitL = axisDirection();              // unit vector from P1 to P2
        double x1c = P1.dot(unitL);             // projection of P1 along direction of unit vector
        double x2c = P2.dot(unitL);             // projection of P2 along direction of unit vector
        cVector3d yc = P1 - x1c * unitL;        // projection of center axis along the direction perpendicular to axis direction



        cVector3d Px = voxel->getLocalPos();
        double xx = Px.dot(unitL);
        cVector3d yx = Px - xx * unitL;
        //for collision-generate-all-force case, we only need to know if the drill is in collision with rock.



        //some params used as global params in original main.cpp
        double Rs = voxel->getRadius();
        double Rc = m_radius;
        //double epsForElimination = 0.1 * Rs;
        double epsForElimination = -0.5 * Rs;

        if (lastLinearVel.length()>1e-5 && unitL.dot(lastLinearVel)/lastLinearVel.length() > 0.999){ // angle is about 5.13 degree
            stickyFlag = true;
            // assume not in collision, because if the velocity is within 5 degree with the backward direction,
            //it means that you want to retrieve the drill, not drilling laterally.
        }






        if (xx <= x1c){

            double dist1x = P1.distance(Px);
            if (Rc - Rs - epsForElimination < dist1x && dist1x < Rc + Rs){// near P1 collison and the obstacle hasn't been broken
                if ((Px - P1).dot(lastLinearVel) > 0){
                    stickyFlag = false;
                }
                return 1;

            }else if(dist1x <= Rc - Rs - epsForElimination){// the ball is inside the capsule and should be crushed
                voxel->setEnabled(false);//disable the object, as it is broken by collision
                return 2;//already crushed the ball, no longer in collision

            }else{// no collision with the boundary layer, may be because it is at the start of the simulation, or the capsule is move upward in relaxed position
                return 3;
            }


        }else if(xx > x1c && xx < x2c){

            double distcx = yc.distance(yx);
            if (Rc - Rs - epsForElimination < distcx && distcx < Rc + Rs){// between P1 nad P2 collision and the obstacle hasn't been broken
                cVector3d PL;
                PL = P1 + unitL.dot(Px - P1) * unitL; // projection point of Px on center axis
                if ((Px - PL).dot(lastLinearVel) > 0){
                    stickyFlag = false;
                }

                return 1;

            }else if(distcx <= Rc - Rs - epsForElimination){
                voxel->setEnabled(false);//disable the object, as it is broken by collision
                return 2;//already crushed the ball, no longer in collision

            }else{
                return 3;

            }


        }else{//xx >= x2c near P2 collision

            double dist2x = P2.distance(Px);
            if (Rc - Rs - epsForElimination< dist2x && dist2x < Rc + Rs){// near P2 collison and the obstacle hasn't been broken
                if ((Px - P2).dot(lastLinearVel) > 0){
                    stickyFlag = false;
                }
                return 1;

            }else if(dist2x <= Rc - Rs - epsForElimination){
                voxel->setEnabled(false);//disable the object, as it is broken by collision
                return 2;//already crushed the ball, no longer in collision
            }else{
                return 3;
            }

        }













    }

    //==============================================================================
    


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------



public:

    //! sphere S1 and S2's radius.
    double m_radius;

    //! height of the cylinder.
    double m_height;

    //! node of the whole drill.
    voxelBoxNode m_node;
    //! The (only two nodes for two spheres will be saved as m_leftSubTree and m_rightSubTree

    //! cShapeSpheres for two drill heads
    std::vector<cShapeSphere*> m_spheres;

    //! cShapeCylinder for the middle part of the drill
    cShapeCylinder* m_cylinder;

    //! nodes of the two spheres, just used to help update the drill bbox
    std::vector<voxelBoxNode> m_sphereNodes;

    cMultiMesh* m_drillObj;

};


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

