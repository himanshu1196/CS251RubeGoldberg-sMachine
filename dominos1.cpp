/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/*
 * Base code for CS 251 Software Systems Lab
 * Department of Computer Science and Engineering, IIT Bombay
 *
 */


#include "cs251_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
  #include <GLUT/glut.h>
#else
  #include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

namespace cs251
{  /**  
   *  A constructor function for the dominos_t class.
   *  Sets up the Box2D simulation.
   *  Creates 17 simulation objects
   *      -# Ground Object
   *      -# Horizontal Shelves
   *      -# Redirecting Curvy Ramp
   *      -# Top Pendulum that knocks 5 Dominos
   *      -# Sphere on Top Platform(Kick Off)   
   *      -# Series of 5 Dominos on Top Platform
   *      -# See Saw System
   *      -# The Pushing Box(Nothing much? Not Really!)
   *      -# Chain of Pendulums that move the Saw
   *      -# Saw System
   *      -# The Three Plank System
   *      -# Pulley System with on Open Box and two Loading Slabs
   *      -# Revolvable Horizontal Platform
   *      -# A Heavy Sphere
   *      -# See Saw Series System
   *      -# The Containers 
   *      -# The 2 Rotating Fans
   *      
   *      
   *  The chain of events as seen in when the code is run are as follows,
   *  The pendulum bob hits one of the dominos in series.
   *  The last one of the dominos hits the heavy balls.
   *  The heavy ball bounces over the see saw and make the light box go up, while it falls down into the left container
   *  The light box pushes the chain of pendulums which then pushes the saw.
   *  The saw then cuts the rope.
   *  The ball pushes the dominos and all the dominos on the 3 planks fall down one by one in series.
   *  The last revolving domino taps the sphere on the horizontal plank below.
   *  The sphere falls into the open box of the pulley i.e. pulls up the loading bars.
   *  The 4 spheres in the loaded bars fall down into the other(right) container.
   *  The 1st sphere which had fallen into the adjacent see-saw's open box now is thrown up.
   *  The sphere is made to roll down to the right to touch the switch to make the fan rotate. :)
   *  Now we can have some cool air! :P
   */
  dominos_t::dominos_t()
  {
     //Ground
    // \var ground
    /*! 1) Ground Object.
     * -# Defines a b2Body object pointer to the ground object as the reference point/level for other objects in the simulation.
     * -# Points to an edge-shaped object in the Box2D simulation.
     * -# Creates a b2EdgeShape object 'shape', a b2BodyDef object 'bd', required to create the Ground object.
     */
     b2Body* ground;
    { b2EdgeShape shape;
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
      b2BodyDef bd;
      ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
     } 
    
     //Top horizontal shelf
    // \var b1 (local)
    /*! 2) Horizontal Shelves.
     *  - 2.1) Top Horizontal shelf object.
     *   -# Creates a shelf modelled as a horizontal box.
     *   -# Uses b2PolygonShape 'shape' to define shape of the b2Body object 'b1' which represents the horizontal shelf.
     */
    
    {
      b2PolygonShape shape;
      shape.SetAsBox(3.1f, 0.25f);

      b2BodyDef bd;
      bd.position.Set(-31.0f, 38.0f);
       b2Body* b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
    }
    //horizontal shelf
    // \var b1 (local)
    /*! - 2.2) Another horizontal shelf object.
     *
     *    -# Creates a shelf modelled as a horizontal box.
     *    -# Uses b2PolygonShape 'shape' to define shape of the b2Body object 'b1' which represents the horizontal shelf.
     */
    {
      b2PolygonShape shape;
      shape.SetAsBox(13.f, 0.5f);

      b2BodyDef bd21;
      bd21.position.Set(-22.0f, 25.0f);
      b2Body* b1 = m_world->CreateBody(&bd21);
      b1->CreateFixture(&shape, 0.5f);

     // redirecting edges for the ball on the top
     /*! 
     *  3) Redirecting Curvy Ramp. 
     *  -# Creates a redirecting Curvy Ramp modelled as a set of edges in succession forming a curvy connected component.
     *  -# Uses b2EdgeShape to define 'shape(l/r)' of the b2Body object 'left/right' which represent the curvy slopes.
     *  -# Redifined shape(l/r) to reuse the variable for creating set of connected-edges.
     *  -# Redirecting Path : This element is right below the saw and has similar definition.
     */
   
      b2Body* left;
      b2Body* right;
      b2EdgeShape shapel, shaper;
        //shapel.Set(b2Vec2(-31.5f, 29.0f), b2Vec2(-33.0f, 32.0f));
      float x=-28.5, y=29.0, x2=-27.5;
      for(int i=0;i<3;i++)
      {
        shapel.Set(b2Vec2(x,y), b2Vec2(x-(3-i)*0.6, y+(1+i)*0.6));
        shaper.Set(b2Vec2(x2,y+4.f), b2Vec2(x2+(3-i)*0.6, y+4.f+(1+i)*0.6));
        x=x-(3-i)*0.6;
        x2=x2+(3-i)*0.6;
        y=y+(1+i)*0.6;
        b2BodyDef bd;
        left = m_world->CreateBody(&bd);
        right = m_world->CreateBody(&bd);
        left->CreateFixture(&shapel, 0.0f);
        right->CreateFixture(&shaper, 0.0f);
      }
    }

    //The pendulum that knocks the dominos off
    /*! 4) Top Pendulum that knocks the 5 Dominos.
     *  - 4.1) Pendulum-Stand-Base object.
     *   -# A b2Body pointer 'b2' to the pendulum-stand's base's object, in the simulation, modelled as a rectangular box.
     *   -# Defines b2PolygonShape 'shape' to define the box-shape of the base.
     *   -# 'b2' is defined as bodyA in the definition of revolute joint for pendulum.
     */
    {

      b2Body* b2;
      {
        b2PolygonShape shape;
        shape.SetAsBox(0.25f, 0.5f);

        b2BodyDef bd;
        bd.position.Set(-34.5f, 38.0f);
        b2 = m_world->CreateBody(&bd);
        b2->CreateFixture(&shape, 10.0f);
      }

    
    // \var b4(local) 
    /*! - 4.2) Pendulum Bob.
     *
     *   -# A b2Body pointer 'b4' to the bob object hanging from a point, to act as a supended pendulum bob in the simulation, modelled as a small dense square box.
     *   -# Defines b2PolygonShape shape to define the small-box-shape of the bob. 
     *   -# Creates a b2RevoluteJointDef object 'jd' to create a revolute joint between an anchor point and the bob(bodyB/'b4') 
     */
      b2Body* b4;
      {
        b2PolygonShape shape;
        shape.SetAsBox(0.25f, 0.25f);

        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.position.Set(-38.0f, 41.50f);
        b4 = m_world->CreateBody(&bd);
        b4->CreateFixture(&shape, 25.0f);
      }
      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-34.5f, 45.0f);
      jd.Initialize(b2, b4, anchor);

      m_world->CreateJoint(&jd);
    }

    //The sphere on the top platform
    // var: sbody (local)
    /*! 5) Sphere on Top Platform.
     *  -# Creates pointer 'sbody' to a heavy sphere objects created on the top horizontal plank/platform.
     *  -# Points to a (b2CircleShape)circle/sphere-shaped object in the Box2D simulation.
     *  -# Creates a b2CircleShape object 'circle', a b2BodyDef object 'ballbd', b2FixtureDef ballfd.
     */
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;

      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 15.0f;
      ballfd.friction = 1.0f;
      ballfd.restitution = 0.7f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(-28.f, 38.25f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
      sbody->SetId(3);
    }

    //Dominos on the top
    //  var: body (local)
    /*! 6) Series of 5 Dominos on Top Horizontal Shelf(Start the game :P).
     *   -# Creates 5 dominos on the above created horizontal shelf, modelled as thin boxes.
     *   -# b2FixtureDef object 'fd' used to specifying physical properties(density, friction, shape) of the b2Body 'body' object(for each domino).
     *   -# b2bodyDef object 'bd' used as body-definition for each b2Body 'body' representing a domino in the simulation.
     */
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 1.0f);

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;
      fd.friction = 0.1f;

      for (int i =0; i < 5; ++i)
      {
       b2BodyDef bd;
       bd.type = b2_dynamicBody;
       bd.position.Set(-33.5f + 1.0f * i, 40.f);
       b2Body* body = m_world->CreateBody(&bd);
       body->CreateFixture(&fd);
      }
    }

    //The see-saw system
    /*! 7) See Saw system.
     * - See saw is modelled as a plank joined with a triangular which acts as its fulcrum.
     * Uses b2PolygonShape to define shapes(rectangular/triangular) of the b2Body objects(plank/fulcrum).
     * Define a revolute joint between plank and th tip of wedge.
     */
    {
      //The triangle wedge
      // var: sbody (local)
     /*! - 7.1) See Saw system fulcrum.
      *
      *   -# Fulcrum is modelled as a triangle, simply a 3 vertex polygon.
      *   -# Uses b2PolygonShape to define 'poly' shape of the b2Body object 'sbody', wihich represents the fulcrum .
      *   -# 'wedgefd' is b2FixtureDef for 'sbody', defining its b2PolygonShape 'poly' and density, friction, resitution with (b2FixtureDef)wedgefd.
      */
      b2Body* sbody;
      b2PolygonShape poly;
      b2Vec2 vertices[3];
      vertices[0].Set(-1,0);
      vertices[1].Set(1,0);
      vertices[2].Set(0,1);
      poly.Set(vertices, 3);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(-20.0f, 25.5f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);
      //The plank on top of the wedge
       //  var: body (local)
      /*!  - 7.2) The Plank.
       *
       *     -# Creates a plank modelled as a flattened (horizontal) box.
       *     -# Defines b2PolygonShape 'shape' for the flattened shape and b2FixtureDef 'fd2' to define the physical properties (density,shape) of the b2Body object 'body', which represents the plank.
       *     -# Defines b2RevoluteJointDef to define a revolute joint between the plank, and the tip of the wedge(fulcrum), on which it rests, at its center. 
       */   
      b2PolygonShape shape;
      shape.SetAsBox(8.0f, 0.2f);
      b2BodyDef bd2;
      bd2.position.Set(-20.0f, 26.5f);
      bd2.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd2);
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 1.f;
      fd2->shape = new b2PolygonShape;
      fd2->shape = &shape;
      body->CreateFixture(fd2);

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-20.0f, 26.5f);
      jd.Initialize(sbody, body, anchor);
      m_world->CreateJoint(&jd);
      //The box on the right side of the see-saw
      // var: b3 (local)
     /*! 
      * 8) The Pushing  Box.
      *  -# Creates a light box as a rectangular object in the simulation used to push the pendulum.
      *  -# Uses b2PolygonShape object 'shape2' to define shape of the b2Body object 'b3' which represents the light pushing box.
      *  -# 'fd3' is a b2FixtureDef onject defining the physical properities of 'b3'. Density of the box is 0.1(small)
      */
      b2PolygonShape shape2;
      shape2.SetAsBox(2.0f, 2.0f);
      b2BodyDef bd3;
      bd3.position.Set(-11.f, 28.5f);
      bd3.type = b2_dynamicBody;
      b2Body *b3 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = .1f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape2;
      b3->CreateFixture(fd3);
    }

    //The chain of pendulums that push the saw
    {
      // var: b2 (local)
      /*! 
       * 9) Chain of Pendulums that move the Saw.
       * - 9.1) Pendulum-Hinge-Base object.
       *   -# A b2Body pointer 'b2' to the Pendulum-Hinge-Base object on which the pendulum hangs in the simulation, modelled as a wide rectangular box.
       *   -# Defined as bodyA in the definition of revolute joint for pendulum.
       *   -# Creates a b2PolygonShape object 'shape', a b2BodyDef object 'bd', required to create the Pendulum-Hinge-Base object.
       */
      b2Body* b2;
      {
            b2PolygonShape shape;
            shape.SetAsBox(6,0.5f);
            b2BodyDef bd;
            bd.position.Set(7,40);
            b2 = m_world->CreateBody(&bd);
            b2->CreateFixture(&shape, 0.01f);
      }
      // \var b4 
      /*! - 9.2) Pendulum Bob objects.
       *
       *   -# b2Body pointer 'b4' to the pendulum bob object hanging from a point to act, in the simulation, for each pendulem, one by one.
       *   -# Defines b2PolygonShape shape to define the small-box-shape of the bob. 
       *   -# Creates a b2RevoluteJointDef object 'jd' to create a revolute joint between an anchor point and the bob(bodyB/'b4') 
       */
      for(int i=0; i<4; i++)
      {
          b2Body* b4;
         {
            b2CircleShape shape;
            shape.m_radius=2.f;

            b2BodyDef bd;
            bd.position.Set(1.f+i*4, 34.0f);
            bd.type=b2_dynamicBody;
            b4= m_world->CreateBody(&bd);
            b4->CreateFixture(&shape,0.01f);
          }
          b2RevoluteJointDef jd;
          b2Vec2 anchor;
          anchor.Set(1.f+i*4,40);
          jd.Initialize(b2,b4,anchor);

          m_world->CreateJoint(&jd);
      }
    }
      //The saw system(top right)
      // var: sbody (local)
      /*! 10) Saw system.
       *    -# Saw, b2Body 'body', is modelled as a triangle(3 vertices to define 'shape') hung on a small b2Body 'b1'.
       *    -# Uses b2PolygonShape 'poly' to define shape of the b2Body object 'body'.
       *    -# 'fd' is b2FixtureDef for 'body' definig its shape('poly'), density, friction, resitution.
       *    -# Creates a b2MotorJointDef object 'mjd' to create a motor joint, basically to create the effect of a saw.
       */
    {
      b2BodyDef bd;
      bd.position.Set(16.5f,34.0f);
      b2Body* b1=m_world->CreateBody(&bd);
      b2PolygonShape shape;
      shape.SetAsBox(.01f,.01f);
      b2FixtureDef fd;
      fd.shape=&shape;
      fd.density=1.f;
      fd.restitution=0.f;        
      b1->CreateFixture(&fd);

      bd.type=b2_dynamicBody;
      bd.position.Set(15.f,34.0f);
      b2Body* body=m_world->CreateBody(&bd);
      b2Vec2 v[3];
      v[0].Set(0,0.5f);
      v[1].Set(0.,-1.3f);
      v[2].Set(5,.5f);  
      shape.Set(v,3);
      fd.shape=&shape;
      fd.density=.01f;
      fd.friction=1.f;
      body->CreateFixture(&fd); 
  
      b2MotorJointDef mjd;
      mjd.Initialize(b1,body);    
      mjd.maxForce=1.f;
      mjd.maxTorque=1000.f;
      m_world->CreateJoint(&mjd);
    } 
     
   
   // redirecting path
     /* 
     *    - 10.1) Redirecting Curvy Ramp2. 
     *        -# Creates a redirecting Curvy Ramp modelled as a set of edges in succession forming a curvy connected component.
     *        -# Uses b2EdgeShape to define 'shapel' of the b2Body object 'left' which represent the curvy slopes.
     *        -# Redifined 'shapel' to reuse the variable for creating set of connected-edges.
     */
   
      b2Body* left;
      b2EdgeShape shapel, shaper;
      float x=24, y=24;
      for(int i=0;i<3;i++)
      {
        shapel.Set(b2Vec2(x,y), b2Vec2(x-(3-i)*0.8, y+(1+i)*0.6));
        x=x-(3-i)*0.8;
        y=y+(1+i)*0.6;
        b2BodyDef bd;
        left = m_world->CreateBody(&bd);
        left->CreateFixture(&shapel, 0.0f);
      }
    
    

    //Top 3 horizontal planks
    // \var plankbase[3]
    /*! 
     * 11) The Three Plank System.
     */
    /*!  - 11.1) The 3 Base-Planks.
     *    -# Creates 3 b2Body object pointers plankbase[0,1,2] representing 3 shelves modelled as flattened horizontal boxes.
     *    -# Uses b2PolygonShape 'shape' to define shape of the b2Body objects 'plankbase[0,1,2]'.
     *    -# The 'shape' object is changed as plankbase[0](1st plank) is smaller than the rest and placed at a shifted location. Basically, we set 'shape' and bd.position for every plankbase[i] in the loop.
     */ 
    b2Body* plankbase[3];
    {
      b2PolygonShape shape;
      shape.SetAsBox(3.5f, 0.25f);

      b2BodyDef bd;
      bd.position.Set(-31.0f+60.1f, 20.0f+3.0f);
      
      for(int i=0;i<3;i++)
      { plankbase[i] = m_world->CreateBody(&bd);
        plankbase[i]->CreateFixture(&shape, 0.5f);
        bd.position.Set(-31.0f+58.0f, 17.0f+3.0f-3.0f*i);
        shape.SetAsBox(5.6f, 0.25f);
      }  
    }
  
   {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 1.0f);

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;
      fd.friction = 0.7f;
        
      b2Body* body;
      b2BodyDef bd;
      bd.type = b2_dynamicBody;
      
      b2RevoluteJointDef jointDef;
      int lower=4, higher=10;          //jugaad  
     // Dominos series on the three planks 
     // var: body (local)
     /*!  - 11.2) Series of Dominos on 3 Horizontal Shelves.
      *
      *    -# Creates 7,12,12 dominos on the above created 3 horizontal shelves(i:lower-higher in for loop decides location and number of the dominos).
      *    -# b2FixtureDef object 'fd' used to specifying physical properties(density, friction, shape) of the b2Body object 'body'.
      *    -# b2bodyDef object 'bd' used as body-definition for each b2Body 'body' representing a domino in the simulation.
      *    -# Creates b2RevoluteJointDef object 'jointDef' to create revolute joint for 3 dominos, hinged at their bases with the plank-base.
      *    -# Since, the location of these dominos and corresponding joints is asymmetric we broke up for the 3 cases(3 planks) using "switch-case". 
      */
        
      for(int k=0;k<3;k++) 
      { for (int i = lower; i <=higher; ++i)    
         { bd.position.Set(-35.5f+58.0f + 1.0f * i, 21.25f+3.0f-k*3.0f);
           body = m_world->CreateBody(&bd);
           body->CreateFixture(&fd);
         }
        // use the last created domino
        
         switch(k)
         { case 0 :   jointDef.localAnchorA.Set(3.5,0.25);
                      jointDef.localAnchorB.Set(0.1,-1.0);
                      lower=0; higher=10;
                      break;
           case 1 :   bd.position.Set(-36.5f+58.0f, 22.25f);
                      body = m_world->CreateBody(&bd);
                      body->CreateFixture(&fd);
                      lower=-1; higher=10;
                      jointDef.localAnchorA.Set(-5.6,0.25);
                      jointDef.localAnchorB.Set(-0.1,-1.0);
                      break;      
           case 2 :   jointDef.localAnchorA.Set(5.6,0.25);
                      jointDef.localAnchorB.Set(0.1,-1.0);
         }
         jointDef.bodyA = plankbase[k];
         jointDef.bodyB = body;
         jointDef.collideConnected = true;
       
         m_world->CreateJoint(&jointDef);
       }  
    }

    //The pulley system (bottom right corner)
    // 
    /*! 12) Pulley system with an Open Box and 2 Loading Slabs. 
     *  -# Consists of 2 bars/loading slabs(b2Body object pointers box1[0,1]), an open box(b2Body object pointer 'box2'), few balls(b2Body object pointer 'spherebody') and a pulley joint connecting the two in the simulation.
     *  -# The 2 bars/loading slabs('box1[0,1]') modelled as 2 rectangular boxes hinged from one end, with a wall on the other side modelled as another verticle box. Formed using b2PolygonShape object 'bs1'(redefined over and over) & correspondingb2FixtureDef object 'fd1' for 2 base bars and walls. Also, b2Body pointer 'body1' is defines as the anchor for erevolute joint. The density of lower slab is kept on higher side for simulation purposes.
     *  -# The open box('box2') is modelled as set of 3 connected slabs(1 horizontal and 2 verticle), formed by using b2PolygonShape object 'bs1'(redefined over and over) & correspondingb2FixtureDef object 'fd1' redefined for -left, right and base walls, kept horizontal with fixedRotation set to true. These are created using 'fd1' to specify shape, friction, restitution, density etc and b2PolygonShape object 'bs1' to specify shape(horizontal rectangular boxes and verticle edges).
     *  -# Pulley joint is created using b2PulleyJoitDef object pointer 'myjoint', used to define the pulley string across anchor points(worldAnchorGround(1/2)), connecting 2 bodies -'box1[0]' and 'box2'.
     *  -# Revolute joint is created for the 2 loading slabs using b2RevoluteJoint object rd, for both the bars. 'body1' is created as the dummy(no fixture) base body for the anchor. The anchor points etc are set before the for loop and in the for loop(at end) for the next iteration.
     */
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(30,11);
      bd->fixedRotation = true;
      //The open box
      
      b2Body* box2 = m_world->CreateBody(bd);
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 500.0;
      fd1->friction = 5;
      fd1->restitution = 0.3f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      
      b2EdgeShape shape;
      shape.Set(b2Vec2(1.5,-1.5), b2Vec2(1.5,1.5));
      fd1->shape = &shape;
      box2->CreateFixture(fd1);
 
      shape.Set(b2Vec2(-1.5,-1.5), b2Vec2(-1.5,1.5));
      fd1->shape = &shape;
      box2->CreateFixture(fd1);

      bs1.SetAsBox(1.5,0.1,b2Vec2(0,-1.5),0);
      fd1->shape = &bs1;
      box2->CreateFixture(fd1);
      
      //The first and second bar on left
      bd->fixedRotation = false;
      bd->position.Set(23,13.5f);
      fd1->shape = &bs1;
      fd1->density = 500.0;
      fd1->friction = 0.01f;

       b2BodyDef bd1;
       bd1.position.Set(18,13.5);
       

      //(revolute joint for both the bars)
      b2RevoluteJointDef rjd;
      b2Vec2 anchor;
      anchor.Set(18,13.5);
      
      b2Body *box1[2];
      for(int i=0;i<2;i++)
      { bs1.SetAsBox(0.2,0.4+i*0.8,b2Vec2(0,i*1.2),0);
        box1[i] = m_world->CreateBody(bd);
        box1[i]->CreateFixture(fd1);
        fd1->density=1.f;
        bs1.SetAsBox(3,0.2, b2Vec2(-3.f,0),0);
        box1[i]->CreateFixture(fd1);
      
        b2Body* body1 = m_world->CreateBody(&bd1);
       
        rjd.Initialize(box1[i], body1, anchor);
        m_world->CreateJoint(&rjd);
      
        //Setting up for next iteration
        anchor.Set(18,10.5);
        bd->position.Set(23,10.5);
        bd1.position.Set(18,10.5);
        fd1->friction=10.f;
      }

      //Revolute joint between the two left bars
      b2RevoluteJointDef rd;
      rd.Initialize(box1[1],box1[0],b2Vec2(23,13));
      m_world->CreateJoint(&rd);

      //The train of spheres
      for(int j=0;j<2;j++)
      {
        b2Body* spherebody;
        b2CircleShape circle;
        circle.m_radius=0.8;
        b2FixtureDef ballfd;
        ballfd.density = 50.0f;
        ballfd.friction = 10.f;
        ballfd.restitution = 0.f;
        ballfd.shape=&circle;
        for (int i = 0; i <2; ++i)
        {
          b2BodyDef ballbd;
          ballbd.type = b2_dynamicBody;
          ballbd.position.Set(18.f + i*2.0+2*j, 13.7f-j*3);
          spherebody = m_world->CreateBody(&ballbd);
          spherebody->CreateFixture(&ballfd);
        }
      }
      
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody2(23, 11); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody1(30, 13); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround2(30, 15); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround1(23, 15); // Anchor point for ground 2 in world axis
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(box1[0], box2, worldAnchorGround1, worldAnchorGround2, box1[0]->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);
    }
    
    //The Revolvable horizontal platform
    //   var: body (local)
    /*! 13) Revolvable Horizontal Platform.
     *  -# Creates a revolvable shelf modelled as a horizontal box with joint at center.
     *  -# Uses b2PolygonShape objects to define shape of the b2Body objects body, body2 which represent the revolvable horizontal shelf.
     *  -# Revolute joint is created using b2RevoluteJointDef object specifying localanchorA,B and bodyA,B.
     */
    {
      b2PolygonShape shape;
      shape.SetAsBox(1.f, 0.2f);

      b2BodyDef bd;
      bd.position.Set(32.f, 14.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      fd->friction=0.f;
      body->CreateFixture(fd);
    
      bd.type=b2_staticBody;
      b2Body* b2=m_world->CreateBody(&bd);
    
      b2RevoluteJointDef rjd;
      rjd.bodyA=body;
      rjd.bodyB=b2;
      rjd.localAnchorA.Set(0,0);
      rjd.localAnchorB.Set(0,0);
      m_world->CreateJoint(&rjd);

    }
    //The heavy sphere on the platform
    // var: sbody (local)
    /*! 14) A Heavy Sphere.
     *  -# Creates b2Body pointer 'sbody' to a heavy sphere objects created on the above horizontal revolvable shelf.
     *  -# Points to a Circle/Sphere-shaped object in the Box2D simulation.
     *  -# Creates (1) b2CircleShape object 'circle' (2) b2BodyDef object 'ballbd', to set position and type(b2_dynamicBody) (3) b2FixtureDef 'ballfd', to specify physical properties - density(high-25), friction, shape('circle'), restitution(0.5-so that the ball doesn't bounce).
     */
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 100.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = .5f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(32.0f, 15.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }
    //The see-saw system
    /*! 15) See Saw Series system.
     *  - See saw is modelled as a plank joined with a triangule which acts as its fulcrum.
     *  - Uses b2PolygonShape to define shapes(rectangular/triangular) of the b2Body objects(plank/fulcrum).
     *  Define a revolute joint between plank and th tip of wedge.
     */
    {
      //The triangle wedges
      // var: sbody (local)
      /*!  - 15.1) See Saw system fulcrum.
       *
       *    -# Fulcrum is modelled as a triangle, simply a 3 vertex polygon.
       *    -# Uses b2PolygonShape to define 'poly' shape of the b2Body object 'sbody', wihich represents the fulcrum .
       *    -# 'wedgefd' is b2FixtureDef for 'sbody', defining its b2PolygonShape 'poly' and density, friction, resitution with (b2FixtureDef)wedgefd.
       */
      {b2Body* sbody;
       b2PolygonShape poly;
       b2Vec2 vertices[3];
       vertices[0].Set(-1,0);
       vertices[1].Set(1,0);
       vertices[2].Set(0,1.5);
       poly.Set(vertices, 3);
       b2FixtureDef wedgefd;
       wedgefd.shape = &poly;
       wedgefd.density = 10.0f;
       wedgefd.friction = 0.0f;
       wedgefd.restitution = 0.0f;
       b2BodyDef wedgebd;
       wedgebd.position.Set(8.0f, 0.0f);
       sbody = m_world->CreateBody(&wedgebd);
       sbody->CreateFixture(&wedgefd);
       //The plank on top of the wedge
       //  var: body (local)
       /*!  - 15.2) The Plank
        *    -# Creates a plank modelled as a flattened (horizontal) box.
        *    -# Defines b2PolygonShape 'shape' for the flattened shape and b2FixtureDef 'fd2' to define the physical properties (density,shape) of the b2Body object 'body', which represents the plank.
        *    -# Defines b2RevoluteJointDef to define a revolute joint between the plank, and the tip of the wedge(fulcrum), on which it rests, at its center. 
        */  
       b2PolygonShape shape;
       shape.SetAsBox(10.0f, 0.2f);
       b2BodyDef bd2;
       bd2.position.Set(8.0f, 1.5f);
       bd2.type = b2_dynamicBody;
       b2Body* body = m_world->CreateBody(&bd2);
       b2FixtureDef *fd2 = new b2FixtureDef;
       fd2->density = 1.f;
       fd2->shape = new b2PolygonShape;
       fd2->shape = &shape;
       //fd2->friction=0.0f;
       body->CreateFixture(fd2);

       b2RevoluteJointDef jd;
       b2Vec2 anchor;
       anchor.Set(8.0f, 1.5f);
       jd.Initialize(sbody, body, anchor);
       m_world->CreateJoint(&jd);
     } 
      //The triangle wedge
      // var: sbody (local)
      /*  15.1)fulcrum. (PART OF ABOVE ELEMENT)
       *    -# Fulcrum is modelled as a triangle.
       *    -# Uses b2PolygonShape to define 'poly' shape of the b2Body object 'sbody'(fulcrum).
       *    -# 'wedgefd' is b2FixtureDef for 'sbody' definig its shape('poly'), density, friction, resitution.
       */
      b2Body* sbody;
      b2PolygonShape poly;
      b2Vec2 vertices[3];
      vertices[0].Set(-1,0);
      vertices[1].Set(1,0);
      vertices[2].Set(0,2.);
      poly.Set(vertices, 3);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(-8.0f, 0.0f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);

      //The plank on top of the wedge(PART OF ABOVE ELEMENT)
      //  var: body (local)
      /* - 15.2) Plank.
       *   -# Creates a plank modelled as a horizontal box.
       *   -# Uses b2PolygonShape to define 'shape' and b2FixtureDef 'fd2' which defines the physical properties(density,shape) of the b2Body object 'body' which represents the plank.
       *   -# The Plank rests on the above created wedge(fulcrum) and its center is fixed on the tip of the wedge(fulcrum). 
       *   -# Also the planks friction is defined to be 0.05f.
       */
     {
      b2PolygonShape shape;
      shape.SetAsBox(8.0f, 0.2f);
      b2BodyDef bd2;
      bd2.position.Set(-8.0f, 2.f);
      bd2.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd2);
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 1.f;
      fd2->shape = new b2PolygonShape;
      fd2->shape = &shape;
      fd2->friction=0.5f;
      body->CreateFixture(fd2);

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-8.0f, 2.f);
      jd.Initialize(sbody, body, anchor);
      m_world->CreateJoint(&jd);
     }  
      //The Containers
      /*!  16)The Containers
        *  -# The open box('box1') is formed by using 3 b2PolygonShape & b2FixtureDef-left, right and base walls.
        *  -# Defined a b2FixtureDef object 'fd1' to specify shape, friction, restitution, density etc.
        *  -# Defined a b2PolygonShape object 'bs1' to specify shape(horizontal and vertical rectangular boxes).
        */
      {
       b2BodyDef *bd = new b2BodyDef;
       bd->type = b2_dynamicBody;
       bd->position.Set(-1.5,5);
        //bd->fixedRotation = true;
     
       b2Body* box2 = m_world->CreateBody(bd);
       b2FixtureDef *fd1 = new b2FixtureDef;
       fd1->density = 10.0;
       fd1->friction = 0.5;
       fd1->restitution = 0.f;
       fd1->shape = new b2PolygonShape;
       b2PolygonShape bs1;
       for(int i=0;i<2;i++)
       {  bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
          fd1->shape = &bs1;
          box2->CreateFixture(fd1);
      
          bs1.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
          fd1->shape = &bs1;
          box2->CreateFixture(fd1);
      
          bs1.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
          fd1->shape = &bs1;
          box2->CreateFixture(fd1);

          fd1->density = 25.f;  // changin for next iteration
          fd1->friction = 5;
          bd->position.Set(15.f,5);
          box2 = m_world->CreateBody(bd);
        }  
      }
    }

    // Car
    {
      b2PolygonShape chassis;
      b2Vec2 vertices[8];
      vertices[0].Set(-2.5f, -0.7f);
      vertices[1].Set(2.5f, -0.7f);
      vertices[2].Set(2.5f, 0.0f);
      vertices[4].Set(2.0f, 1.9f);
      vertices[3].Set(0.f, 1.9f);
      vertices[5].Set(-2.5f, 0.2f);
      chassis.Set(vertices, 6);

      b2CircleShape circle;
      circle.m_radius = 1.f;

      b2BodyDef bd;
      bd.type = b2_dynamicBody;
      bd.position.Set(-22.f, 1.0f);
      b2Body* m_car = m_world->CreateBody(&bd);
      m_car->CreateFixture(&chassis, 0.001f);
      m_car->SetId(3);
      b2FixtureDef fd;
      fd.shape = &circle;
      fd.density = .001f;
      fd.friction = 0.9f;

      bd.position.Set(-23.f, 0.35f);
      b2Body*  m_wheel1 = m_world->CreateBody(&bd);
      m_wheel1->CreateFixture(&fd);
      m_wheel1->SetId(3);
      bd.position.Set(-21.f, 0.4f);
      b2Body*  m_wheel2 = m_world->CreateBody(&bd);
      fd.density=0.005f;
      m_wheel2->CreateFixture(&fd);
      m_wheel2->SetId(3);
      b2WheelJointDef jd;
      b2Vec2 axis(0.0f, 1.0f);

      jd.Initialize(m_car, m_wheel1, m_wheel1->GetPosition(), axis);
      jd.motorSpeed = 0.0f;
      jd.maxMotorTorque = 20.0f;
      jd.enableMotor = true;
      jd.frequencyHz = 4;
      jd.dampingRatio = 0.7f;
      m_world->CreateJoint(&jd);

      jd.Initialize(m_car, m_wheel2, m_wheel2->GetPosition(), axis);
      jd.motorSpeed = 0.0f;
      jd.maxMotorTorque = 10.0f;
      jd.enableMotor = false;
      jd.frequencyHz = 4;
      jd.dampingRatio = 0.7f;
      m_world->CreateJoint(&jd);
    }
     // Ramp
     /* 
     *    - 10.1) Curvy Ramp2. 
     *        -# Creates a redirecting Curvy Ramp modelled as a set of edges in succession forming a curvy connected component.
     *        -# Uses b2EdgeShape to define 'shapel' of the b2Body object 'left' which represent the curvy slopes.
     *        -# Redifined 'shapel' to reuse the variable for creating set of connected-edges.
     */
     {
      b2Body* left;
      b2EdgeShape shapel, shaper;
      float x=-23, y=0;
      for(int i=0;i<3;i++)
      {
        shapel.Set(b2Vec2(x,y), b2Vec2(x-(4-i)*0.8, y+(1+i)*0.4));
        x=x-(4-i)*0.8;
        y=y+(1+i)*0.4;
        b2BodyDef bd;
        left = m_world->CreateBody(&bd);
        left->CreateFixture(&shapel, 0.0f);
      }
      /*for(int i=0;i<4;i++)
      {
        shapel.Set(b2Vec2(x,y), b2Vec2(x+(i+1)*0.6, y+(4-i)*0.6));
        x=x+(i+1)*0.6;
        y=y+(4-i)*0.6;
        b2BodyDef bd;
        left = m_world->CreateBody(&bd);
        left->CreateFixture(&shapel, 0.0f);
      }*/
     }
    //Bottom button box
    // \var b1 (local)
    /*   Button Base.
     *   -# Creates a box modelled as a horizontal square polygon.
     *   -# Uses b2PolygonShape 'shape' to define shape of the b2Body object 'b1' which represents the box.
     */
    {  
    b2PolygonShape shape;
    shape.SetAsBox(2.f, 2.f);
    b2BodyDef bd;
    bd.position.Set(-42.8f, 2.0f);
    b2Body* b1 = m_world->CreateBody(&bd);
    b1->CreateFixture(&shape, 0.5f);
    }
}

//  ***************************************************************************
  //! A member taking one argument.
  /*!
   *  Destroys m_mouseJoint( a b2MouseJoint object ) when the mouse is released   
   *  \param p a b2Vec2 object.
   *  \sa mouse_down(), mouse_move()
  */
  void base_sim_t::mouse_up(const b2Vec2& p)
  {
      if (m_mouseJoint)
      { m_world->DestroyJoint(m_mouseJoint);
        m_mouseJoint = NULL;
      }

  }
  //! A member taking one argument.
  /*!
   *  Sets m_mouseJoint( a b2MouseJoint object ) when the mouse moves, to the new position of the mouse pointer, specified by 'p'- a vector.   
   *  \param p a b2Vec2 object.
   *  \sa mouse_down(), mouse_up()
  */
  void base_sim_t::mouse_move(const b2Vec2& p)
  { m_mouseWorld = p;
    if (m_mouseJoint)
    { m_mouseJoint->SetTarget(p);
    }
  }

  //! A member taking one argument.
  /*!
   *  Create a m_mouseJoint( a b2MouseJoint object ) when the mouse is pressed for the first time, to set the position of the mouse to current position of the mouse pointer, specified by 'p'- a vector.   
   *  Defines QueryCallback object 'callback' used to get 'body' by querying over the b2World object 'm_world' to get the body overlapping with a small square(b2AABB aabb) at 'p'.   
   *  Creates new b2MouseJoint with bodyA as reference m_ground_body, bodyB as 'body' and target as'p'.
   *  \param p a b2Vec2 object.
   *  \sa mouse_up(), mouse_move()
  */
  void base_sim_t::mouse_down ( const b2Vec2& p)
  { m_mouseWorld = p;
    if (m_mouseJoint != NULL)
    {
      return;
    }
    // Make a small box.
    b2AABB aabb;
    b2Vec2 d;
    d.Set(0.001f, 0.001f);
    aabb.lowerBound = p - d;
    aabb.upperBound = p + d;

    // Query the world for overlapping shapes.
    QueryCallback callback(p);
    m_world->QueryAABB(&callback, aabb);

    if (callback.m_fixture)
    { b2Body* body = callback.m_fixture->GetBody();
      b2MouseJointDef md;
      md.bodyA = m_ground_body;
      md.bodyB = body;
      md.target = p;
      md.maxForce = 1000.0f * body->GetMass();
      m_mouseJoint = (b2MouseJoint*)m_world->CreateJoint(&md);
      body->SetAwake(true);
   }

  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}

