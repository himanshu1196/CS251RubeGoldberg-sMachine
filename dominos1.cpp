
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
   *  Creates 10 simulation objects
   *      -# ground
   *      -# horizontal shelves
   *      -# a curvilinear edge
   *      -# pendulum that knocks the dominos
   *      -# top sphere   
   *      -# a see saw with a light box
   *      -# a box
   *      -# chain of pendulums 
   *      -# series of dominos
   *      -# a saw 
   *      -# series of dominos on 3 planks
   *      -# pulley system
   *      -# revolvable horizontal platform
   *      -# a heavy sphere 
   *      -# a see saw 
   *      -# a see saw with open box 
   *
   *  The chain of events as seen in when the code is run are as follows,
   *  The pendulum bob hits one of the dominos in series.
   *  The last one of the dominos hits the heavy balls.
   *  The heavy ball bounces over the see saw an d make the light box go up
   *  The light box pushes the chain of pendulums which then pushes the saw.
   *  The saw then pushes the dominos and all the dominos on thhe 3 planks fall down one by one in series.
   *  The last revolving domino taps the sphere on the horizontal plank below.
   *  The sphere falls into the open box of the pulley which then pushes the see-saw down.
   *  The 1st sphere which had fallen into the adjacent see-saw's open box now is thrown up.
   *  The sphere is made to roll down to the right to touch the switch to make the fan rotate.
   */ 
  dominos_t::dominos_t()
  {
     //Ground
    // \var ground
    /*! 1) Creates pointer 'ground' to Ground object.
     * -# A pointer to the ground object as the reference point/level for other objects in the simulation.
     * -# Points to an edge-shaped object in the Box2D simulation.
     * -# Creates a b2EdgeShape object 'shape', a b2BodyDef object 'bd', required to create the Ground object.
     */
     b2Body* ground;
    {

      b2EdgeShape shape;
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
      b2BodyDef bd;

      ground = m_world->CreateBody(&bd);

      ground->CreateFixture(&shape, 0.0f);

    }
    
        //Top horizontal shelf
    // \var b1 (local)
    /*! 2-1) Top Horizontal shelf object.
     *  -# Creates a shelf modelled as a horizontal box.
     *  -# Uses b2PolygonShape to define 'shape' of the b2Body object 'ground' which represents the horizontal shelf.
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
    /*! 2-2) Horizontal shelf object.
     *  -# Creates a shelf modelled as a horizontal box.
     *  -# Uses b2PolygonShape to define 'shape' of the b2Body object 'ground' which represents the horizontal shelf.
     */
    {
      b2PolygonShape shape;
      shape.SetAsBox(13.f, 0.5f);

      b2BodyDef bd21;
      bd21.position.Set(-22.0f, 25.0f);
      b2Body* b1 = m_world->CreateBody(&bd21);
      b1->CreateFixture(&shape, 0.0f);

	//    redirecting edges for the ball on the top
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
      	//shaper.Set(b2Vec2(-25.0f, 37.0f), b2Vec2(-28.0f, 34.0f));
    }
    //The sphere on the top platform
     // var: sbody (local)
    /*! 3) A Heavy Sphere.
     *  -# Creates pointer 'sbody' to a heavy sphere objects created on the above horizontal revolvable shelf.
     *  -# Points to a Circle/Sphere-shaped object in the Box2D simulation.
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
    }

    //Dominos on the top
     //  var: body (local)
    /*! 4) Series of 5 Dominos on horizontal shelf.
     *  -# Creates 5 dominos on the above created horizontal shelf.
     *  -# b2FixtureDef object 'fd' used to specifying physical properties(density, friction, shape) of the object.
     *  -# b2bodyDef object 'bd' used as body-definition for each b2Body 'body' representing a domino in the simulation.
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
    // var: ground (local)
    /*! 5) See Saw system.
     *  - See saw is modelled as a plank joined with a triangle which acts as its fulcrum.
     *  - Uses b2PolygonShape to define shapes(rectangular/triangular) of the b2Body objects(plank/fulcrum).
     */
    {
      //The triangle wedge
       // var: sbody (local)
    /*! - 5-1) See Saw system fulcrum.
     *
     *    -# Fulcrum is modelled as a triangle.
     *    -# Uses b2PolygonShape to define 'poly' shape of the b2Body object 'sbody'(fulcrum).
     *    -# 'wedgefd' is b2FixtureDef for 'sbody' definig its shape('poly'), density, friction, resitution.
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
      /*! - 5-2) Plank.
       *   -# Creates a plank modelled as a horizontal box.
       *   -# Uses b2PolygonShape to define 'shape' and b2FixtureDef 'fd2' which defines the physical properties
       *   (density,shape) of the b2Body object 'body' which represents the plank.
       *   -# The Plank rests on the above created wedge(fulcrum) and its center is fixed on the tip of the wedge(fulcrum). 
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
      // var: body3 (local)
    /*! 6) A Box.
     *  -# Creates a light box as arectangular object in the simulation.
     *  -# Uses b2PolygonShape object 'shape2' to define shape of the b2Body object 'body3' which represents the light box.
     *  -# 'fd3' is a b2FixtureDef onject defining the physical properities of 'body3'. Density of the box is 17(not small)
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

      //The pendulum that knocks the dominos off
    //The pendulum that knocks the dominos off
    // var: b2 (local)
    /*! 7) Pendulum that knocks the dominos 
    * - 7-1) Pointer to Pendulum-Stand-Base object.
     *  
     *   -# A pointer to the pendulum stand's base object on which the pendulum stands in the simulation.
     *   -# Points to a Box-shaped object in the Box2D simulation.
     *   -# Defined as bodyA in the definition of revolute joint for pendulum.
     *   -# Creates a b2PolygonShape object 'shape', a b2BodyDef object 'bd', required to create the Pendulum Base object.
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

    
          // \var b4 
    /*! 
     * 
     * - 7-2) Pointer to Pendulum Bob object.
     *   -# A b2Body pointer 'b4' to the pendulum bob object hanging from a point to act as a pendulum in the simulation.
     *   -# Points to a Box-shaped object in the Box2D simulation.
     *   -# Defined as bodyB in the definition of revolute joint.
     *   -# Creates a b2PolygonShape object 'shape', a b2BodyDef object 'bd', required to create the Pendulum Bob object.
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
      /*
    //The train of small spheres
    {
      b2Body* spherebody;
      b2CircleShape circle;
      circle.m_radius = 0.5;
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 1.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      for (int i = 0; i < 10; ++i)
      {
    	b2BodyDef ballbd;
    	ballbd.type = b2_dynamicBody;
    	ballbd.position.Set(-22.2f + i*1.0, 26.6f);
    	spherebody = m_world->CreateBody(&ballbd);
    	spherebody->CreateFixture(&ballfd);
      }
    }*/

    //The chain of pendulums that push the saw
    {
       // var: b2 (local)
    /*! 4) Pendulums that move the saw
     * - 8-1) Pointer to Pendulum-Hinge-Base object.
     *  
     *   -# A pointer to the pendulum stand's base object on which the pendulum stands in the simulation.
     *   -# Points to a Box-shaped object in the Box2D simulation.
     *   -# Defined as bodyA in the definition of revolute joint for pendulum.
     *   -# Creates a b2PolygonShape object 'shape', a b2BodyDef object 'bd', required to create the Pendulum Base object.
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
    /*! 
     *  - 8.2) Pointer to Pendulum Bob objects.
     *   -# A b2Body pointer 'b4' to the pendulum bob object hanging from a point to act as a pendulum in the simulation.
     *   -# Points to a Box-shaped object in the Box2D simulation.
     *   -# Defined as bodyB in the definition of revolute joint.
     *   -# Creates a b2PolygonShape object 'shape', a b2BodyDef object 'bd', required to create the Pendulum Bob object.
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
     //The triangle wedge
    // var: sbody (local)
    /*! - 9) Saw system.
     *
     *    -# Saw is modelled as a triangle.
     *    -# Uses b2PolygonShape to define 'poly' shape of the b2Body object 'sbody'(fulcrum).
     *    -# 'fd' is b2FixtureDef for 'sbody' definig its shape('poly'), density, friction, resitution.
     *    -# Also a motor joint is used to create the effect of a saw.
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
	   //mjd.motorSpeed=1.0f;
	   //mjd.enableMotor=true;
	    m_world->CreateJoint(&mjd);
    }

//Top 3 horizontal planks
/*! 10) Horizontal plank objects.
     *  -# Creates a shelf modelled as a horizontal box.
     *  -# Uses b2PolygonShape to define 'shape' of the b2Body object 'plankbase(1/2/3)' which represents the horizontal shelf.
     */ 
	b2Body* plankbase;
    b2Body* plankbase2;
    b2Body* plankbase3;
    {
      b2PolygonShape shape;
      shape.SetAsBox(5.6f, 0.25f);
	b2PolygonShape shape2;
      shape2.SetAsBox(3.5f, 0.25f);

      b2BodyDef bd;
      bd.position.Set(-31.0f+54.1f, 30.0f+3.0f);
      plankbase = m_world->CreateBody(&bd);
      plankbase->CreateFixture(&shape2, 0.0f);
      bd.position.Set(-31.0f+52.0f, 27.00f+3.0f);
      plankbase2 = m_world->CreateBody(&bd);
      plankbase2->CreateFixture(&shape, 0.0f);
      bd.position.Set(-31.0f+52.0f, 24.00f+3.0f);
      plankbase3 = m_world->CreateBody(&bd);
      plankbase3->CreateFixture(&shape, 0.0f);
    }
  
        //Dominos series on the three planks 
      //  var: body (local)
    /*! 11) Series of 12 Dominos on each horizontal shelf.
     *  -# Creates 12 dominos on the above created horizontal shelves.
     *  -# b2FixtureDef object 'fd' used to specifying physical properties(density, friction, shape) of the object.
     *  -# b2bodyDef object 'bd' used as body-definition for each b2Body 'body' representing a domino in the simulation.
     *  -# Also creates 3 dominos with a revolute joint hinged at their bases.
     */
   {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 1.0f);

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;
      fd.friction = 0.1f;
      for (int i = 4; i < 10; ++i)
	{
	  b2BodyDef bd;
	  bd.type = b2_dynamicBody;
	  bd.position.Set(-35.5f+52.0f + 1.0f * i, 31.25f+3.0f);
	  b2Body* body = m_world->CreateBody(&bd);
	  body->CreateFixture(&fd);
	}
      b2BodyDef bd;
	  bd.type = b2_dynamicBody;
	  bd.position.Set(-35.5f+52.0f + 1.0f * 10, 31.25f+3.0f);
	  b2Body* body = m_world->CreateBody(&bd);
	  body->CreateFixture(&fd);
      b2RevoluteJointDef jointDef;
      jointDef.bodyA = plankbase;
      jointDef.bodyB = body;
      jointDef.localAnchorA.Set(3.5,0.25);
      jointDef.localAnchorB.Set(0.1,-1.0);
      jointDef.collideConnected = true;
      m_world->CreateJoint(&jointDef);
	//  bd.position.Set(-36.5f+52.0f, 31.25f+3.0f);
	//  fd.friction= 1.0f;
	  b2Body* body2 ;//= m_world->CreateBody(&bd);
	//  body2->CreateFixture(&fd);
      //fd.friction = 0.1f;
	        for (int i = 0; i < 10; ++i)
	{
	  //b2BodyDef bd;
	  bd.type = b2_dynamicBody;
	  bd.position.Set(-35.5f +52.0f+ 1.0f * i, 28.25f+3.0f);
	  body = m_world->CreateBody(&bd);
	  body->CreateFixture(&fd);
	}
      //b2BodyDef bd;
      fd.friction=1.0f;
	  bd.type = b2_dynamicBody;
	  bd.position.Set(-35.5f+52.0f + 1.0f * 10, 28.25f+3.0f);
	  body = m_world->CreateBody(&bd);
	  body->CreateFixture(&fd);
	  bd.position.Set(-36.5f+52.0f, 28.25f+3.0f);
	  fd.friction= 0.1f;
	  body2 = m_world->CreateBody(&bd);
	  body2->CreateFixture(&fd);
      //b2RevoluteJointDef jointDef;
      jointDef.bodyA = plankbase2;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(-5.6,0.25);
      jointDef.localAnchorB.Set(-0.1,-1.0);
      jointDef.collideConnected = true;
      m_world->CreateJoint(&jointDef);
	  fd.friction=0.1f;
	        for (int i = 0; i < 10; ++i)
	{
	  //b2BodyDef bd;
	  bd.type = b2_dynamicBody;
	  bd.position.Set(-35.5f +52.0f+ 1.0f * i, 25.25f+3.0f);
	  body = m_world->CreateBody(&bd);
	  body->CreateFixture(&fd);
	}
      //b2BodyDef bd;
	  bd.type = b2_dynamicBody;
	  bd.position.Set(-35.5f +52.0f+ 1.0f * 10, 25.25f+3.0f);
	  body = m_world->CreateBody(&bd);
	  body->CreateFixture(&fd);
      //b2RevoluteJointDef jointDef;
      jointDef.bodyA = plankbase3;
      jointDef.bodyB = body;
      jointDef.localAnchorA.Set(5.6,0.25);
      jointDef.localAnchorB.Set(0.1,-1.0);
      jointDef.collideConnected = true;
      m_world->CreateJoint(&jointDef);
	  bd.position.Set(-36.5f+52.0f, 25.25f+3.0f);
	  fd.friction= 1.0f;
	  body2 = m_world->CreateBody(&bd);
	  body2->CreateFixture(&fd);
    }

    //The pulley system (bottom right corner)
    // 
    /*! 12) Pulley system with an open box. 
     *  -# Consists of a bar, an open box and a pulley joint connecting the two in the simulation.
     *  -# The bar('box2') is a simple Box-shaped b2Body, kept horizontal.
     *  -# The open box('box1') is formed by using 3 b2PolygonShape & b2FixtureDef.
     *  -# Pulley joint is created using b2PulleyJoitDef object used to define the associated objects and create the joint.
     */
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(18,5);
      bd->fixedRotation = true;
      //The open box
      
	  b2Body* box2 = m_world->CreateBody(bd);
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
      fd1->shape = &bs1;
      box2->CreateFixture(fd1);
      
      bs1.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
      fd1->shape = &bs1;
      box2->CreateFixture(fd1);
      
      bs1.SetAsBox(0.2,2,b2Vec2(-2.0f,0.f), 0);
      fd1->shape = &bs1;
	  box2->CreateFixture(fd1);
      
      
      
      //The bar
      bd->position.Set(30,5);
      bs1.SetAsBox(2,0.2);
      fd1->shape = &bs1;
	  fd1->density = 34.0;
      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
		
      // The pulley joint
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(30, 5); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody2(18, 5); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(30, 10); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(18, 10); // Anchor point for ground 2 in world axis
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);
    }
    
    //The revolving horizontal platform
    //   var: body (local)
    /*! 13) Revolvable Horizontal Pulley
     *  -# Creates a revolvable shelf modelled as a horizontal box with joint at center.
     *  -# Uses b2PolygonShape objects to define shape of the b2Body objects body, body2 which represent the revolvable horizontal shelf.
     *  -# Revolute joint is created using b2RevoluteJointDef object specifying localanchorA,B and bodyA,B.
     */
    {
      b2PolygonShape shape;
      shape.SetAsBox(2.4f, 0.2f);

      b2BodyDef bd;
      bd.position.Set(24.8f, 24.0f);
      //bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);
    }

    //The heavy sphere on the platform
    // var: sbody (local)
    /*! 14) A Heavy Sphere.
     *  -# Creates pointer 'sbody' to a heavy sphere objects created on the above horizontal revolvable shelf.
     *  -# Points to a Circle/Sphere-shaped object in the Box2D simulation.
     *  -# Creates a b2CircleShape object 'circle', a b2BodyDef object 'ballbd', b2FixtureDef ballfd.
     */
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 25.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = .5f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(26.0f, 25.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }


    //The see-saw system at the bottom
    /*! 15) See Saw system.
     *  - See saw is modelled as a plank joined with a triangle which acts as its fulcrum.
     *  - Uses b2PolygonShape to define shapes(rectangular/triangular) of the b2Body objects(plank/fulcrum).
     */
    {
      //The triangle wedge
      // var: sbody (local)
    /*! - 15.1) See Saw system fulcrum.
     *
     *    -# Fulcrum is modelled as a triangle.
     *    -# Uses b2PolygonShape to define 'poly' shape of the b2Body object 'sbody'(fulcrum).
     *    -# 'wedgefd' is b2FixtureDef for 'sbody' definig its shape('poly'), density, friction, resitution.
     */
      b2Body* sbody;
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
      /*! - 15.2) Plank.
       *   -# Creates a plank modelled as a horizontal box.
       *   -# Uses b2PolygonShape to define 'shape' and b2FixtureDef 'fd2' which defines the physical properties
       *   (density,shape) of the b2Body object 'body' which represents the plank.
       *   -# The Plank rests on the above created wedge(fulcrum) and its center is fixed on the tip of the wedge(fulcrum). 
       *   -# Also the planks friction is defined to be 0.05f.
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
      body->CreateFixture(fd2);

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(8.0f, 1.5f);
      jd.Initialize(sbody, body, anchor);
      m_world->CreateJoint(&jd);


      //The light box on the right side of the see-saw
       // var: b3 (local)
    /*! 16) A Box.
     *  -# Creates a light box as arectangular object in the simulation.
     *  -# Uses b2PolygonShape object 'shape2' to define shape of the b2Body object 'bo3' which represents the light box.
     *  -# 'fd3' is a b2FixtureDef onject defining the physical properities of 'b3'. Density of the box is 17(not small)
     */
      b2PolygonShape shape2;
      shape2.SetAsBox(2.0f, 2.0f);
      b2BodyDef bd3;
      bd3.position.Set(40.0f, 2.0f);
      bd3.type = b2_dynamicBody;
      b2Body *b3 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.01f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape2;
      b3->CreateFixture(fd3);
    }


    //The see-saw system at the bottom
     // var: ground (local)
    /*! 17) See Saw system.
     *  - See saw is modelled as a plank joined with a triangle which acts as its fulcrum.
     *  - Uses b2PolygonShape to define shapes(rectangular/triangular) of the b2Body objects(plank/fulcrum).
     */
    {
      //The triangle wedge
      // var: sbody (local)
    /*! - 17.1) See Saw system fulcrum.
     *
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

      //The plank on top of the wedge
      //  var: body (local)
      /*! - 17.2) Plank.
       *   -# Creates a plank modelled as a horizontal box.
       *   -# Uses b2PolygonShape to define 'shape' and b2FixtureDef 'fd2' which defines the physical properties
       *   (density,shape) of the b2Body object 'body' which represents the plank.
       *   -# The Plank rests on the above created wedge(fulcrum) and its center is fixed on the tip of the wedge(fulcrum). 
       *   -# Also the planks friction is defined to be 0.05f.
       */
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
      body->CreateFixture(fd2);

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-8.0f, 2.f);
      jd.Initialize(sbody, body, anchor);
      m_world->CreateJoint(&jd);

      //The light box on the right side of the see-saw
       // var: b3 (local)
    /*! 18) A Box.
     *  -# Creates a light box as arectangular object in the simulation.
     *  -# Uses b2PolygonShape object 'shape2' to define shape of the b2Body object 'b3' which represents the light box.
     *  -# 'fd3' is a b2FixtureDef onject defining the physical properities of 'b3'. Density of the box is 17(not small)
     */
      b2PolygonShape shape2;
      shape2.SetAsBox(2.0f, 2.0f);
      b2BodyDef bd3;
      bd3.position.Set(40.0f, 2.0f);
      bd3.type = b2_dynamicBody;
      b2Body *b3 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.01f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape2;
      b3->CreateFixture(fd3);
    }
    //The open box
     /**  The open box
          # The open box('box1') is formed by using 3 b2PolygonShape & b2FixtureDef.
      */
      {
       b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-3,5);
      //bd->fixedRotation = true;

        
	  b2Body* box2 = m_world->CreateBody(bd);
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
      fd1->shape = &bs1;
      box2->CreateFixture(fd1);
      
      bs1.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
      fd1->shape = &bs1;
      box2->CreateFixture(fd1);
      
      bs1.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
      fd1->shape = &bs1;
	  box2->CreateFixture(fd1);
      
      /*
 
      // Platform for conveyor
		{
			b2BodyDef bd;
			bd.position.Set(-5.0f, 5.0f);
			b2Body* body = m_world->CreateBody(&bd);

			b2PolygonShape shape;
			shape.SetAsBox(10.0f, 0.5f);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.friction = 0.8f;
			m_platform = body->CreateFixture(&fd);
		}

		// Boxes for testing
		for (int32 i = 0; i < 5; ++i)
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(-10.0f + 2.0f * i, 7.0f);
			b2Body* body = m_world->CreateBody(&bd);

			b2PolygonShape shape;
			shape.SetAsBox(0.5f, 0.5f);
			body->CreateFixture(&shape, 20.0f);
		}
*/

			

			// Define crank.
			{
	  
 	  b2BodyDef bd;
	  
	  b2Body* b1;		
	  bd.position.Set(-44.0f, 44.0f);
	  b1 = m_world->CreateBody(&bd);
	  b2Body* prevBody1 = b1;
	  bd.position.Set(44.0f, 44.0f);
	  b1 = m_world->CreateBody(&bd);
	  b2Body* prevBody2 = b1;
	  
	  bd.type = b2_dynamicBody;
	  bd.position.Set(-44.0f, 40.0f);
	  b2Body* body1 = m_world->CreateBody(&bd);
	  bd.position.Set(44.0f, 40.0f);
	  b2Body* body2 = m_world->CreateBody(&bd);
	   
	  
	  b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(0.5,4, b2Vec2(0.0f,-2.f), 0);
      fd1->shape = &bs1;
      body1->CreateFixture(fd1);
      body2->CreateFixture(fd1);
      
      
      bs1.SetAsBox(4,0.5, b2Vec2(0.0f,-2.f), 0);
      fd1->shape = &bs1;
      body1->CreateFixture(fd1);
      body2->CreateFixture(fd1);
      
      b2CircleShape circle;
      circle.m_radius = 1.0;
      circle.m_p.Set(0.0f,-2.f);
      fd1->shape = &circle;
      body1->CreateFixture(fd1);
      body2->CreateFixture(fd1);
      
      
		
 	b2RevoluteJointDef rjd;
		rjd.motorSpeed = 1.0f * b2_pi;
		rjd.maxMotorTorque = 10000.0f;
		rjd.enableMotor = true;
		rjd.Initialize(prevBody1, body1, b2Vec2(-44.0f, 38.0f));
		m_world->CreateJoint(&rjd);
		rjd.Initialize(prevBody2, body2, b2Vec2(44.0f, 38.0f));
		m_world->CreateJoint(&rjd);
	}

		
		
	
 }

}








//  ***************************************************************************
/* virtual  void base_sim_t::pre_solve(b2Contact* contact, const b2Manifold* oldManifold)
{
	const b2Manifold* manifold = contact->GetManifold();

	if (manifold->pointCount == 0)
	{
		return;
	}

	b2Fixture* fixtureA = contact->GetFixtureA();
	b2Fixture* fixtureB = contact->GetFixtureB();

	b2PointState state1[b2_maxManifoldPoints], state2[b2_maxManifoldPoints];
	b2GetPointStates(state1, state2, oldManifold, manifold);

	b2WorldManifold worldManifold;
	contact->GetWorldManifold(&worldManifold);

	for (int32 i = 0; i < manifold->pointCount && m_point_count < k_max_contact_points; ++i)
	{
		contact_point_t* cp = m_points + m_point_count;
		cp->fixtureA = fixtureA;
		cp->fixtureB = fixtureB;
		cp->position = worldManifold.points[i];
		cp->normal = worldManifold.normal;
		cp->state = state2[i];
		cp->normalImpulse = manifold->points[i].normalImpulse;
		cp->tangentImpulse = manifold->points[i].tangentImpulse;
		cp->separation = worldManifold.separations[i];
		++m_point_count;
	}
	//b2Fixture* fixtureA = contact->GetFixtureA();
	//b2Fixture* fixtureB = contact->GetFixtureB();
	if (fixtureA == m_platform)
	{
		contact->SetTangentSpeed(5.0f);
	}
	if (fixtureB == m_platform)
	{
		contact->SetTangentSpeed(-5.0f);
	}

}*/


    void base_sim_t::mouse_up(const b2Vec2& p)
    {
  if (m_mouseJoint)
  {
    m_world->DestroyJoint(m_mouseJoint);
    m_mouseJoint = NULL;
  }

     }

     void base_sim_t::mouse_move(const b2Vec2& p)
     {
  m_mouseWorld = p;

  if (m_mouseJoint)
  {
    m_mouseJoint->SetTarget(p);
  }
     }


  void base_sim_t::mouse_down ( const b2Vec2& p)
  {
  m_mouseWorld = p;

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
  {
    b2Body* body = callback.m_fixture->GetBody();
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
