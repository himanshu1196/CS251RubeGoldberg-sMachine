
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
{  //b2Body *body3;
   //b2Body* ground;
  /**  The is the constructor
   * This is the documentation block for the constructor.
   */

  dominos_t::dominos_t()
  {
    //Ground
    /*! \var ground
     * \brief pointer to the body ground
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
    {
      b2PolygonShape shape;
      shape.SetAsBox(3.1f, 0.25f);

      b2BodyDef bd;
      bd.position.Set(-31.0f, 38.0f);
      b2Body* b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
    }

    //horizontal shelf
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
    {
      //The triangle wedge
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

    //The chain of pendulums that push the saw
    {
      b2Body* b2;
      {
            b2PolygonShape shape;
            shape.SetAsBox(6,0.5f);
            b2BodyDef bd;
            bd.position.Set(7,40);
            b2 = m_world->CreateBody(&bd);
            b2->CreateFixture(&shape, 0.01f);
      }
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
    {
      b2BodyDef bd;
      bd.position.Set(16.5f,34.0f);
      b2Body* b1=m_world->CreateBody(&bd);
	    b2PolygonShape shape;
	    shape.SetAsBox(0.01f,.01f);
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
/*
      b2BodyDef box;
      box.position.Set(20.5f,38.0f);
      b2PolygonShape boxshape;
      boxshape.SetAsBox(2.0f,0.1f);
      fd.shape=&boxshape;
      fd.density=1.f;
      b2Body* boxx=m_world->CreateBody(&box);
      boxx->CreateFixture(&fd);

      b2BodyDef box1;
      box1.position.Set(20.5f,34.5f);
      box1.type=b2_dynamicBody;
      //b2PolygonShape boxshape;
      boxshape.SetAsBox(.01f,0.01f);
      fd.shape=&boxshape;
      fd.density=1.f;
      b2Body* boxx1=m_world->CreateBody(&box1);
      boxx1->CreateFixture(&fd);

      b2BodyDef box2;
      box2.position.Set(20.5f,34.4f);
      box2.type=b2_dynamicBody;
      //b2PolygonShape boxshape;
      boxshape.SetAsBox(.01f,0.01f);
      fd.shape=&boxshape;
      fd.density=1.f;
      b2Body* boxx2=m_world->CreateBody(&box2);
      boxx2->CreateFixture(&fd);

      b2BodyDef ball;
      ball.position.Set(20.5f,30.0f);
      b2CircleShape circle;
      circle.m_radius=1.0f;
      fd.shape=&circle;
      ball.type=b2_dynamicBody;
      fd.density=1.f;
      b2Body* cir=m_world->CreateBody(&ball);
      cir->CreateFixture(&fd);
*/
	    b2MotorJointDef mjd;
      //b2RevoluteJointDef string;
      //b2RevoluteJointDef string1;
      //b2RevoluteJointDef string2;
      //b2Vec2 anchor;
      //anchor.Set(20.5f,38.0f);
      //b2Vec2 anchor1;
      //anchor1.Set(20.5f,34.5f);
      //b2Vec2 anchor2;
      //anchor2.Set(20.5f,34.4f);
      //string.Initialize(boxx,boxx1,anchor);
      //string1.Initialize(boxx1,boxx2,anchor1);
      //string2.Initialize(boxx2,cir,anchor2);
	    mjd.Initialize(b1,body);
    	mjd.maxForce=1.f;
	    mjd.maxTorque=1000.f;
	   //mjd.motorSpeed=1.0f;
	   //mjd.enableMotor=true;
	    m_world->CreateJoint(&mjd);
      //b2Joint* jstring=m_world->CreateJoint(&string);
      //m_world->CreateJoint(&string1);
      //m_world->CreateJoint(&string2);
      //m_world->DestroyJoint(jstring);
    }
    /*The planks were here ########################
//Top 3 horizontal planks
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
#################################################*/
    //The pulley system (bottom right corner)
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(18,5);
      bd->fixedRotation = true;

      //The open box
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 10.0;
      fd3->friction = 0.5;
      fd3->restitution = 0.f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
      fd3->shape = &bs3;

      b2Body* box2 = m_world->CreateBody(bd);
      box2->CreateFixture(fd1);
      box2->CreateFixture(fd2);
      box2->CreateFixture(fd3);

      //The bar
      bd->position.Set(30,5);
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
    {
      //The triangle wedge
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
    {
      //The triangle wedge
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
      {
       b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-3,5);
      //bd->fixedRotation = true;

      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 10.0;
      fd3->friction = 0.5;
      fd3->restitution = 0.f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
      fd3->shape = &bs3;

      b2Body* box2 = m_world->CreateBody(bd);
      box2->CreateFixture(fd1);
      box2->CreateFixture(fd2);
      box2->CreateFixture(fd3);
      }

}








//  ***************************************************************************


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
