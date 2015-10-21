/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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
#define DEGTORAD 3.14/180
#define RADTODEG 180/3.14
#include "render.hpp"

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/freeglut.h>
#endif

#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <ctime>
using namespace std;

void debug_draw_t::DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color)
{
	glColor3f(color.r, color.g, color.b);
	glBegin(GL_LINE_LOOP);
	for (int32 i = 0; i < vertexCount; ++i)
	{
		glVertex2f(vertices[i].x, vertices[i].y);
	}
	glEnd();
}

void debug_draw_t::DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color)
{
	glEnable(GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glColor4f(0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 0.5f);
	glPushMatrix();
	//!Texture
	/*!
	The texture to be added.
	*/
	GLuint tex;
	glGenTextures(1,&tex);
	//!GL_TEXTURE_2D
	/*!
	The texture has to be applied on 2d surface.
	*/
	glBindTexture(GL_TEXTURE_2D, tex);
	glBegin(GL_TRIANGLE_STRIP);
	for (int32 i = 0; i < vertexCount; ++i)
	{
		glTexCoord2f(vertices[i].x, vertices[i].y);
		glVertex2f(vertices[i].x, vertices[i].y);
	}
	glPopMatrix();
	glEnd();
	glDisable(GL_BLEND);

	glColor4f(color.r, color.g, color.b, 1.0f);
	glBegin(GL_LINE_LOOP);
	for (int32 i = 0; i < vertexCount; ++i)
	{
		glVertex2f(vertices[i].x, vertices[i].y);
	}
	glEnd();
}

void debug_draw_t::DrawCircle(const b2Vec2& center, float32 radius, const b2Color& color)
{
	const float32 k_segments = 16.0f;
	const float32 k_increment = 2.0f * b2_pi / k_segments;
	float32 theta = 0.0f;
	glColor3f(color.r, color.g, color.b);
	glBegin(GL_LINE_LOOP);
	for (int32 i = 0; i < k_segments; ++i)
	{
		b2Vec2 v = center + radius * b2Vec2(cosf(theta), sinf(theta));
		glVertex2f(v.x, v.y);
		theta += k_increment;
	}
	glEnd();
}

void debug_draw_t::DrawSolidCircle(const b2Vec2& center, float32 radius, const b2Vec2& axis, const b2Color& color)
{
	const float32 k_segments = 16.0f;
	const float32 k_increment = 2.0f * b2_pi / k_segments;
	float32 theta = 0.0f;
	glEnable(GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glColor4f(0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 0.5f);
	glBegin(GL_TRIANGLE_FAN);
	for (int32 i = 0; i < k_segments; ++i)
	{
		b2Vec2 v = center + radius * b2Vec2(cosf(theta), sinf(theta));
		glVertex2f(v.x, v.y);
		theta += k_increment;
	}
	glEnd();
	glDisable(GL_BLEND);

	theta = 0.0f;
	glColor4f(color.r, color.g, color.b, 1.0f);
	glBegin(GL_LINE_LOOP);
	for (int32 i = 0; i < k_segments; ++i)
	{
		b2Vec2 v = center + radius * b2Vec2(cosf(theta), sinf(theta));
		glVertex2f(v.x, v.y);
		theta += k_increment;
	}
	glEnd();

	b2Vec2 p = center + radius * axis;
	glBegin(GL_LINES);
	glVertex2f(center.x, center.y);
	glVertex2f(p.x, p.y);
	glEnd();
}

void debug_draw_t::DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color)
{
	glColor3f(color.r, color.g, color.b);
	glBegin(GL_LINES);
	glVertex2f(p1.x, p1.y);
	glVertex2f(p2.x, p2.y);
	glEnd();
}

void debug_draw_t::DrawTransform(const b2Transform& xf)
{
	b2Vec2 p1 = xf.p, p2;
	const float32 k_axisScale = 0.4f;
	glBegin(GL_LINES);

	glColor3f(1.0f, 0.5f, 0.0f);
	glVertex2f(p1.x, p1.y);
	p2 = p1 + k_axisScale * xf.q.GetXAxis();
	glVertex2f(p2.x, p2.y);

	glColor3f(0.0f, 1.0f, 0.5f);
	glVertex2f(p1.x, p1.y);
	p2 = p1 + k_axisScale * xf.q.GetYAxis();
	glVertex2f(p2.x, p2.y);

	glEnd();
}

void debug_draw_t::DrawPoint(const b2Vec2& p, float32 size, const b2Color& color)
{
	glPointSize(size);
	glBegin(GL_POINTS);
	glColor3f(color.r, color.g, color.b);
	glVertex2f(p.x, p.y);
	glEnd();
	glPointSize(1.0f);
}

void debug_draw_t::DrawString(int x, int y, const char *string, ...)
{
	char buffer[128];

	va_list arg;
	va_start(arg, string);
	vsprintf(buffer, string, arg);
	va_end(arg);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	int w = glutGet(GLUT_WINDOW_WIDTH);
	int h = glutGet(GLUT_WINDOW_HEIGHT);
	gluOrtho2D(0, w, h, 0);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glColor3f(0.9f, 0.6f, 0.6f);
	glRasterPos2i(x, y);
	int32 length = (int32)strlen(buffer);
	for (int32 i = 0; i < length; ++i)
	{
		glutBitmapCharacter(GLUT_BITMAP_8_BY_13, buffer[i]);
	}

	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}

void debug_draw_t::DrawAABB(b2AABB* aabb, const b2Color& c)
{
	glColor3f(c.r, c.g, c.b);
	glBegin(GL_LINE_LOOP);
	glVertex2f(aabb->lowerBound.x, aabb->lowerBound.y);
	glVertex2f(aabb->upperBound.x, aabb->lowerBound.y);
	glVertex2f(aabb->upperBound.x, aabb->upperBound.y);
	glVertex2f(aabb->lowerBound.x, aabb->upperBound.y);
	glEnd();
}
void debug_draw_t::DrawBall()
{
      glColor3f(1,1,1);//white

      //nose and eyes
      glPointSize(4);
      glBegin(GL_POINTS);
      glVertex2f( 0, 0 );
      glVertex2f(-0.5, 0.5 );
      glVertex2f( 0.5, 0.5 );
      glEnd();

      //mouth
      glBegin(GL_LINES);
      glVertex2f(-0.5,  -0.5 );
      glVertex2f(-0.16, -0.6 );
      glVertex2f( 0.16, -0.6 );
      glVertex2f( 0.5,  -0.5 );
      glEnd();

      //circle outline
      glBegin(GL_LINE_LOOP);
      for (float a = 0; a < 360 * DEGTORAD; a += 30 * DEGTORAD)
        glVertex2f( sinf(a), cosf(a) );
      glEnd();
}
    void debug_draw_t::renderAtBodyPosition(b2Body* m_body,bool m_contacting) {

      //get current position from Box2D
      b2Vec2 pos = m_body->GetPosition();
      float angle = m_body->GetAngle();

      //call normal render at different position/rotation
      glPushMatrix();
      glTranslatef( pos.x, pos.y, 0 );
      glScalef(2,2,1);
      glRotatef( angle * RADTODEG, 0, 0, 1 );//OpenGL uses degrees here
                    if ( m_contacting )
    glColor3f(1,0,0);//red
  else
    glColor3f(0,0,1);//white

      //nose and eyes
      glPointSize(4);
      glBegin(GL_POINTS);
      glVertex2f( 0, 0 );
      glVertex2f(-0.5, 0.5 );
      glVertex2f( 0.5, 0.5 );
      glEnd();
/*
      glPointSize(1);
      glBegin(GL_POINTS);
for(int i=0;i<20;i++)
{int g=rand();
if(g%4==0){
glVertex2f( rand()/(float)RAND_MAX,rand()/(float)RAND_MAX );}
if(g%4==1){
glVertex2f( -1*rand()/(float)RAND_MAX,rand()/(float)RAND_MAX );}
if(g%4==2){
glVertex2f( rand()/(float)RAND_MAX,-1*rand()/(float)RAND_MAX );}
if(g%4==3){
glVertex2f( -1*rand()/(float)RAND_MAX,-1*rand()/(float)RAND_MAX );}
}
*/      glEnd();

      //mouth
      glBegin(GL_LINES);
      glVertex2f(-0.5,  -0.5 );
      glVertex2f(-0.16, -0.6 );
      glVertex2f( 0.16, -0.6 );
      glVertex2f( 0.5,  -0.5 );
      glEnd();

      //circle outline
      glBegin(GL_LINE_LOOP);
      for (float a = 0; a < 360 * DEGTORAD; a += 30 * DEGTORAD)
        glVertex2f( sinf(a), cosf(a) );
      glEnd();
      glPopMatrix();


    }
    void debug_draw_t::renderStars(b2Body* m_body) {
    b2Vec2 pos = m_body->GetPosition();
    //srand(time(0));
    glPushMatrix();
    glTranslatef( pos.x, pos.y, 0 );
    //int r = rand()/(float)RAND_MAX;
    //float g = rand()/(float)RAND_MAX;
    //float b = rand()/(float)RAND_MAX;
    //c = ((rand()+rand()+rand())/(float)RAND_MAX)/3;
    //if(r%2){
    glColor3f(1,1,0);
    //else{glColor3f(0,0,1);}
          glPointSize(1);
      glBegin(GL_POINTS);
      glVertex2f( 0, 0 );
      glEnd();
      glPopMatrix();

    }
