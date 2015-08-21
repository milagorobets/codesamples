#ifndef H_OPENGL
#define H_OPENGL

#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "glu32.lib")

#pragma once

#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <math.h>


// Declare globals

#ifndef M_PI
#define M_PI  3.14159265358979323846f
#endif

#define DEGREES_PER_PIXEL 0.6f
#define RADIANS_PER_PIXEL 0.002f
#define UNITS_PER_PIXEL 0.01f
#define UNITS_PER_WHEELTICK 0.35f
#define ZOOM_FACTOR .04f

extern GLfloat angle;

struct c_p_v{
		GLfloat x;
		GLfloat y;
		GLfloat z;
		GLfloat n_x;
		GLfloat n_y;
		GLfloat n_z;
		GLfloat theta;
};

struct g_mouseState{
	bool leftButton;
	bool rightButton;
	bool middleButton;
	int x;
	int y;
	int old_x;
	int old_y;
	int x_down; // For rotating parts, starting x
	int y_down; // For rotating parts, starting y
	int clicked_element;
};

extern c_p_v f_p[12];
extern c_p_v temp;
extern GLfloat modifier;
extern bool FLAG_GL_HOVER;
extern bool FLAG_PHALANX_BUSY;
extern g_mouseState  MouseState;
extern float g_xRotation, g_yRotation, g_xViewRot;

static float eyex, eyey, eyez;  // eye x,y,z values for gluLookAt (location of eye)
static float focusx, focusy, focusz; // the point the eye is looking at

namespace GUI_1{ref class Form1;}
namespace ModelSpace{ref class Model;}
	
namespace OpenGLForm 
{
	using namespace System::Windows::Forms;

	
	public ref class COpenGL: public System::Windows::Forms::NativeWindow
	{
	public:

		static GUI_1::Form1^ f_one;
		static ModelSpace::Model^ palmHand;
		static ModelSpace::Model^ phalanx1Hand;
		static ModelSpace::Model^ phalanx2Hand;
		static ModelSpace::Model^ phalanx3Hand;
		
		COpenGL();
		OpenGLForm::COpenGL::COpenGL(System::Windows::Forms::Panel ^ parentForm, System::Windows::Forms::Label ^ lbl, GLsizei iWidth, GLsizei iHeight);
		int COpenGL_one(GLsizei iWidth, GLsizei iHeight);
		
		// Mathematical manipulations
		GLfloat dt(GLfloat a, GLfloat b, GLfloat c, GLfloat d);
		void rotatePointAboutLine(GLfloat a, GLfloat b, GLfloat c, GLfloat u, GLfloat v, GLfloat w, GLfloat x, GLfloat y, GLfloat z, GLfloat angle);
		GLfloat dotProduct(c_p_v a, c_p_v b);
		void findOrthogonalAxis(int v);
		void spinAroundPoint(int vertex, GLfloat angle);
		GLfloat addAngle(int vertex, GLfloat angle);
		void updateAngle(int vertex, GLfloat angle);
		

		// OpenGL stuff:
		
		// Scene rendering code
		int DrawGLScene(void);

		// Object selection rendering code
		int DrawGLSceneSelect(void);

		// OpenGL buffer swap
		void SwapOpenGLBuffers(void);

		// Window murder :(
		GLvoid KILLGLWindow(GLvoid);

		// Update model coordinates during mouse rotations
		GLvoid updateRotations(GLvoid);

		// Draw cylinders (no longer used)
		void drawCylinder(int v);

		// Mouse selection function
		GLvoid selectGL(int x, int y);

		// Determines which objects have been clicked on
		GLvoid list_hits(GLint hits, GLuint *names);

		// Updating the angle and orientation of phalanx after rotation
		GLvoid update_rotation_phalanx();

		// Updating form sliders after mouse interaction
		GLvoid update_sliders(GLint vertex);

		// Updating model components' orientation
		GLvoid update_model_angles(GLint vertex, GLfloat new_angle);


	private:
		HDC m_hDC;
		HGLRC m_hglrc;
		GLfloat	rtri;				// Angle for the triangle
		GLfloat	rquad;				// Angle for the quad

	protected:
		~COpenGL(System::Void)
		{
			this->DestroyHandle();
		}
		GLint MySetPixelFormat(HDC hdc);
		bool InitGL(GLvoid);
		GLvoid ReSizeGLScene(GLsizei width, GLsizei height);	// Resize and initialise the gl window

};
}
#endif
