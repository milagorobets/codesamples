#include "stdafx.h"
#include "Form1.h"

GLfloat angle = 0.0f; // Rotation angle value
c_p_v f_p[12]; // All vertex coordinates
c_p_v temp; // Temporary coordinate/vector variable
GLfloat modifier; // Modifier for matrix manipulations
bool FLAG_GL_HOVER; // Flag for mouse hover over panel
bool FLAG_PHALANX_BUSY; // Flag to only handle one phalanx at a time
g_mouseState  MouseState; // State of mouse: leftButton, rightButton, middleButton, x, y, old_x, old_y
float g_xRotation, g_yRotation, g_xViewRot; // Rotation values calculated from mouse actions

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
COpenGL(): Default Constructor
-------------------------------------------------------------------------
* Called by default by the main
* Creates an instance
-------------------------------------------------------------------------
REQUIRES: nothing
RETURNS: nothing
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
OpenGLForm::COpenGL::COpenGL(){};

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
COpenGL(non void): Constructor #2
-------------------------------------------------------------------------
* Backup constructor
* Creates an instance
* Binds OpenGL to a panel on the form
* Initializes the phalanx coordinates
-------------------------------------------------------------------------
REQUIRES: Panel pointer, Label pointer, window width, window height
RETURNS: nothing
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
 OpenGLForm::COpenGL::COpenGL(System::Windows::Forms::Panel ^ parentForm, System::Windows::Forms::Label ^ lbl, GLsizei iWidth, GLsizei iHeight)
		{
			CreateParams^ cp = gcnew CreateParams;
			c_p_v v1, v2;

			//form_pt::frm->label2 = "moo";
						
			// Set the position on the form
			cp->X = 0;
			cp->Y = 0;
			cp->Height = iHeight;
			cp->Width = iWidth;

			// Specify the form as the parent.
			cp->Parent = parentForm -> Handle;
			
			// Create as a child of the specified parent and make OpenGL compliant (no clipping)
			cp->Style = WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | WS_CLIPCHILDREN;

			// Create the actual window
			this->CreateHandle(cp);

			m_hDC = GetDC((HWND)this->Handle.ToPointer());

			if(m_hDC)
			{
				MySetPixelFormat(m_hDC);
				ReSizeGLScene(iWidth, iHeight);
				InitGL();
			}

			rtri = 0.0f;
			rquad = 0.0f;

			// Initialize finger 1
			f_p[0].x = 0.5f; f_p[0].y = 0.0f; f_p[0].z = 0.0f; // Origin
            f_p[1].x = 1.0f; f_p[1].y = 0.5f; f_p[1].z = 0.0f; // Phalanx 1
			f_p[2].x = 1.2f; f_p[2].y = 1.0f; f_p[2].z = 0.0f; // Phalanx 2
			f_p[3].x = 0.7f; f_p[3].y = 1.5f; f_p[3].z = 0.0f; // Phalanx 3

			// Initialize finger 2
			f_p[4].x = -0.5f; f_p[4].y = 0.0f; f_p[4].z = 0.0f; // Origin
            f_p[5].x = -1.0f; f_p[5].y = 0.5f; f_p[5].z = 0.0f; // Phalanx 1
			f_p[6].x = -1.2f; f_p[6].y = 1.0f; f_p[6].z = 0.0f; // Phalanx 2
			f_p[7].x = -0.7f; f_p[7].y = 1.5f; f_p[7].z = 0.0f; // Phalanx 3

			// Initialize finger 3
			f_p[8].x = 0.0f; f_p[8].y = 0.0f; f_p[8].z = 0.5f; // Origin
            f_p[9].x = 0.0f; f_p[9].y = 0.5f; f_p[9].z = 1.0f; // Phalanx 1
			f_p[10].x = 0.0f; f_p[10].y = 1.0f; f_p[10].z = 1.2f; // Phalanx 2
			f_p[11].x = 0.0f; f_p[11].y = 1.5f; f_p[11].z = 1.7f; // Phalanx 3	

			for (int i = 0; i < 12; i++)
			{
				if ((i == 0)|(i == 4)|(i == 8))
				{	
					temp.x = 0.0f;
					temp.y = 0.0f;
					temp.z = 0.0f;
				}
				else
				{
					temp = f_p[i-1];
				}

				// Calculate vectors around axis
				v1.x = temp.x - f_p[i].x;
				v1.y = temp.y - f_p[i].y;
				v1.z = temp.z - f_p[i].z;

				v2.x = f_p[i+1].x - f_p[i].x;
				v2.y = f_p[i+1].y - f_p[i].y;
				v2.z = f_p[i+1].z - f_p[i].z;
				f_p[i].theta = (GLfloat)(acos(dotProduct(v1,v2)) * 180.0 / 3.1415926);
			}
		}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
COpenGL(non void): Constructor #3
-------------------------------------------------------------------------
* Constructor used to update the object properties
* Binds OpenGL to a panel on the form
* Initializes the phalanx coordinates
-------------------------------------------------------------------------
REQUIRES: Window width, window height
RETURNS: nothing
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
 int OpenGLForm::COpenGL::COpenGL_one(GLsizei iWidth, GLsizei iHeight){
			CreateParams^ cp = gcnew CreateParams;
			c_p_v v1, v2;

			palmHand -> Model_init();
			phalanx1Hand -> Model_init();
			phalanx2Hand -> Model_init();
			phalanx3Hand -> Model_init();
			
			
	//		palmHand -> loadModel("model3.obj");
			phalanx1Hand -> loadModel("");
			phalanx2Hand -> loadModel("");
			phalanx3Hand -> loadModel("");
			palmHand->loadModel("phalanx1_5.obj");
	//		palmHand->loadModel("palm2.obj");
	//		phalanx1Hand->loadModel("phalanx1_5.obj");
	/*		phalanx1Hand -> loadModel("phalanx1_5.obj");
	/*		phalanx2Hand -> loadModel("phalanx2_3.obj");
			phalanx3Hand -> loadModel("phalanx3_5.obj");*/
			


			// Set the position on the form
			cp->X = 0;
			cp->Y = 0;
			cp->Height = iHeight;
			cp->Width = iWidth;

			System::Windows::Forms::Panel ^ tinyPanel = f_one -> returnpanel();
			
			// Specify the form as the parent.
			cp->Parent = tinyPanel -> Handle;
			tinyPanel->Focus();
			
			// Create as a child of the specified parent and make OpenGL compliant (no clipping)
			cp->Style = WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | WS_CLIPCHILDREN | WS_DISABLED;

			// Create the actual window
			this->CreateHandle(cp);

			m_hDC = GetDC((HWND)this->Handle.ToPointer());

			if(m_hDC)
			{
				MySetPixelFormat(m_hDC);
				ReSizeGLScene(iWidth, iHeight);
				InitGL();
			}

			rtri = 0.0f;
			rquad = 0.0f;

			// Initialize
			f_p[0].theta = 30.0f;
			f_p[1].theta = 30.0f;
			f_p[2].theta = 30.0f;

			f_p[4].theta = 30.0f;
			f_p[5].theta = 30.0f;
			f_p[6].theta = 30.0f;

			f_p[8].theta = 30.0f;
			f_p[9].theta = 30.0f;
			f_p[10].theta = 30.0f;

			f_p[0].theta = 90.0f;
			f_p[1].theta = 90.0f;
			f_p[2].theta = 90.0f;

			f_p[4].theta = 90.0f;
			f_p[5].theta = 90.0f;
			f_p[6].theta = 90.0f;

			f_p[8].theta = 90.0f;
			f_p[9].theta = 90.0f;
			f_p[10].theta = 90.0f;
		return TRUE;
 }

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
dt(non void): Matrix determinant calculator
-------------------------------------------------------------------------
* Calculates determinant calculator
-------------------------------------------------------------------------
REQUIRES: 2x2 matrix elements: a, b, c, d
RETURNS: float - determinant
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
 GLfloat OpenGLForm::COpenGL::dt(GLfloat a, GLfloat b, GLfloat c, GLfloat d)
		{
			return (a*d-b*c);
		}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
rotatePointAboutLine(non void): Spins point around an axis
-------------------------------------------------------------------------
* Rotates point around an axis
* Writes new coordinates to temp
-------------------------------------------------------------------------
REQUIRES: Point at (x,y,z) about vector (u,v,w) passing through (a,b,c) 
		  through angle 
RETURNS: nothing
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void OpenGLForm::COpenGL::rotatePointAboutLine(GLfloat a, GLfloat b, GLfloat c, GLfloat u, GLfloat v, GLfloat w, GLfloat x, GLfloat y, GLfloat z, GLfloat angle)
		{
			GLfloat bv = b*v;
			GLfloat cw = c*w;
			GLfloat ux = u*x;
			GLfloat vy = v*y;
			GLfloat wz = w*z;
			GLfloat cv = c*v;
			GLfloat bw = b*w;
			GLfloat wy = w*y;
			GLfloat vz = v*z;
			GLfloat au = a*u;
			GLfloat cu = c*u;
			GLfloat aw = a*w;
			GLfloat wx = w*x;
			GLfloat uz = u*z;
			GLfloat bu = b*u;
			GLfloat av = a*v;
			GLfloat vx = v*x;
			GLfloat uy = u*y;
			GLfloat ta = angle/180.0f * 3.14159f;

			temp.x = (a*(pow(v,2)+pow(w,2)) - u*(bv+cw-ux-vy-wz))*(1.0f-cos(ta)) + x*cos(ta) + (-1.0f*cv+bw-wy+vz)*sin(ta);
			temp.y = (b*(pow(u,2)+pow(w,2)) - v*(au+cw-ux-vy-wz))*(1.0f-cos(ta)) + y*cos(ta) + (cu-aw+wx-uz)*sin(ta);
			temp.z = (c*(pow(u,2)+pow(v,2)) - w*(au+bv-ux-vy-wz))*(1.0f-cos(ta)) + z*cos(ta) + (-1.0f*bu+av-vx+uy)*sin(ta);
		}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
dotProduct(non void): Dot product calculator
-------------------------------------------------------------------------
* Calculates dot product of two vectors
-------------------------------------------------------------------------
REQUIRES: Vector a, Vector b
RETURNS: dot product (float)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
GLfloat OpenGLForm::COpenGL::dotProduct(c_p_v a, c_p_v b)
		{
			return ((a.x*b.x)+(a.y*b.y)+(a.z*b.z));
		}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
findOrthogonalAxis(non void): Finds axis orthogonal to two vectors
-------------------------------------------------------------------------
* Uses 3 vertices to calculate rotational axis for a point
* Assumes all fingers rotate the same
-------------------------------------------------------------------------
REQUIRES: Number of vertex for axis
RETURNS: nothing, writes to temp variable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void OpenGLForm::COpenGL::findOrthogonalAxis(int v)
		{
				c_p_v a, b, ref, nv;
				GLfloat mag, tang, t_d, mod;

				ref.x = 0.0f; ref.y = 1.0f; ref.z = 0.0f;

				if ((v == 0)|(v == 4)|(v == 8))
				{	
					temp.x = 0.0f;
					temp.y = 0.0f;
					temp.z = 0.0f;
				}
				else
				{
					temp = f_p[v-1];
				}

				// Calculate vectors around axis
				a.x = temp.x - f_p[v].x;
				a.y = temp.y - f_p[v].y;
				a.z = temp.z - f_p[v].z;

				b.x = f_p[v+1].x - f_p[v].x;
				b.y = f_p[v+1].y - f_p[v].y;
				b.z = f_p[v+1].z - f_p[v].z;

				if (f_p[v].theta > 180){
					mod = -1.0f;
				}
				else
				{
					mod = 1.0f;			
				}

				// Determinant
				nv.x = a.y*b.z - a.z*b.y;
				nv.y = a.z*b.x - a.x*b.z;
				nv.z = a.x*b.y - a.y*b.x; 

				mag = sqrt(pow(nv.x,2) + pow(nv.y,2) + pow(nv.z,2));

				tang = atan2(mag, dotProduct(a,b));

				t_d = dotProduct(nv, ref);
				// Write orthogonal vector into temp
				temp.x = mod * nv.x/mag;
				temp.y = mod * nv.y/mag;
				temp.z = mod * nv.z/mag;
		}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
drawCylinder(void): Draws a cylinder
-------------------------------------------------------------------------
* Draws cylinder between two vertices
-------------------------------------------------------------------------
REQUIRES: Number of vertex for beginning
RETURNS: Nothing
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void OpenGLForm::COpenGL::drawCylinder(int v)
{
	c_p_v a, xross, zaxis;
	GLfloat rotation_angle, mag;
	zaxis.x = 0.0f; zaxis.y = 0.0f; zaxis.z = 1.0f;
	
	// Move origin to vertex
	glTranslated(f_p[v].x, f_p[v].y, f_p[v].z);

	// Calculate vector from f_p[v] to f_p[v+1]
	a.x = f_p[v+1].x - f_p[v].x;
	a.y = f_p[v+1].y - f_p[v].y;
	a.z = f_p[v+1].z - f_p[v].z;

	// Cross z-axis with calculated vector to get rotation axis
	xross.x = zaxis.y*a.z - zaxis.z*a.y; // a2b3 - a3b2
	xross.y = zaxis.z*a.x - zaxis.x*a.z; // a3b1 - a1b3
	xross.z = zaxis.x*a.y - zaxis.y*a.x; // a1b2 - a2b1

	// Use dot product to calculate angle of rotation
	mag = sqrt(pow(a.x,2) + pow(a.y,2) + pow(a.z,2));
	rotation_angle = 180.0f / M_PI * acos(dotProduct(zaxis, a) / mag);

	// Rotate
	glRotatef(rotation_angle, xross.x, xross.y , xross.z);

	// Draw cylinder
	GLUquadricObj * quadObj = gluNewQuadric();
	gluCylinder(quadObj, 0.5f, 0.5f, mag, 32, 32);

	// Rotate back (negative angle) around same axis
	glRotatef((-1.0f * rotation_angle), xross.x, xross.y, xross.z);

	// Translate back to normal vertex
	glTranslated(-f_p[v].x, -f_p[v].y, -f_p[v].z);

}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
findOrthogonalAxis(non void): Finds axis orthogonal to two vectors
-------------------------------------------------------------------------
* Uses 3 vertices to calculate rotational axis for a point
* Assumes all fingers rotate the same
-------------------------------------------------------------------------
REQUIRES: Number of vertex for axis
RETURNS: nothing, writes to temp variable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void OpenGLForm::COpenGL::spinAroundPoint(int vertex, GLfloat angle)
		{
			GLfloat a, b, c, u, v, w, limit;

			a = f_p[vertex].x;
			b = f_p[vertex].y;
			c = f_p[vertex].z;

			u = f_p[vertex].n_x;
			v = f_p[vertex].n_y;
			w = f_p[vertex].n_z;

			if ((vertex == 0)|(vertex == 4)|(vertex == 8)) // If at finger origin, whole finger rotates
			{
				limit = 3;
			}
			else if ((vertex == 1)|(vertex == 5)|(vertex == 9))
			{
				limit = 2;
			}
			else if ((vertex == 2)|(vertex == 6)|(vertex == 10))
			{
				limit = 1;
			}
			else
			{
				limit = 0;
			}

			GLfloat t_a_1 = addAngle(vertex, angle);
			GLfloat t_a_2 =  t_a_1 - f_p[vertex].theta;
			f_p[vertex].theta = t_a_1;
			for (int i = 0; i < limit; i++)
			{
				rotatePointAboutLine(a, b, c, u, v, w, f_p[vertex+1].x,f_p[vertex+1].y, f_p[vertex+1].z, t_a_2);

				f_p[vertex+1].x = temp.x;
				f_p[vertex+1].y = temp.y;
				f_p[vertex+1].z = temp.z;
				vertex++;				
			}	

		}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
findOrthogonalAxis(non void): Finds axis orthogonal to two vectors
-------------------------------------------------------------------------
* Uses 3 vertices to calculate rotational axis for a point
* Assumes all fingers rotate the same
-------------------------------------------------------------------------
REQUIRES: Number of vertex for axis
RETURNS: nothing, writes to temp variable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
GLfloat OpenGLForm::COpenGL::addAngle(int vertex, GLfloat angle)
		{	
			GLfloat temp = f_p[vertex].theta + angle;

			if ((vertex == 0)|(vertex == 1)|(vertex == 2)|(vertex == 5)|(vertex == 6)|(vertex == 9)|(vertex == 10))
			{
					if (temp <= 90.0f){temp = 90.0f;}
					else if (temp >= 270.0f){temp = 270.0f;}
			}
			else if (vertex == 4)
			{
					if (temp <= -20.0f){temp = -20.0f;}
					else if (temp >= 90.0f){temp = 90.0f;}
			}
			else if (vertex == 8)
			{
					if (temp <= -90.0f){temp = -90.0f;}
					else if (temp >= 20.0f){temp = 20.0f;}
			}
			
			return temp;
		}

void OpenGLForm::COpenGL::updateAngle(int vertex, GLfloat angle)
{	
		GLfloat temp = f_p[vertex].theta + angle;

		if ((vertex == 0)|(vertex == 1)|(vertex == 2)|(vertex == 5)|(vertex == 6)|(vertex == 9)|(vertex == 10))
		{
				if (temp <= 0.0f){f_p[vertex].theta = 0.0f;}
				else if (temp >= 180.0f){f_p[vertex].theta = 180.0f;}
				else {f_p[vertex].theta = temp;}
		}
		else if (vertex == 4)
		{
				if (temp <= 70.0f){f_p[vertex].theta = 70.0f;}
				else if (temp >= 180.0f){f_p[vertex].theta = 180.0f;}
				else {f_p[vertex].theta = temp;}
		}
		else if (vertex == 8)
		{
				if (temp <= 0.0f){f_p[vertex].theta = 0.0f;}
				else if (temp >= 110.0f){f_p[vertex].theta = 110.0f;}
				else {f_p[vertex].theta = temp;}
		}
		//g_xViewRot = 0;
			
			
}

int OpenGLForm::COpenGL::DrawGLScene(void)
		{
			double t_x, t_y, t_z;
			glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT); // Clear screen and depth buffer
			glLoadIdentity();
			gluLookAt(eyex+100.0f, eyey+100.0f, eyez+100.0f, focusx, focusy, focusz, 0, 1, 0);
			//gluLookAt(eyex , eyey , eyez , focusx, focusy, focusz, 0, 1, 0);
			glPushMatrix();
									
			glRotatef(g_xRotation, 0,1,0); // Rotate with mouse
			glRotatef(g_yRotation, 1,0,0);
		
			// --------------------------------------------------------------------------------------
			// DRAW PALM
			glColor3f(1.0f, 1.0f, 1.0f); // Use proper color - white
			palmHand -> draw();
			// --------------------------------------------------------------------------------------
			// --------------------------------------------------------------------------------------
			// DRAW FINGER 1
			// CONSTANT: Move to first vertex
			glTranslatef(0.0f, 0.0f, 1.87f); 
									
			glRotatef(f_p[0].theta, 0, 0, 1); // Do rotation for phalanx 1 with specified angle
			glColor3f(16.0f/255.0f, 78.0f/255.0f, 139.0f/255.0f); // Use proper color - blue
			phalanx1Hand -> draw(); 
						
			// CONSTANT: Necessary adjustment because of model file offset (nobody's perfect, okay?)
			glTranslatef(0.0f, 0.03f, 0.0f);
						
			// CONSTANT: Move to second vertex
			glTranslatef(1.3f, 0.0f, 0.0f); 
			
			glRotatef(f_p[1].theta, 0, 1, 0); // Do rotation for phalanx 2 with specified angle
			phalanx2Hand -> draw();

			// CONSTANT: Rotate to third vertex
			glRotatef(-90.0f, 0, 1, 0);
			
			// CONSTANT: Move to third vertex
			glTranslatef(2.0f, 0.0f, 0.0f);
			
			glRotatef(f_p[2].theta, 0, 1, 0); // Do rotation for phalanx 3 with specified angle
			phalanx3Hand -> draw();
			
			// CONSTANT: Return to origin
			// Undo rotations and translations in proper order
			glRotatef((-1.0f*f_p[2].theta), 0, 1, 0); // Phalanx 3 rotation
			glTranslatef(-2.0f, 0.0f, 0.0f); // Phalanx 3 translation
			glRotatef(90.0f, 0, 1, 0); // Rotation to Phalanx 3
			glRotatef((-1.0f*f_p[1].theta), 0, 1, 0); // Phalanx 2
			glTranslatef(-1.3f, -0.03f, 0.0f); // Phalanx 2 translation and adjustment
			glRotatef((-1.0f*f_p[0].theta), 0, 0, 1); // Rotation to Phalanx 1
			glTranslatef(0.0f, 0.0f, -1.87f); // Phalanx 1 translation
			// --------------------------------------------------------------------------------------

			// CONSTANT: Keep parts right side up
			glRotatef(90.0f, 0, 0, 1);
			
			// --------------------------------------------------------------------------------------
			// DRAW FINGER 2
					
			// CONSTANT: Rotate 60 degrees because of palm orientation
			glRotatef(60.0f, 0, 1, 0);
			// CONSTANT: Rotate part 180 degrees to match prototype
			glRotatef(180.0f, 1, 0, 0);

			// CONSTANT: Move to first vertex
			glTranslatef(0.0f, 0.6f, 1.6f);
			glRotatef((f_p[4].theta-90.0f), 0, 0, 1); // Do rotation for phalanx 1 with specified angle
			glColor3f(205.0f/255.0f, 51.0f/255.0f, 51.0f/255.0f); // Use proper color - red
			phalanx1Hand -> draw();

			// CONSTANT: Necessary adjustment because of model file offset
			glTranslatef(0.0f, 0.03f, 0.0f);
						
			// CONSTANT: Move to second vertex
			glTranslatef(1.3f, 0.0f, 0.0f);
			glRotatef(f_p[5].theta, 0, 1, 0); // Do rotation for phalanx 2 with specified angle
			phalanx2Hand -> draw();

			// CONSTANT: Rotate to third vertex
			glRotatef(-90.0f, 0, 1, 0);

			// CONSTANT: Move to third vertex
			glTranslatef(2.0f, 0.0f, 0.0f);
			
			glRotatef(f_p[6].theta, 0, 1, 0); // Do rotation for phalanx 3 with specified angle
			phalanx3Hand -> draw();
			
			// CONSTANT: Return to origin
			// Undo rotations and translations in proper order
			glRotatef((-1.0f*f_p[6].theta), 0, 1, 0); // Phalanx 3 rotation
			glTranslatef(-2.0f, 0.0f, 0.0f); // Phalanx 3 translation
			glRotatef(90.0f, 0, 1, 0); // Rotation to Phalanx 3
			glRotatef((-1.0f*f_p[5].theta), 0, 1, 0); // Phalanx 2
			glTranslatef(-1.3f, -0.03f, 0.0f); // Phalanx 2 translation and adjustment
			glRotatef((-1.0f*(f_p[4].theta-90.0f)), 0, 0, 1); // Rotation to Phalanx 1
			glTranslatef(0.2f, -0.6f, -1.6f); // Phalanx 1 translation
			// --------------------------------------------------------------------------------------
			// --------------------------------------------------------------------------------------
			// DRAW FINGER 3

			// Move to next finger - FINGER 3
			glTranslatef(-0.2f, -0.6f, 1.6f);
			glColor3f((34.0f/255.0f), (139.0f/255.0f), (34.0f/255.0f)); // Use proper color - green
			glRotatef((f_p[8].theta-90.0f), 0, 0, 1); // Do rotation for phalanx 1 with specified angle
			phalanx1Hand -> draw();

			// CONSTANT: Necessary adjustment because of model file offset
			glTranslatef(0.0f, 0.03f, 0.0f);
						
			// CONSTANT: Move to second vertex
			glTranslatef(1.3f, 0.0f, 0.0f);
			glRotatef(f_p[9].theta, 0, 1, 0); // Do rotation for phalanx 2 with specified angle
			phalanx2Hand -> draw();

			// CONSTANT: Rotate to third vertex
			glRotatef(-90.0f, 0, 1, 0);

			// CONSTANT: Move to third vertex
			glTranslatef(2.0f, 0.0f, 0.0f);
			
			glRotatef(f_p[10].theta, 0, 1, 0); // Do rotation for phalanx 3 with specified angle
			phalanx3Hand -> draw();
			
			// CONSTANT: Return to origin
			// Undo rotations and translations in proper order
			glRotatef((-1.0f*f_p[10].theta), 0, 1, 0); // Phalanx 3 rotation
			glTranslatef(-2.0f, 0.0f, 0.0f); // Phalanx 3 translation
			glRotatef(90.0f, 0, 1, 0); // Rotation to Phalanx 3
			glRotatef((-1.0f*f_p[9].theta), 0, 1, 0); // Phalanx 2
			glTranslatef(-1.3f, -0.03f, 0.0f); // Phalanx 2 translation and adjustment
			glRotatef((-1.0f*(f_p[8].theta-90.0f)), 0, 0, 1); // Rotation to Phalanx 1
			glTranslatef(0.2f, -0.6f, -1.6f); // Phalanx 1 translation

		glPopMatrix();
			
		return TRUE;
		}

int OpenGLForm::COpenGL::DrawGLSceneSelect(void)
		{
			glLoadIdentity();
			
			/*gluLookAt(eyex+3.0f, eyey+3.0f, eyez+3.0f, focusx, focusy, focusz, 0,1,0);*/
			gluLookAt(eyex , eyey , eyez , focusx, focusy, focusz, 0, 1, 0);
			glPushMatrix();
						
			glRotatef(g_xRotation, 0,1,0); // Rotate with mouse
			glRotatef(g_yRotation, 1,0,0);

			// --------------------------------------------------------------------------------------
			// DRAW PALM
			glColor3f(1.0f, 1.0f, 1.0f); // Use proper color - white
			palmHand -> draw();
			// --------------------------------------------------------------------------------------
			// --------------------------------------------------------------------------------------
			// DRAW FINGER 1
			// CONSTANT: Move to first vertex
			glTranslatef(0.0f, 0.0f, 1.87f); 
									
			glRotatef(f_p[0].theta, 0, 0, 1); // Do rotation for phalanx 1 with specified angle
			glColor3f(0.0f, 0.0f, 1.0f); // Use proper color - blue
			glLoadName(1);
			phalanx1Hand -> draw(); 
						
			// CONSTANT: Necessary adjustment because of model file offset (nobody's perfect, okay?)
			glTranslatef(0.0f, 0.03f, 0.0f);
						
			// CONSTANT: Move to second vertex
			glTranslatef(1.3f, 0.0f, 0.0f); 
			
			glRotatef(f_p[1].theta, 0, 1, 0); // Do rotation for phalanx 2 with specified angle
			glLoadName(2);
			phalanx2Hand -> draw();

			// CONSTANT: Rotate to third vertex
			glRotatef(-90.0f, 0, 1, 0);
			
			// CONSTANT: Move to third vertex
			glTranslatef(2.0f, 0.0f, 0.0f);
			
			glRotatef(f_p[2].theta, 0, 1, 0); // Do rotation for phalanx 3 with specified angle
			glLoadName(3);
			phalanx3Hand -> draw();
			
			// CONSTANT: Return to origin
			// Undo rotations and translations in proper order
			glRotatef((-1.0f*f_p[2].theta), 0, 1, 0); // Phalanx 3 rotation
			glTranslatef(-2.0f, 0.0f, 0.0f); // Phalanx 3 translation
			glRotatef(90.0f, 0, 1, 0); // Rotation to Phalanx 3
			glRotatef((-1.0f*f_p[1].theta), 0, 1, 0); // Phalanx 2
			glTranslatef(-1.3f, -0.03f, 0.0f); // Phalanx 2 translation and adjustment
			glRotatef((-1.0f*f_p[0].theta), 0, 0, 1); // Rotation to Phalanx 1
			glTranslatef(0.0f, 0.0f, -1.87f); // Phalanx 1 translation
			// --------------------------------------------------------------------------------------

			// CONSTANT: Keep parts right side up
			glRotatef(90.0f, 0, 0, 1);
			
			// --------------------------------------------------------------------------------------
			// DRAW FINGER 2
					
			// CONSTANT: Rotate 60 degrees because of palm orientation
			glRotatef(60.0f, 0, 1, 0);
			// CONSTANT: Rotate part 180 degrees to match prototype
			glRotatef(180.0f, 1, 0, 0);

			// CONSTANT: Move to first vertex
			glTranslatef(0.0f, 0.6f, 1.45f);
			glRotatef((f_p[4].theta-90.0f), 0, 0, 1); // Do rotation for phalanx 1 with specified angle
			glColor3f(1.0f, 0.0f, 0.0f); // Use proper color - red
			glLoadName(5);
			phalanx1Hand -> draw();

			// CONSTANT: Necessary adjustment because of model file offset
			glTranslatef(0.0f, 0.03f, 0.0f);
						
			// CONSTANT: Move to second vertex
			glTranslatef(1.3f, 0.0f, 0.0f);
			glRotatef(f_p[5].theta, 0, 1, 0); // Do rotation for phalanx 2 with specified angle
			glLoadName(6);
			phalanx2Hand -> draw();

			// CONSTANT: Rotate to third vertex
			glRotatef(-90.0f, 0, 1, 0);

			// CONSTANT: Move to third vertex
			glTranslatef(2.0f, 0.0f, 0.0f);
			
			glRotatef(f_p[6].theta, 0, 1, 0); // Do rotation for phalanx 3 with specified angle
			glLoadName(7);
			phalanx3Hand -> draw();
			
			// CONSTANT: Return to origin
			// Undo rotations and translations in proper order
			glRotatef((-1.0f*f_p[6].theta), 0, 1, 0); // Phalanx 3 rotation
			glTranslatef(-2.0f, 0.0f, 0.0f); // Phalanx 3 translation
			glRotatef(90.0f, 0, 1, 0); // Rotation to Phalanx 3
			glRotatef((-1.0f*f_p[5].theta), 0, 1, 0); // Phalanx 2
			glTranslatef(-1.3f, -0.03f, 0.0f); // Phalanx 2 translation and adjustment
			glRotatef((-1.0f*(f_p[4].theta-90.0f)), 0, 0, 1); // Rotation to Phalanx 1
			glTranslatef(0.2f, -0.6f, -1.6f); // Phalanx 1 translation
			// --------------------------------------------------------------------------------------
			// --------------------------------------------------------------------------------------
			// DRAW FINGER 3

			// Move to next finger - FINGER 3
			glTranslatef(-0.2f, -0.6f, 1.6f);
			glColor3f(0.0f, 1.0f, 0.0f); // Use proper color - green
			glRotatef((f_p[8].theta-90.0f), 0, 0, 1); // Do rotation for phalanx 1 with specified angle
			glLoadName(9);
			phalanx1Hand -> draw();

			// CONSTANT: Necessary adjustment because of model file offset
			glTranslatef(0.0f, 0.03f, 0.0f);
						
			// CONSTANT: Move to second vertex
			glTranslatef(1.3f, 0.0f, 0.0f);
			glRotatef(f_p[9].theta, 0, 1, 0); // Do rotation for phalanx 2 with specified angle
			glLoadName(10);
			phalanx2Hand -> draw();

			// CONSTANT: Rotate to third vertex
			glRotatef(-90.0f, 0, 1, 0);

			// CONSTANT: Move to third vertex
			glTranslatef(2.0f, 0.0f, 0.0f);
			
			glRotatef(f_p[10].theta, 0, 1, 0); // Do rotation for phalanx 3 with specified angle
			glLoadName(11);
			phalanx3Hand -> draw();
			
			// CONSTANT: Return to origin
			// Undo rotations and translations in proper order
			glRotatef((-1.0f*f_p[10].theta), 0, 1, 0); // Phalanx 3 rotation
			glTranslatef(-2.0f, 0.0f, 0.0f); // Phalanx 3 translation
			glRotatef(90.0f, 0, 1, 0); // Rotation to Phalanx 3
			glRotatef((-1.0f*f_p[9].theta), 0, 1, 0); // Phalanx 2
			glTranslatef(-1.3f, -0.03f, 0.0f); // Phalanx 2 translation and adjustment
			glRotatef((-1.0f*(f_p[8].theta-90.0f)), 0, 0, 1); // Rotation to Phalanx 1
			glTranslatef(0.2f, -0.6f, -1.6f); // Phalanx 1 translation

		glPopMatrix();
		
		return TRUE;
		}

void OpenGLForm::COpenGL::SwapOpenGLBuffers(void)
		{
			SwapBuffers(m_hDC) ;
		}

GLint OpenGLForm::COpenGL::MySetPixelFormat(HDC hdc)
	{
		static	PIXELFORMATDESCRIPTOR pfd=				// pfd Tells Windows How We Want Things To Be
			{
				sizeof(PIXELFORMATDESCRIPTOR),				// Size Of This Pixel Format Descriptor
				1,											// Version Number
				PFD_DRAW_TO_WINDOW |						// Format Must Support Window
				PFD_SUPPORT_OPENGL |						// Format Must Support OpenGL
				PFD_DOUBLEBUFFER,							// Must Support Double Buffering
				PFD_TYPE_RGBA,								// Request An RGBA Format
				16,										// Select Our Color Depth
				0, 0, 0, 0, 0, 0,							// Color Bits Ignored
				0,											// No Alpha Buffer
				0,											// Shift Bit Ignored
				0,											// No Accumulation Buffer
				0, 0, 0, 0,									// Accumulation Bits Ignored
				16,											// 16Bit Z-Buffer (Depth Buffer)  
				0,											// No Stencil Buffer
				0,											// No Auxiliary Buffer
				PFD_MAIN_PLANE,								// Main Drawing Layer
				0,											// Reserved
				0, 0, 0										// Layer Masks Ignored
			};
			
		GLint  iPixelFormat; 
		 
		// get the device context's best, available pixel format match 
		if((iPixelFormat = ChoosePixelFormat(hdc, &pfd)) == 0)
		{
			MessageBox::Show("ChoosePixelFormat Failed");
			return 0;
		}
			 
		// make that match the device context's current pixel format 
		if(SetPixelFormat(hdc, iPixelFormat, &pfd) == FALSE)
		{
			MessageBox::Show("SetPixelFormat Failed");
			return 0;
		}

		if((m_hglrc = wglCreateContext(m_hDC)) == NULL)
		{
			MessageBox::Show("wglCreateContext Failed");
			return 0;
		}

		if((wglMakeCurrent(m_hDC, m_hglrc)) == NULL)
		{
			MessageBox::Show("wglMakeCurrent Failed");
			return 0;
		}


		return 1;
	}

bool OpenGLForm::COpenGL::InitGL(GLvoid)										// All setup for opengl goes here
		{
			glShadeModel(GL_SMOOTH);							// Enable smooth shading
			glClearColor(0.0f, 0.0f, 0.0f, 0.0f);			    // Black background
			glClearDepth(1.0f);		
			
			//initialize the mouse state
			MouseState.leftButton = MouseState.rightButton = MouseState.middleButton = false;
			MouseState.x = MouseState.y = 0;

			// init our eye location
			eyex = 0;
			eyey = 0;
			eyez = 5;
			focusx = focusy = focusz = 0.0f;// Depth buffer setup

			glEnable(GL_DEPTH_TEST);							// Enables depth testing
			glDepthFunc(GL_LEQUAL);	
			// The type of depth testing to do
			glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Really nice perspective calculations
			return TRUE;										// Initialisation went ok
		}

GLvoid OpenGLForm::COpenGL::ReSizeGLScene(GLsizei width, GLsizei height)		// Resize and initialise the gl window
		{
			if (height==0)										// Prevent A Divide By Zero By
			{
				height=1;										// Making Height Equal One
			}

			glViewport(0,0,width,height);						// Reset The Current Viewport

			glMatrixMode(GL_PROJECTION);						// Select The Projection Matrix
			glLoadIdentity();		
			
			// Calculate The Aspect Ratio Of The Window
			gluPerspective(60.0f,785.0f/528.0f,0.1f,1000.0f);
			//gluPerspective(60.0f, 785.0f / 528.0f, 0.1f, 1000.0f);
			//gluPerspective(90.0, (GLfloat)width / (GLfloat)height, 0.1f, 50.0f);

			glMatrixMode(GL_MODELVIEW);							// Select The Modelview Matrix
			glLoadIdentity();									// Reset The Modelview Matrix
		}

GLvoid OpenGLForm::COpenGL::KILLGLWindow(GLvoid)
{
}

GLvoid OpenGLForm::COpenGL::updateRotations(GLvoid){
	// calculate a delta in movement
	int yDelta = MouseState.old_y - MouseState.y;
	int xDelta = MouseState.old_x - MouseState.x;

	// when we need to rotate (only the left button is down)
	if(MouseState.rightButton && !MouseState.leftButton && !MouseState.middleButton)
	{
		// rotate by the delta
		g_xRotation -= xDelta * DEGREES_PER_PIXEL;
		g_yRotation -= yDelta * DEGREES_PER_PIXEL;
	}
	// if we need to move translate (left and right buttons are down
	//else if(MouseState.leftButton && MouseState.rightButton && !MouseState.middleButton)
	//{
	//	// move our eye
	//	eyex += xDelta * UNITS_PER_PIXEL;
	//	eyey -= yDelta * UNITS_PER_PIXEL;

	//	// move our focus point
	//	focusx += xDelta * UNITS_PER_PIXEL;
	//	focusy -= yDelta * UNITS_PER_PIXEL;
	//}
}

GLvoid OpenGLForm::COpenGL::selectGL(int x, int y)
{
	GLuint buffer[16] = {0};
 	GLint hits, view[4];
 		
	// Establish a buffer for selection mode values
	glSelectBuffer(16, buffer);

	// Get viewport
	glGetIntegerv(GL_VIEWPORT, view);

	// Switch to SELECT
	glRenderMode(GL_SELECT);

	// Clear names stack
	glInitNames();
	// Add one name (or error occurs)
	glPushName(0);

	// 
	glMatrixMode(GL_PROJECTION);
 	glPushMatrix();
 		glLoadIdentity();
 
		// Draw area around cursor only
 		gluPickMatrix(x, y, 1.0, 1.0, view);
 		gluPerspective(60.0f,785.0f/528.0f,0.1f,1000.0f);
 
		// Draw objects onto screen
 		glMatrixMode(GL_MODELVIEW);
 
		// Draw objects in names buffer
 		SwapOpenGLBuffers();
 		DrawGLSceneSelect();
 
		// do pushmatrix in Projection mode
 		glMatrixMode(GL_PROJECTION);
 	glPopMatrix();

	// Gets number of elements drawn
	hits = glRenderMode(GL_RENDER);
	if (!FLAG_PHALANX_BUSY){
		list_hits(hits, buffer);}
	update_rotation_phalanx();
 
 	glMatrixMode(GL_MODELVIEW);
	DrawGLScene();

}

GLvoid OpenGLForm::COpenGL::update_rotation_phalanx()
{
	// calculate a delta in movement
	double yDelta = MouseState.old_y - MouseState.y; // was int
	double xDelta = MouseState.old_x - MouseState.x;

	// when we need to rotate (only the left button is down)
	if(MouseState.leftButton && !MouseState.rightButton && !MouseState.middleButton)
	{
		// rotate by the delta
		xDelta += xDelta/10000.0f;
		yDelta -= yDelta/10000.0f;
		g_xViewRot = (float)(sqrt(pow((double)xDelta, 2) + pow((double)yDelta, 2)));
		xDelta = MouseState.old_x - MouseState.x;
		yDelta = MouseState.old_y - MouseState.y;
		if (xDelta < 0){g_xViewRot = -1.0f * g_xViewRot;}
	}
	updateAngle(MouseState.clicked_element, g_xViewRot);
	update_sliders(MouseState.clicked_element);

}

GLvoid OpenGLForm::COpenGL::list_hits(GLint hits, GLuint *names)
 {
 	/*
 		For each hit in the buffer are allocated 4 bytes:
 		1. Number of hits selected (always one,
 									beacuse when we draw each object
 									we use glLoadName, so we replace the
 									prevous name in the stack)
 		2. Min Z
 		3. Max Z
 		4. Name of the hit (glLoadName)
 	*/

	 int one = names[1];
	 one = names[2];
	 one = names[3];
	System::Windows::Forms::Label ^ lbltest = f_one -> returnlabel2();
	MouseState.clicked_element = (int)names[3] - 1;
	lbltest -> Text = "Number: " + (int)(names[0]) + " Min Z: " + (int)(names[1]) + " Max Z: " +(int)names[2] + " Name on stack: " + MouseState.clicked_element;
	
 }

GLvoid OpenGLForm::COpenGL::update_sliders(GLint vertex)
{	
	f_one -> updateTrackBar(vertex);
}

GLvoid OpenGLForm::COpenGL::update_model_angles(GLint vertex, GLfloat new_angle)
{
	f_p[vertex].theta = new_angle;

}

//GLint OpenGLForm::COpenGL::check_capsule_collision(GLint vertex_one, GLint vertex_two)
//{
//	c_p_v a, b, c, d;
//	
//	// Copy segment one
//	a.x = f_p[vertex_one].x;
//	a.y = f_p[vertex_one].y;
//	a.z = f_p[vertex_one].z;
//	b.x = f_p[vertex_one + 1].x;
//	b.y = f_p[vertex_one + 1].y;
//	b.z = f_p[vertex_one + 1].z;
//
//	// Copy segment two
//	c.x = f_p[vertex_two].x;
//	c.y = f_p[vertex_two].y;
//	c.z = f_p[vertex_two].z;
//	d.x = f_p[vertex_two + 1].x;
//	d.y = f_p[vertex_two + 1].y;
//	d.z = f_p[vertex_two + 1].z;
//
//	// Check for parallel lines
//	// Always same distance, just find that
//
//	// For skew lines
//
//	// Intersecting lines? Pretty obvious
//}