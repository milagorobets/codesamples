#include "stdafx.h"
#include "Model.h"
#include <GL/GL.h>
#include <GL/GLU.h>


namespace ModelSpace
{

// Default constructor
Model::Model(){}

// Init function for object
void Model::Model_init()
{
	verticesCount = 0;
	triangleCount = 0;
    normalCount = 0;
	texCount = 0;

	this->triangles = gcnew array<Triangle>(MAX_POLYGONS);
	this->normals = gcnew array<Normal>(MAX_NORMALS);
	this->vertices = gcnew array<Vertex>(MAX_VERTICES);
}

// Destructor
Model::~Model(void)
{
	delete this->triangles;
	delete this->normals;
	delete this->vertices;
}

// Load model from an .obj file
int Model::loadModel(char *mFilename)
{
	FILE *f;
	char buffer[256];
	double temp1, temp2, temp3;
	
	if((f = fopen(mFilename, "r"))==NULL) return 0;

	// Read file until the end
	while(!feof(f))
	{
		memset(buffer,0,255);

		fgets(buffer, 256, f);

		if( strncmp("vn ",buffer,3) == 0 ) // normal
		{
			sscanf((buffer+2),"%f%f%f",
							&(normals[normalCount].i),
							&(normals[normalCount].j),
							&(normals[normalCount].k));
			++normalCount;
		}
		else if(strncmp("v ",buffer,2) == 0 ) // vertex
		{

			sscanf((buffer+1),"%f%f%f",
							&vertices[verticesCount].x,
							&vertices[verticesCount].y,
							&vertices[verticesCount].z);
			++verticesCount;
		}
		else if(strncmp("f ",buffer,2) == 0 )
		{
			sscanf((buffer+1),"%d/%d/%d%d/%d/%d%d/%d/%d",
				&triangles[triangleCount].v0, &temp1, &triangles[triangleCount].n0,
				&triangles[triangleCount].v1, &temp2, &triangles[triangleCount].n1,
				&triangles[triangleCount].v2, &temp3, &triangles[triangleCount].n2);
			++triangleCount;			
		}
		
	}
	fclose(f);
	return 1;
}
// Render the object that was loaded
void Model::draw()
{
	const GLfloat lightfvarray[]= {0.5f,0.5f,0.5f,1.0f};
	
	// Set OpenGL parameters
	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);
	glShadeModel(GL_SMOOTH); // Smooth shading
	glColorMaterial(GL_FRONT, GL_DIFFUSE);
	glEnable(GL_LIGHT0); // Add a light source
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightfvarray);

	glBegin(GL_TRIANGLES); // Display entire object using triangles
	for(int i=0; i<triangleCount; i++)
	{
		glNormal3f( normals[triangles[i].n0].i,
					normals[triangles[i].n0].j,
					normals[triangles[i].n0].k);
		glVertex3f( vertices[triangles[i].v0].x,
					vertices[triangles[i].v0].y,
					vertices[triangles[i].v0].z);

		glNormal3f( normals[triangles[i].n1].i,
					normals[triangles[i].n1].j,
					normals[triangles[i].n1].k);
		glVertex3f( vertices[triangles[i].v1].x,
					vertices[triangles[i].v1].y,
					vertices[triangles[i].v1].z);

		glNormal3f( normals[triangles[i].n2].i,
					normals[triangles[i].n2 ].j,
					normals[triangles[i].n2 ].k);
		glVertex3f( vertices[triangles[i].v2].x,
					vertices[triangles[i].v2].y,
					vertices[triangles[i].v2].z);
	}
	glEnd();
}

}