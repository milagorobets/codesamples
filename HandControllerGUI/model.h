#ifndef H_MODEL
#define H_MODEL

#include <stdio.h>
#include <string.h>
#include <windows.h>

#define BITMAP_ID 0x4D42
#define MAX_VERTICES 80000
#define MAX_POLYGONS 80000
#define MAX_NORMALS  80000 

value struct Vertex{
	float x, y, z;
};
value struct Texcoor{
	float u, v;
};
value struct Normal{
	float i, j, k;
};
value struct Triangle{
	int n0, n1, n2;
	int v0, v1, v2;
};	

namespace ModelSpace
{
public ref class Model
{
	public:		

		Model();
		~Model(void);

		//Give the name of the .obj model
		int loadModel(char *mFilename);

		//Draw the model
		void draw();
		
		// Initialize the model's parameters
		void Model_init();
		
	private:

		array<Triangle>^ triangles;
		array<Normal>^ normals;
		array<Vertex>^ vertices;

		// Holds information about number of triangles that define the object:
		int triangleCount; 
		int verticesCount;
		int normalCount;
		int texCount;
		
};
}

#endif
