#include "mrcore/mrcore.h"
void createBox(double height, double width, double deep, Vector2D origin, vector <Face> &wardrobe)
{
	wardrobe.resize(5);
	wardrobe[0].setBase(Transformation3D(origin.x,origin.y,0,X_AXIS,PI/2));
	wardrobe[0].addVertex(0.0,0.0);
	wardrobe[0].addVertex(0.0,height);
	wardrobe[0].addVertex(deep,height);
	wardrobe[0].addVertex(deep,0.0);

	wardrobe[1]=wardrobe[0];
	wardrobe[1].setBase(Transformation3D(origin.x,origin.y+width,0,X_AXIS,PI/2));

	wardrobe[2].setBase(Transformation3D(origin.x,origin.y,height));
	wardrobe[2].addVertex(0.0,0.0);
	wardrobe[2].addVertex(0.0,width);
	wardrobe[2].addVertex(deep,width);
	wardrobe[2].addVertex(deep,0.0);

	wardrobe[3].setBase(Transformation3D(origin.x,origin.y,0,Y_AXIS,-PI/2));
	wardrobe[3].addVertex(0.0,0.0);
	wardrobe[3].addVertex(height,0.0);
	wardrobe[3].addVertex(height,width);
	wardrobe[3].addVertex(0.0,width);

	wardrobe[4]=wardrobe[3];
	wardrobe[4].setBase(Transformation3D(origin.x+deep,origin.y,0,Y_AXIS,-PI/2));

	wardrobe[0].setColor(0.5,0.5,0.5,1);
	wardrobe[1].setColor(0.5,0.5,0.5,1);
	wardrobe[2].setColor(0.5,0.5,0.5,1);
	wardrobe[3].setColor(0.5,0.5,0.5,1);
	wardrobe[4].setColor(0.5,0.5,0.5,1);
}
void CreateEmptyWorld(string filename)
{
	World world;
	FaceSetPart *building=new FaceSetPart;
	Face ground;
	ground.setBase(Transformation3D(0,0,0));
	ground.addVertex(0,0);
	ground.addVertex(20.0,0);
	ground.addVertex(20.0,20.0);
	ground.addVertex(0,20.0);
	ground.setColor(0.6f, 0.4f, 0.4f, 1.0f);
	building->addFace(ground);

	Face wall1;
	wall1.setBase(Transformation3D(0,0,0,X_AXIS,PI/2));
	wall1.addVertex(0,0);
	wall1.addVertex(20.0,0);
	wall1.addVertex(20.0,2.0);
	wall1.addVertex(0,2.0);
	wall1.setColor(0.9f, 0.9f, 0.5f, 1.0f);
	building->addFace(wall1);

	Face wall2(wall1);
	wall2.setBase(Transformation3D(0,20.0,0,X_AXIS,PI/2));
	building->addFace(wall2);

	Face wall3;
	wall3.setBase(Transformation3D(0,0,0,Y_AXIS,-PI/2));
	wall3.addVertex(0,0);
	wall3.addVertex(2.0,0);
	wall3.addVertex(2.0,20.0);
	wall3.addVertex(0,20.0);
	wall3.setColor(0.9f, 0.9f, 0.5f, 1.0f);
	building->addFace(wall3);

	Face wall4(wall3);
	wall4.setBase(Transformation3D(20.0,0,0,Y_AXIS,-PI/2));
	building->addFace(wall4);

	vector<Face> obstacle01,obstacle02;
	//createBox(2.0, 0.5, 0.5, Vector2D(12.5,8.5), obstacle);	//near corner
	//createBox(2.0, 0.5, 0.5, Vector2D(11.25,9.75), obstacle);	//middle interior
	//createBox(2.0, 0.5, 0.5, Vector2D(12.25,9.75), obstacle);	//middle exterior
	//createBox(2.0, 0.5, 0.5, Vector2D(11.75,9.75), obstacle);	//blocking path

	//for reactive example
	//createBox(2.0, 0.5, 0.5, Vector2D(11.25,9.75), obstacle01);
	//createBox(2.0, 0.5, 0.5, Vector2D(9.75,12.25), obstacle02);

	//for replanner example
	createBox(2.0, 0.5, 0.5, Vector2D(11.75,9.75), obstacle01);
	createBox(2.0, 0.5, 0.5, Vector2D(9.75,11.75), obstacle02);
	for(int i=0; i<obstacle01.size(); i++)
	{
		building->addFace(obstacle01.at(i));
		building->addFace(obstacle02.at(i));
	}

	world+=building;
	StreamFile myfile(filename,false);
	myfile.write(&world);
}