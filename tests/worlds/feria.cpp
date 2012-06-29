#include "mrcore/mrcore.h"

Face calculateFace(Vector2D ini, Vector2D end, double h)
{
	double width=0, height=0;
	Transformation3D trans;
	
	if(ini.y==end.y)//paralela eje X
	{
		if(ini.x<end.x)
			trans=Transformation3D(ini.x,ini.y,0.0,X_AXIS,PI/2);
		else
			trans=Transformation3D(end.x,end.y,0.0,X_AXIS,PI/2);
		width=ini.distance(end);
		height=h;
	}
	else if(ini.x==end.x)//paralela eje Y
	{
		if(ini.y<end.y)
			trans=Transformation3D(end.x,end.y,0.0,Y_AXIS,-PI/2);
		else
			trans=Transformation3D(ini.x,ini.y,0.0,Y_AXIS,-PI/2);
		height=-ini.distance(end);
		width=h;
	}
	else
	{
		//pared diagonal
	}
	Face ret;
	ret.setBase(trans);
	ret.addVertex(0,0);
	ret.addVertex(width,0);
	ret.addVertex(width,height);
	ret.addVertex(0,height);
	//ret.setColor((float)0.9, (float)0.9, (float)0.5, (float)1);
	return ret;
}

void CreateBox(double height, double deep, double width, Vector2D origin, vector<Face>& box)
{
	box.resize(6);
	box[0].setBase(Transformation3D(origin.x,origin.y,0,X_AXIS,PI/2));
	box[0].addVertex(0.0,0.0);
	box[0].addVertex(0.0,height);
	box[0].addVertex(deep,height);
	box[0].addVertex(deep,0.0);

	box[1]=box[0];
	box[1].setBase(Transformation3D(origin.x,origin.y+width,0,X_AXIS,PI/2));

	box[2].setBase(Transformation3D(origin.x,origin.y,height));
	box[2].addVertex(0.0,0.0);
	box[2].addVertex(0.0,width);
	box[2].addVertex(deep,width);
	box[2].addVertex(deep,0.0);

	box[3].setBase(Transformation3D(origin.x,origin.y,0,Y_AXIS,-PI/2));
	box[3].addVertex(0.0,0.0);
	box[3].addVertex(height,0.0);
	box[3].addVertex(height,width);
	box[3].addVertex(0.0,width);

	box[4]=box[3];
	box[4].setBase(Transformation3D(origin.x+deep,origin.y,0,Y_AXIS,-PI/2));
}

void CreateWorldRobocityFair(string filename)
{
	Face a;
	a.setDefaultColor(0.9,0.9,0.2,1.0);
	unsigned int i=0;
	World world;
	FaceSetPart *building=new FaceSetPart;
	double extraDeep=5.0;
	Face ground;
	ground.setBase(Transformation3D(0,0,0));
	ground.addVertex(0,0);
	ground.addVertex(20.66,0);
	ground.addVertex(20.66,12.4);
	ground.addVertex(20.66+extraDeep,12.4);
	ground.addVertex(20.66+extraDeep,15.76);
	ground.addVertex(20.66,15.76);
	ground.addVertex(20.66,25.21);
	ground.addVertex(20.66+extraDeep,25.21);
	ground.addVertex(20.66+extraDeep,26.93);
	ground.addVertex(20.66,26.93);
	ground.addVertex(20.66,28.08);
	ground.addVertex(19.68,28.08);
	ground.addVertex(19.68,28.08+extraDeep);
	ground.addVertex(17.94,28.08+extraDeep);
	ground.addVertex(17.94,28.08);
	ground.addVertex(0,28.08);
	ground.addVertex(0,17.05);
	ground.addVertex(-extraDeep,17.05);
	ground.addVertex(-extraDeep,11.01);
	ground.addVertex(0,11.01);
	ground.setColor((float)0.6, (float)0.4, (float)0.4, 1);
	building->addFace(ground);

	double height=3.0;
	vector<Face> walls;
	for(int i=0;i<ground.getNumVertex()-1;i++)
	{
		Vector2D ini(ground.getAbsoluteVertex(i).x,ground.getAbsoluteVertex(i).y);
		Vector2D fin(ground.getAbsoluteVertex(i+1).x,ground.getAbsoluteVertex(i+1).y);
		walls.push_back(calculateFace(ini,fin,height));
	}
	int last=ground.getNumVertex();
	Vector2D ini(ground.getAbsoluteVertex(last-1).x,ground.getAbsoluteVertex(last-1).y);
	Vector2D fin(ground.getAbsoluteVertex(0).x,ground.getAbsoluteVertex(0).y);
	walls.push_back(calculateFace(ini,fin,height));

	for(int i=0;i<walls.size();i++)
		building->addFace(walls[i]);

	double columnSize=0.6;
	a.setDefaultColor(0.4,0.4,0.4,1.0);
	vector<vector<Face>> columnsList(12);
	CreateBox(height, columnSize, columnSize, Vector2D(3.80,9.34), columnsList[0]);
	CreateBox(height, columnSize, columnSize, Vector2D(3.80,4.22), columnsList[1]);
	CreateBox(height, columnSize, columnSize, Vector2D(7.94,4.22), columnsList[2]);
	CreateBox(height, columnSize, columnSize, Vector2D(12.12,4.22), columnsList[3]);
	CreateBox(height, columnSize, columnSize, Vector2D(16.26,4.22), columnsList[4]);
	CreateBox(height, columnSize, columnSize, Vector2D(16.26,9.34), columnsList[5]);
	CreateBox(height, columnSize, columnSize, Vector2D(16.26,18.14), columnsList[6]);
	CreateBox(height, columnSize, columnSize, Vector2D(16.26,23.28), columnsList[7]);
	CreateBox(height, columnSize, columnSize, Vector2D(7.94,23.28), columnsList[8]);
	CreateBox(height, columnSize, columnSize, Vector2D(12.12,23.28), columnsList[9]);
	CreateBox(height, columnSize, columnSize, Vector2D(3.80,23.28), columnsList[10]);
	CreateBox(height, columnSize, columnSize, Vector2D(3.80,18.18), columnsList[11]);
	for(int i=0;i<columnsList.size();i++)
	{
		for(int j=0;j<columnsList[0].size();j++)
		{
			building->addFace(columnsList[i][j]);
		}
	}

	vector<Face> mueble;
	a.setDefaultColor(0.6,0.6,0.2,1.0);
	CreateBox(2.0,4.50,1.0,Vector2D(7.6,0.0),mueble);
	for(i=0;i<mueble.size();i++)
		building->addFace(mueble[i]);

	a.setDefaultColor(1.0,1.0,1.0,1.0);
	double wallHeight=3.0, tableHeight=1.2;
	
	vector<Face> paredes;
	paredes.push_back(calculateFace(Vector2D(7.60,0), Vector2D(7.60,3.95), wallHeight));
	paredes.push_back(calculateFace(Vector2D(12.2,0), Vector2D(12.2,3.95), wallHeight));
	paredes.push_back(calculateFace(Vector2D(20.66,7.08), Vector2D(16.86,7.08), wallHeight));
	paredes.push_back(calculateFace(Vector2D(20.66,9.34), Vector2D(16.86,9.34), wallHeight));
	paredes.push_back(calculateFace(Vector2D(12.72,28.08), Vector2D(12.72,24), wallHeight));
	paredes.push_back(calculateFace(Vector2D(9.72,28.08), Vector2D(9.72,24), wallHeight));
	paredes.push_back(calculateFace(Vector2D(0,20.58), Vector2D(4.4,20.58), wallHeight));
	paredes.push_back(calculateFace(Vector2D(8.55,11.97), Vector2D(6.55,11.97), wallHeight));
	paredes.push_back(calculateFace(Vector2D(8.55,15.97), Vector2D(6.55,15.97), wallHeight));
	paredes.push_back(calculateFace(Vector2D(14.72,11.97), Vector2D(12.72,11.97), wallHeight));
	paredes.push_back(calculateFace(Vector2D(14.72,15.97), Vector2D(12.72,15.97), wallHeight));
	paredes.push_back(calculateFace(Vector2D(8.55,9.97), Vector2D(12.72,9.97), wallHeight));
	paredes.push_back(calculateFace(Vector2D(8.55,18.09), Vector2D(12.72,18.09), wallHeight));
	paredes.push_back(calculateFace(Vector2D(8.55,9.97), Vector2D(8.55,18.09), wallHeight));
	paredes.push_back(calculateFace(Vector2D(12.72,9.97), Vector2D(12.72,18.09), wallHeight));
	for(i=0;i<paredes.size();i++)
		building->addFace(paredes[i]);

	
	a.setDefaultColor((float)0.9, (float)0.9, (float)0.5, (float)0.5);
	double tableWidth=1.0, tableDeep=0.5;
	vector<vector<Face>> tableList(13);
	CreateBox(tableHeight, tableWidth, tableDeep, Vector2D(8.54,4.22), tableList[0]);
	CreateBox(tableHeight, tableWidth, tableDeep, Vector2D(14.12,4.22), tableList[1]);
	CreateBox(tableHeight, tableDeep, tableWidth, Vector2D(17.86,8.34), tableList[2]);
	CreateBox(tableHeight, tableWidth, tableDeep, Vector2D(17.86,11.8), tableList[3]);
	CreateBox(tableHeight, tableDeep, tableWidth, Vector2D(16.26,17.14), tableList[4]);
	CreateBox(tableHeight, tableWidth, tableDeep, Vector2D(15.26,23.88), tableList[5]);
	CreateBox(tableHeight, tableWidth, tableDeep, Vector2D(9.7,23.88), tableList[6]);
	CreateBox(tableHeight, tableDeep, tableWidth, Vector2D(6.4,26), tableList[7]);
	CreateBox(tableHeight, tableDeep, tableWidth, Vector2D(3.3,20.9), tableList[8]);
	CreateBox(tableHeight, tableWidth, tableDeep, Vector2D(2.8,18.18), tableList[9]);
	CreateBox(tableHeight, tableDeep, tableWidth, Vector2D(6.55,7.97), tableList[10]);
	CreateBox(tableHeight, tableWidth, tableDeep, Vector2D(6.55,19.59), tableList[11]);
	CreateBox(tableHeight, tableWidth, tableDeep, Vector2D(10.14,19.59), tableList[12]);
	for(int i=0;i<tableList.size();i++)
	{
		for(int j=0;j<tableList[0].size();j++)
		{
			building->addFace(tableList[i][j]);
		}
	}

	world+=building;
	StreamFile myfile(filename,false);
	myfile.write(&world);
}