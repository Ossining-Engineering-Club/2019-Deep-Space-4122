#pragma once
#include <memory>
#include "frc/WPIlib.h"
#include <networktables/NetworkTableInstance.h>
#include <algorithm>
#include <vector>
#include "Constants.h"
using namespace std;
typedef shared_ptr<NetworkTable> NetTable;
typedef vector<double> Gvector;				// a vector designed for Grip
class Vision
{
public:
	Vision();
	double GetArea(unsigned int val);	// returns the area
	void Update();
	void Filter();
	double GetX(unsigned int val);		// returns the x value of the tracked image
	double GetY(unsigned int val);		// returns the y value of the tracked image
	double GetHeight(unsigned int val);	// returns the height
	int GetNumContours(); // returns the number of contours
	double GetWidth(unsigned int val);	// returns the width
	~Vision();
private:
	NetTable Table;
	Gvector area;
	Gvector X;
	Gvector Y;
	Gvector Height;
	Gvector Width;
	bool found;
};

