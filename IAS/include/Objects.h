
#ifndef OBJECTS
#define OBJECTS

typedef struct Ball
{
	int
	ballID,
	isObserved;

	double
	radius,
	screenRadius,
	lastSeen,
	pos 		[3],
	screenPos 	[2],
	colorRGB	[3],
	colorHsvMin [3],
	colorHsvMax [3],
	orientation [4];

//	const struct BallPrototype prototype = {
//			0,						//ballID
//			0.f,					//radius
//			0.f,					//screenRadius
//			0.f, 					//lastSeen
//			0.f , 0.f, 0.f,			//pos
//			0.f, 0.f,				//screenPos
//			0.f, 0.f, 0.f, 			//RGB
//			0.f, 0.f, 0.f, 			//HsvMin
//			0.f, 0.f, 0.f, 			//HsvMax
//			1.f, 0.f, 0.f, 0.f		//orientation
//	};

} Ball;

#endif
