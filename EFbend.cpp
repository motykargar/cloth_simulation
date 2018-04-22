#include "EFbend.h"

double EFbend(Cloth &cloth, int *tris, double DAMP_STIFF, double BEND_STIFF)
{
	Bend bend(cloth, tris);

	double E = .5 * BEND_STIFF * bend.c *bend.c;
	return E;
}
