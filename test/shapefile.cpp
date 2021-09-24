#include "shapefil.h"
using namespace std;
int main() {
	SHPHandle fp = SHPCreate("lane.shp", SHPT_ARCZ);
	double x[] = { 1,2 };
	double y[] = { 2,3 };
	double z[] = { 3,4 };
	SHPObject* obj = new SHPObject;
	obj = SHPCreateSimpleObject(SHPT_ARCZ, 2, x, y, z);
	SHPWriteObject(fp, -1, obj);
}