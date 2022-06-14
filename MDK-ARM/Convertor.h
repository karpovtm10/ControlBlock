#ifndef __CONVERTOR
#define __CONVERTOR

struct Coords_params
{
	double longt, latt, h;
};

struct GsKr
{
	double x, y, h;
};

struct GsKr CK42ToGsKr(struct Coords_params ck42);
struct Coords_params WGS84ToCK42(struct Coords_params wgs84);

#endif
