#include "CoordTrans.h"
#include "math.h"


double M_pi = 3.1415926535;
double ro = 206264.8062; // количество секунд в 1 радиане

struct Ellipsoid Ellipsoid_WGS84 = 
{
    .name = "WGS84",
    .a  = 6378137.0,
	.b  = 6356752.3142,
    .al = 1.0 / 298.257223563,
};

struct Ellipsoid Ellipsoid_Krasovsky = 
{
    .name = "Krasovsky",
    .a  = 6378245.0,
	.b  = 6356863,
    .al = 1.0 / 298.3,
};

struct HelmertParam Helmert_SK42 = 
{
    // SK42->PZ90->WGS84 (ГОСТ Р 51794-2001)
    .p = {23.92, -141.27, -80.9, 0, -0.35, -0.82, -0.12e-6},
};

struct HelmertParam Helmert_WGS84_SK42 = {
    .src = "WGS84",
    .dst = "SK42",
    .esp = &Ellipsoid_WGS84,
    .edp = &Ellipsoid_Krasovsky,
    // SK42->PZ90->WGS84 (ГОСТ Р 51794-2001)
    .p = {23.92, -141.27, -80.9, 0, -0.35, -0.82, -0.12e-6},
};

struct TransverseMercatorParam TM_GaussKruger = {
    .ep   = &Ellipsoid_Krasovsky,
    .zw   = 6,
    .lon0 = 63,
    .e0   = 5e5,
    .zs   = 1e6,
};

double dB(double Bd, double Ld, double H)
{
	Ellipsoid_WGS84.e2 = 2 * Ellipsoid_WGS84.al - Ellipsoid_WGS84.al * Ellipsoid_WGS84.al;
	Ellipsoid_Krasovsky.e2 = 2 * Ellipsoid_Krasovsky.al - Ellipsoid_Krasovsky.al * Ellipsoid_Krasovsky.al;
	
    double B = 	Bd * M_pi / 180;
    double L = 	Ld * M_pi / 180;
	double M = 	Helmert_SK42.a * (1 - Helmert_SK42.e2) / pow(1 - Helmert_SK42.e2 * sin(B) * sin(B), 2);
    double N = 	Helmert_SK42.a * pow(1 - Helmert_SK42.e2 * sin(B) * sin(B), -0.5);
    return 	( 	
				ro / (M + H) * (N / Helmert_SK42.a * Helmert_SK42.e2 * sin(B) * cos(B) * Helmert_SK42.da	
				+ (N * N / Helmert_SK42.a / Helmert_SK42.a + 1) * N * sin(B) * cos(B) * Helmert_SK42.de2 / 2 
				- (Helmert_SK42.p.dx * cos(L) + Helmert_SK42.p.dy * sin(L)) * sin(B) + Helmert_SK42.p.dz * cos(B)) 
				- Helmert_SK42.p.wx * sin(L) * (1 + Helmert_SK42.e2 * cos(2 * B)) 
				+ Helmert_SK42.p.wy * cos(L) * (1 + Helmert_SK42.e2 * cos(2 * B)) 
				- ro * Helmert_SK42.p.ms * Helmert_SK42.e2 * sin(B) * cos(B)
			);
}

double dL(double Bd, double Ld, double H)
{
    double B = Bd * M_pi / 180;
    double L = Ld * M_pi / 180;
    double N = Helmert_SK42.a * pow(1 - Helmert_SK42.e2 * sin(B) * sin(B), -0.5);
    return 	(
				ro / ((N + H) * cos(B)) * (-Helmert_SK42.p.dx * sin(L) + Helmert_SK42.p.dy * cos(L)) 
				+ tan(B) * (1 - Helmert_SK42.e2) * (Helmert_SK42.p.wx * cos(L) + Helmert_SK42.p.wy * sin(L)) - Helmert_SK42.p.wz
			);
}

double WGS84_SK42_Lat(double Bd, double Ld, double H)
{
	return (Bd - dB(Bd, Ld, H) / 3600);
}

double WGS84_SK42_Lon(double Bd, double Ld, double H)
{
	return (Ld - dL(Bd, Ld, H) / 3600);
}


void Convert_Geo_To_Rect(double lat, double lon, double alt, double *X, double *Y, double *Z)
{
	double e2 = 1 - (Ellipsoid_WGS84.b * Ellipsoid_WGS84.b /Ellipsoid_WGS84.a / Ellipsoid_WGS84.a); // Вычисление квадрата эксцентриситета
	double N = Ellipsoid_WGS84.a / (sqrt(1 - e2 * sin(lat) * sin(lat)));
	
	// Преобразование геодезических координат в прямоугльные пространственные координаты
	*X = (N + alt) * cos(lat) * cos(lon);
	*Y = (N + alt) * cos(lat) * sin(lon);
	*Z = ((1 - e2) * N + alt) * sin(lat);
}

void Helmert_Convert(double X, double Y, double Z, double *Xn, double *Yn, double *Zn)
{
	*Xn = (1 - Helmert_SK42.p.ms) * (X + Helmert_SK42.p.wz * Y - Helmert_SK42.p.wy * Z) + Helmert_SK42.p.dx;
	*Yn = (1 - Helmert_SK42.p.ms) * (-Helmert_SK42.p.wz * X + Y + Helmert_SK42.p.wx * Z) + Helmert_SK42.p.dy;
	*Zn = (1 - Helmert_SK42.p.ms) * (Helmert_SK42.p.wy * X - Helmert_SK42.p.wx * Y + Z) + Helmert_SK42.p.dz;
}
int inite = 1;
void Convert_to_MCK(struct HelmertParam params, double lat, double lon, double alt, double *X, double *Y, double *Z)
{
//	double X_rect, Y_rect, Z_rect;
	double Xcoord, Ycoord, Zcoord;
	double pLat, pLon;
//	Convert_Geo_To_Rect(lat, lon, alt, &X_rect, &Y_rect, &Z_rect);
//	Helmert_Convert(X_rect, Y_rect, Z_rect, &Xcoord, &Ycoord, &Zcoord);
//	
//	*X = Xcoord;
//	*Y = Ycoord;
//	*Z = Zcoord;
////	
////	Helmert_SK42.a = (Ellipsoid_Krasovsky.a + Ellipsoid_WGS84.a) / 2;
////	Helmert_SK42.e2 = (Ellipsoid_Krasovsky.al + Ellipsoid_WGS84.al) / 2;
////	Helmert_SK42.da = Ellipsoid_WGS84.a - Ellipsoid_Krasovsky.a;
////	Helmert_SK42.de2 = Ellipsoid_WGS84.e2 - Ellipsoid_Krasovsky.e2;
////	
////	*X = WGS84_SK42_Lat(lat, lon, alt);
////	*Y = WGS84_SK42_Lon(lat, lon, alt);
	if (inite)
	{
		setupHelmert(&Helmert_WGS84_SK42);
		setupTransverseMercator(&TM_GaussKruger);
		inite = 0;
	}
	
	translateHelmertInv(&Helmert_WGS84_SK42, lat, lon, alt, &pLat, &pLon);
	*X = pLat * 180 / M_pi;
	*Y = pLon * 180 / M_pi;
	
//	Convert_Geo_To_Rect(pLat, pLon, alt, &Xcoord, &Ycoord, &Zcoord);
//	
//	*X = Xcoord;
//	*Y = Ycoord;
	
	translateTransverseMercator(&TM_GaussKruger, 1, pLat, pLon,  &Xcoord, &Ycoord);
	translateTransverseMercator(&TM_GaussKruger, 2, pLat, pLon,  &Xcoord, &Ycoord);
	translateTransverseMercator(&TM_GaussKruger, 3, pLat, pLon,  &Xcoord, &Ycoord);
	translateTransverseMercator(&TM_GaussKruger, 4, pLat, pLon,  &Xcoord, &Ycoord);
	translateTransverseMercator(&TM_GaussKruger, 5, pLat, pLon,  &Xcoord, &Ycoord);
	translateTransverseMercator(&TM_GaussKruger, 6, pLat, pLon,  &Xcoord, &Ycoord);
	translateTransverseMercator(&TM_GaussKruger, 7, pLat, pLon,  &Xcoord, &Ycoord);
	translateTransverseMercator(&TM_GaussKruger, 8, pLat, pLon,  &Xcoord, &Ycoord);
	translateTransverseMercator(&TM_GaussKruger, 9, pLat, pLon,  &Xcoord, &Ycoord);
	translateTransverseMercator(&TM_GaussKruger, 10, pLat, pLon,  &Xcoord, &Ycoord);
	translateTransverseMercator(&TM_GaussKruger, 11, pLat, pLon,  &Xcoord, &Ycoord);
//	Convert_Geo_To_Rect(pLat, pLon, alt, &Xcoord, &Ycoord, &Zcoord);
	

	
//	*X = Xcoord * 180 / M_pi;
//	*Y = Ycoord * 180 / M_pi;

*X = Xcoord;
*Y = Ycoord;
	
}

void setupHelmert(struct HelmertParam *pp) {
	pp->edp->e2 = 2 * pp->edp->al - pp->edp->al * pp->edp->al;
	pp->esp->e2 = 2 * pp->esp->al - pp->esp->al * pp->esp->al;
    pp->a = (pp->edp->a + pp->esp->a) / 2;
    pp->da = pp->edp->a - pp->esp->a;
    pp->e2 = (pp->edp->e2 + pp->esp->e2) / 2;
    pp->de2 = pp->edp->e2 - pp->esp->e2;
    pp->de2__2 = pp->de2 / 2;
    pp->dxe2__2 = pp->de2__2 + pp->e2 * pp->da / pp->a;
    pp->n = 1 - pp->e2;
    pp->n__2e2 = pp->n / pp->e2 / 2;
    pp->wx_2e2__ro = pp->p.wx * pp->e2 * 2 * (1 / ro);
    pp->wy_2e2__ro = pp->p.wy * pp->e2 * 2 * (1 / ro);
    pp->wx_n__ro = pp->p.wx * pp->n * (1 / ro);
    pp->wy_n__ro = pp->p.wy * pp->n * (1 / ro);
    pp->wz__ro = pp->p.wz * (1 / ro);
    pp->ms_e2 = pp->p.ms * pp->e2;
}

void translateHelmertInv(struct HelmertParam *pp, double lat, double lon, double h, double *latp, double *lonp) 
{
    double sin_lat, cos_lat;
    double sin_lon, cos_lon;
    double q, n;
	
	double lat_rad = lat * M_pi / 180;	// Перевод широты в радианы
	double lon_rad = lon * M_pi / 180;	// Перевод долготы в радианы
    
    sin_lat = sin(lat_rad);
    cos_lat = cos(lat_rad);
    sin_lon = sin(lon_rad);
    cos_lon = cos(lon_rad);
    q = 1 / (1 - pp->e2 * sin_lat * sin_lat);
    n = pp->a * sqrt(q);

   *latp = lat_rad
			- ((n * (q * pp->de2__2 + pp->dxe2__2) * sin_lat + pp->p.dz) * cos_lat
			- (pp->p.dx * cos_lon + pp->p.dy * sin_lon) * sin_lat
			) / (n * q * pp->n + h)
			+ (pp->wx_2e2__ro * sin_lon - pp->wy_2e2__ro * cos_lon)
			* (cos_lat * cos_lat + pp->n__2e2)
			+ pp->ms_e2 * sin_lat * cos_lat;
   *lonp = lon_rad
			+ ((pp->p.dx * sin_lon - pp->p.dy * cos_lon) / (n + h)
			- (pp->wx_n__ro * cos_lon + pp->wy_n__ro * sin_lon) * sin_lat) / cos_lat
			+ pp->wz__ro;
}

void setupTransverseMercator(struct TransverseMercatorParam *pp) {
    double sin_lat, cos_lat, cos2_lat;
    double q, n, rk, ak;
	
	pp->zw *= M_pi/180;
	pp->lon0 *= M_pi/180;

    if (!pp->k)
        pp->k = 1.0;
    sin_lat = sin(pp->lat0);
    cos_lat = cos(pp->lat0);
    cos2_lat = cos_lat * cos_lat;
    q = pp->ep->e2 / (1 - pp->ep->e2);
    // Приплюснутость n = (a-b)/(a+b)
    n = (pp->ep->a - pp->ep->b) / (pp->ep->a + pp->ep->b);
    rk = (pp->ep->a + pp->ep->b) * pp->k / 2;
    ak = pp->ep->a * pp->k;
    pp->e2__a2k2  = pp->ep->e2 / (ak * ak);
    pp->ie2__a2k2 = (1 - pp->ep->e2) / (ak * ak);

    pp->f6 = 1097.0/4 * n*n*n*n;
    pp->f4 = (151.0/3 - 3291.0/8 * n) * n*n*n;
    pp->f2 = (21.0/2 + (-151.0/3 + 5045.0/32 * n) * n) * n*n;
    pp->f0 = (3.0 + (-21.0/4 + (31.0/4 - 657.0/64 * n) * n) * n) * n;

    pp->m6 = rk * 315.0/4 * n*n*n*n;
    pp->m4 = rk * (-70.0/3 - 945.0/8 * n) * n*n*n;
    pp->m2 = rk * (15.0/2 + (70.0/3 + 1515.0/32 * n) * n) * n*n;
    pp->m1 = rk * (-3.0 + (-15.0/4 + (-4.0 - 255.0/64 * n) * n) * n) * n;

    // polar distance
    pp->mp = rk * (1.0 + (1.0/4 + 1.0/64 * n*n) * n*n);
    pp->imp = 1 / pp->mp;
    pp->m0 = pp->n0 - pp->mp * pp->lat0 - sin_lat * cos_lat *
        (pp->m1 + (pp->m2 + (pp->m4 + pp->m6 * cos2_lat) * cos2_lat) * cos2_lat);

    pp->q   =                        q;
    pp->q1  =                            1.0/6    * q*q;
    pp->q2  =            3.0/8     * q;
    pp->q3  =            5.0/6     * q;
    pp->q4  =  1.0/6   - 11.0/24   * q;
    pp->q6  =            1.0/6     * q;
    pp->q7  =            3.0/5     * q;
    pp->q8  =  1.0/5   - 29.0/60   * q;
    pp->q11 =          - 5.0/12    * q;
    pp->q12 = -5.0/24  + 3.0/8     * q;
    pp->q13 =                          - 1.0/240  * q*q;
    pp->q14 =            149.0/360 * q;
    pp->q15 = 61.0/720 - 63.0/180  * q;
    pp->q16 =                          - 1.0/40   * q*q;
    pp->q17 =          - 1.0/60    * q;
    pp->q18 = 1.0/24   + 1.0/15    * q;

    // Вспомогательные величины - 2
    double e2 = pp->ep->e2;
    pp->apk = ak * (1 + n*n / 4 + n*n*n*n / 64) / (1 + n);
    pp->n = n;
    pp->b = (5 - e2) * e2 * e2 / 6;
    pp->c = (104 - 45 * e2) * e2 * e2 * e2 / 120;
    pp->d = 1237.0/1260 * e2 * e2 * e2 * e2;
    pp->b1 = (1.0/2 + (-2.0/3 + (5.0/16 + 41.0/180 * n) * n) * n) * n;
    pp->b2 = (13.0/48 + (-3.0/5 + 557.0/1440 * n) * n) * n*n;
    pp->b3 = (61.0/240 - 103.0/140 * n) * n*n*n;
    pp->b3 = 49561.0/161280 * n*n*n*n;
}

void translateTransverseMercator(struct TransverseMercatorParam *pp, int zone,
                double lat, double lon, double *ep, double *np) {
    double lon2, v, m;
    double k4, k6, h3, h5;
    double sin_lat = sin(lat);
    double cos_lat = cos(lat);
    double cos2_lat = cos_lat * cos_lat;

    lon -= zone * pp->zw + pp->lon0;
//    while (unlikely(lon <= -M_PI))
//        lon += 2*M_PI;
    lon2 = lon * lon;

    // Вычисление переменных для преобразования
    v  = 1 / sqrt(pp->e2__a2k2 * cos2_lat + pp->ie2__a2k2);
    m  = ((pp->m6 * cos2_lat + pp->m4) * cos2_lat + pp->m2) * cos2_lat + pp->m1;
    k4 = ((pp->q1 * cos2_lat + pp->q2) * cos2_lat + 1.0/4 ) * cos2_lat - 1.0/24;
    k6 = ((pp->q3 * cos2_lat + pp->q4) * cos2_lat - 1.0/12) * cos2_lat + 1.0/720;
    h3 = ((                    pp->q6) * cos2_lat + 1.0/3 ) * cos2_lat - 1.0/6;
    h5 = ((pp->q7 * cos2_lat + pp->q8) * cos2_lat - 1.0/6 ) * cos2_lat + 1.0/120;

    // Вычисление северного и восточного смещения (в метрах)
    *np = pp->m0 + pp->mp * lat
        + (m + v * ((k6 * lon2 + k4) * lon2 + 0.5) * lon2) * cos_lat * sin_lat;
    *ep = pp->e0 + pp->zs * zone
        + (    v * ((h5 * lon2 + h3) * lon2 + 1.0) * lon ) * cos_lat;
}
