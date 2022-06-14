#ifndef __COORDTRANS
#define __COORDTRANS

struct Ellipsoid 
{
    char *name;
    double a;  /* Большая (экваториальная) полуось      */
    double b;  /* Малая (полярная) полуось              */
    double al; /* Сжатие (a-b)/a                        */
    double e2; /* Квадрат эксцентриситета (a^2-b^2)/a^2 */
};

struct HelmertParam 
{
	char *src, *dst;
    struct Ellipsoid *esp;
    struct Ellipsoid *edp;
	struct 
	{
        double dx, dy, dz;
        double wx, wy, wz;
        double ms;
    } p;

	double a,  da;
    double e2, de2;
	double de2__2, dxe2__2;
    double n, n__2e2;
    double wx_2e2__ro, wy_2e2__ro;
    double wx_n__ro, wy_n__ro;
    double wz__ro, ms_e2;
};

struct TransverseMercatorParam {
    struct Ellipsoid *ep;
    double k;           /* Масштабный коэффициент                                 */
    double lat0;        /* Начальная параллель  (в радианах)                      */
    double lon0;        /* Центральный меридиан (в радианах)                      */
    double n0;          /* Условное северное смещение для начальной параллели     */
    double e0;          /* Условное восточное смещение для центрального меридиана */
    double zw;          /* Ширина зоны (в радианах)                               */
    double zs;          /* Условное восточное смещение между зонами               */
    // Вспомогательные величины
    double e2__a2k2, ie2__a2k2, m0, mp, imp;
    double f0, f2, f4, f6;
    double m1, m2, m4, m6;
    double q, q1, q2, q3, q4, q6, q7, q8;
    double q11, q12, q13, q14, q15, q16, q17, q18;
    // Вспомогательные величины - 2
    double apk, n, b, c, d;
    double b1, b2, b3, b4;
};

void Convert_to_MCK(struct HelmertParam params, double lat, double lon, double alt, double *X, double *Y, double *Z);
void setupHelmert(struct HelmertParam *pp);
void translateHelmertInv(struct HelmertParam *pp, double lat, double lon, double h, double *latp, double *lonp);
void setupTransverseMercator(struct TransverseMercatorParam *pp);
void translateTransverseMercator(struct TransverseMercatorParam *pp, int zone, double lat, double lon, double *ep, double *np);
#endif
