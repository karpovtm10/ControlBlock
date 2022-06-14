#include <math.h>
#include "Convertor.h"


const double Pi = 3.14159265358979; // Число Пи
const double ro = 206264.8062; // Число угловых секунд в радиане

// Эллипсоид Красовского
const double aP = 6378245; // Большая полуось
const double alP = 1 / 298.3; // Сжатие
const double e2P = 2 * alP - alP * alP; // Квадрат эксцентриситета

// Эллипсоид WGS84 (GRS80, эти два эллипсоида сходны по большинству параметров)
const double aW = 6378137; // Большая полуось
const double alW = 1 / 298.257223563; // Сжатие
const double e2W = 2 * alW - alW * alW; // Квадрат эксцентриситета

// Вспомогательные значения для преобразования эллипсоидов
const double a = (aP + aW) / 2;
const double e2 = (e2P + e2W) / 2;
const double da = aW - aP;
const double de2 = e2W - e2P;

// Линейные элементы трансформирования, в метрах
const double dx = 23.92;
const double dy = -141.27;
const double dz = -80.9;

// Угловые элементы трансформирования, в секундах
const double wx = 0;
const double wy = -0.35;
const double wz = -0.82;

// Дифференциальное различие масштабов
const double ms = -0.12e-6;

//*********************************************************************************************************************************/
// Функции преобразования между WGS84 и CK42
double dB(double Bd, double Ld, double H)
{
    double B, L, M, N;
    B = Bd * Pi / 180;
    L = Ld * Pi / 180;
    M = a * (1 - e2) / pow((1 - e2 * pow(sin(B) , 2)) , 1.5);
    N = a * pow((1 - e2 * pow(sin(B) , 2)) , -0.5);
    return 	ro / (M + H) * (N / a * e2 * sin(B) * cos(B) * da + ((N * N) / (a * a) + 1) * N * sin(B) * cos(B) * de2 / 2 
			- (dx * cos(L) + dy * sin(L)) * sin(B) + dz * cos(B)) - wx * sin(L) * (1 + e2 * cos(2 * B)) + wy * cos(L) 
			* (1 + e2 * cos(2 * B)) - ro * ms * e2 * sin(B) * cos(B);
}

double dL(double Bd, double Ld, double H)
{
    double  B, L, N;
    B = Bd * Pi / 180;
    L = Ld * Pi / 180;
    N = a * pow((1 - e2 * pow(sin(B) , 2)) , -0.5);
    return ro / ((N + H) * cos(B)) * (-dx * sin(L) + dy * cos(L)) + tan(B) * (1 - e2) * (wx * cos(L) + wy * sin(L)) - wz;
}

double WGS84Alt(double Bd, double Ld, double H)
{
    double B, L, N, dH;
    B = Bd * Pi / 180;
    L = Ld * Pi / 180;
    N = a * pow((1 - e2 * pow(sin(B) , 2)) , -0.5);
    dH = -a / N * da + N * pow(sin(B) , 2) * de2 / 2 + (dx * cos(L) + dy * sin(L)) * cos(B) + dz * sin(B) - N * e2 * sin(B) * cos(B) * (wx / ro * sin(L) - wy / ro * cos(L)) + ((a * a) / N + H) * ms;
    return H + dH;
}

double WGS84_SK42_Lat(double Bd, double Ld, double H)
{
    return Bd - dB(Bd, Ld, H) / 3600;
}

double SK42_WGS84_Lat(double Bd, double Ld, double H)
{
    return Bd + dB(Bd, Ld, H) / 3600;
}

double WGS84_SK42_Long(double Bd, double Ld, double H)
{
    return Ld - dL(Bd, Ld, H) / 3600;
}

double SK42_WGS84_Long(double Bd, double Ld, double H)
{
    return Ld + dL(Bd, Ld, H) / 3600;
}


//*********************************************************************************************************************************/

//*********************************************************************************************************************************/
// Функции преобразования в координатную проекцию Гаусса-Крюгера
double SK42BTOX(double B, double L, double H) 
{
    int No = (6 + L) / 6;
    double Lo = (L - (3 + 6 * (No - 1))) / 57.29577951;
    double Bo = B * Pi / 180;
    double Xa = pow(Lo , 2) * (109500 - 574700 * pow(sin(Bo) , 2) + 863700 * pow(sin(Bo) , 4) - 398600 * pow(sin(Bo) , 6));
    double Xb = pow(Lo , 2) * (278194 - 830174 * pow(sin(Bo) , 2) + 572434 * pow(sin(Bo) , 4) - 16010 * pow(sin(Bo) , 6) + Xa);
    double Xc = pow(Lo , 2) * (672483.4 - 811219.9 * pow(sin(Bo) , 2) + 5420 * pow(sin(Bo) , 4) - 10.6 * pow(sin(Bo) , 6) + Xb);
    double Xd = pow(Lo , 2) * (1594561.25 + 5336.535 * pow(sin(Bo) , 2) + 26.79 * pow(sin(Bo) , 4) + 0.149 * pow(sin(Bo) , 6) + Xc);
    return 6367558.4968 * Bo - sin(Bo * 2) * (16002.89 + 66.9607 * pow(sin(Bo) , 2) + 0.3515 * pow(sin(Bo) , 4) - Xd);
}
double SK42LTOY(double B, double L, double H)
{
    int No = (6 + L) / 6;
    double Lo = (L - (3 + 6 * (No - 1))) / 57.29577951;
    double Bo = B * Pi / 180;
    double Ya = pow(Lo , 2) * (79690 - 866190 * pow(sin(Bo) , 2) + 1730360 * pow(sin(Bo) , 4) - 945460 * pow(sin(Bo) , 6));
    double Yb = pow(Lo , 2) * (270806 - 1523417 * pow(sin(Bo) , 2) + 1327645 * pow(sin(Bo) , 4) - 21701 * pow(sin(Bo) , 6) + Ya);
    double Yc = pow(Lo , 2) * (1070204.16 - 2136826.66 * pow(sin(Bo) , 2) + 17.98 * pow(sin(Bo) , 4) - 11.99 * pow(sin(Bo) , 6) + Yb);
    return (5 + 10 * No) * 100000 + Lo * cos(Bo) * (6378245 + 21346.1415 * pow(sin(Bo) , 2) + 107.159 * pow(sin(Bo) , 4) + 0.5977 * pow(sin(Bo) , 6) + Yc);
}

double SK42XTOB(double X,double Y, double Z)
{
    int No = pow(Y * 10 , -6);
    double Bi = X / 6367558.4968;
    double Bo = Bi + sin(Bi * 2) * (0.00252588685 - 0.0000149186 * pow(sin(Bi) , 2) + 0.00000011904 * pow(sin(Bi) , 4));
    double Zo = (Y - (10 * No + 5) * 100000) / (6378245 * cos(Bo));
    double Ba = Zo * Zo * (0.01672 - 0.0063 * pow(sin(Bo) , 2) + 0.01188 * pow(sin(Bo) , 4) - 0.00328 * pow(sin(Bo) , 6));
    double Bb = Zo * Zo * (0.042858 - 0.025318 * pow(sin(Bo) , 2) + 0.014346 * pow(sin(Bo) , 4) - 0.001264 * pow(sin(Bo) , 6) - Ba);
    double Bc = Zo * Zo * (0.10500614 - 0.04559916 * pow(sin(Bo) , 2) + 0.00228901 * pow(sin(Bo) , 4) - 0.00002987 * pow(sin(Bo) , 6) - Bb);
    double dB = Zo * Zo * sin(Bo * 2) * (0.251684631 - 0.003369263 * pow(sin(Bo) , 2) + 0.000011276 * pow(sin(Bo) , 4) - Bc);
    return (Bo - dB) * 180 / Pi;
}

double SK42YTOL(double X,double Y, double Z)
{
    int No = pow(Y * 10 , -6);
    double Bi = X / 6367558.4968;
    double Bo = Bi + sin(Bi * 2) * (0.00252588685 - 0.0000149186 * pow(sin(Bi) , 2) + 0.00000011904 * pow(sin(Bi) , 4));
    double Zo = (Y - (10 * No + 5) * 100000) / (6378245 * cos(Bo));
    double La = Zo * Zo * (0.0038 + 0.0524 * pow(sin(Bo) , 2) + 0.0482 * pow(sin(Bo) , 4) + 0.0032 * pow(sin(Bo) , 6));
    double Lb = Zo * Zo * (0.01225 + 0.09477 * pow(sin(Bo) , 2) + 0.03282 * pow(sin(Bo) , 4) - 0.00034 * pow(sin(Bo) , 6) - La);
    double Lc = Zo * Zo * (0.0420025 + 0.1487407 * pow(sin(Bo) , 2) + 0.005942 * pow(sin(Bo) , 4) - 0.000015 * pow(sin(Bo) , 6) - Lb);
    double Ld = Zo * Zo * (0.16778975 + 0.16273586 * pow(sin(Bo) , 2) - 0.0005249 * pow(sin(Bo) , 4) - 0.00000846 * pow(sin(Bo) , 6) - Lc);
    double dL = Zo * (1 - 0.0033467108 * pow(sin(Bo) , 2) - 0.0000056002 * pow(sin(Bo) , 4) - 0.0000000187 * pow(sin(Bo) , 6) - Ld);
    return (6 * (No - 0.5) / 57.29577951 + dL) * 180 / Pi;
}

//*********************************************************************************************************************************/


struct Coords_params WGS84ToCK42(struct Coords_params wgs84)
{
	struct Coords_params ck42;
	ck42.latt = WGS84_SK42_Lat(wgs84.latt, wgs84.longt,  wgs84.h);
	ck42.longt = WGS84_SK42_Long(wgs84.latt, wgs84.longt,  wgs84.h);
	ck42.h = wgs84.h;
	return ck42;
}

struct GsKr CK42ToGsKr(struct Coords_params ck42)
{
	struct GsKr gk;
	gk.x = SK42BTOX(ck42.latt, ck42.longt, ck42.h);
	gk.y = SK42LTOY(ck42.latt, ck42.longt, ck42.h);
	gk.h = ck42.h;
	return gk;
}

