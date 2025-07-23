#include <jni.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <numeric>

// uncomment this for release version (remove debugging from code)
//#define RELEASE_VERSION

// for each android device you run this on, you need to figure out the dimensions of the screen
// that is available for the application and then what portion of that is used for video live fragment
// - i.e. what is the size of the border 

// the index in the array is the drone ID (1..6), ignore element at pos. 0

//Image dimensions: (effective always approx. 16:9)
// 1: Samsung A40     2014x996    effective: 1770x996   => border left 122, border right 122 
// 2: SamusungA30     1397x667    effective: 1185x667   => border left 106, border right 106  (canvas.drawBitmap(btmp, 31.0f, 0.0f, paint))       
// 3: GalaxyS8        1980x996    effective: 1770x996   => border left 105, border right 105
// 4: Huawei:         1133x664    effective: 1133x638   => border top   13, border bottom 13 
// 5: Xiaomi:         2079x966    effective: 1717x966   => border left 181, border right 181   (canvas.drawBitmap(btmp, 31.0f, 0.0f, paint))
// 6: Samsung A40     2014x996    effective: 1770x996   => border left 122, border right 122

//    Motorola g20    2400x1800(phone)  2175x1080(app with top bar)  2175x1020 (app)  effective: 1795x1010
//                      but the app reports surface size: 2095x1010, border left 150, border right 150

// for D3: samsung g s8 

static int screen_width[7]      = { 0, 2014, 1397, 1980, 1133, 2079, 2014 };
static int screen_height[7]     = { 0, 996, 667, 996, 664, 966, 996 };
static int border_horizontal[7] = { 0, 122, 106, 105, 0, 181, 122};
static int border_vertical[7]   = { 0, 0, 0, 0, 13, 0, 0 };


// for D3: samsung a41

/*
static int screen_width[7]      = { 0, 2014, 1397, 2111, 1133, 2079, 2014 };
static int screen_height[7]     = { 0, 996, 667, 1006, 664, 966, 996 };
static int border_horizontal[7] = { 0, 122, 106, 156, 0, 181, 122};
static int border_vertical[7]   = { 0, 0, 0, 0, 13, 0, 0 };
*/

// for D3: motorola
/*
static int screen_width[7]      = { 0, 2014, 1397, 2095, 1133, 2079, 2014 };
static int screen_height[7]     = { 0, 996, 667, 1010, 664, 966, 996 };
static int border_horizontal[7] = { 0, 122, 106, 150, 0, 181, 122};
static int border_vertical[7]   = { 0, 0, 0, 0, 13, 0, 0 };
*/

// pixel size:  0.01188185654008438818565400843882 mm ?  (from online calculations)
//focal length: 6.7 mm  => from measurement then pixel_size = 0.00754055977064093182411092861206 mm  (from m1)
//                                                            0.00750417769892791061292331402408 mm  (from m2)

// measurement: d = 500mm,
//              H1 = 1069 mm, H2 = 820 mm
//              x1 = 413 pixels, x2 = 540 pixels

// actual image is 1185 x 667 (drone2 is used as reference from which other are derived)
static int IMAGE_MINIMUM_VALID_X; // = 106;
static int IMAGE_MAXIMUM_VALID_X; // = 1290;
static int IMAGE_MINIMUM_VALID_Y; // = 0;
static int IMAGE_MAXIMUM_VALID_Y; // = 667;

static int IMAGE_MINIMUM_REASONABLE_X;
static int IMAGE_MINIMUM_REASONABLE_Y;
static int IMAGE_MAXIMUM_REASONABLE_X;
static int IMAGE_MAXIMUM_REASONABLE_Y;

static long MIN_CORNER_SEGMENT_LENGTH_SQR; // = (150 * 150);  //12.6%
static long MAX_CLOSE_NEIGHBOR_POINTS_SQR; // = (100 * 100);  //7.2%
static long MIN_CORNER_DISTANCE; 

static const double EPSILON_INTERSECTION_PARALLEL_LINES = 0.15;

static const char *cpp_log_file = "/data/user/0/sk.uniba.krucena/files/cpplog.txt";
static const char *position_log_file = "/data/user/0/sk.uniba.krucena/files/position.txt";
static double time_debug_started = 0;
static int tables_precomputed = 0;

//static const double parallel_vectors_cross_epsilon = 0.14;  // about 8 degrees tolerance
//static const double parallel_vectors_cross_epsilon = 0.25;  // about 15 degrees tolerance
//static const double parallel_vectors_cross_epsilon = 0.7;  // about 45 degrees tolerance
static const double parallel_vectors_cross_epsilon = 0.86;  // about 60 degrees tolerance

#ifdef RELEASE_VERSION

// help the compiler avoid including the code in release version
#define CPP_DEBUG_ON 0
#define POSITION_DEBUG_ON 0
#define visualize_contours 0

#else
	
static int CPP_DEBUG_ON = 1;      // set to 0 to disable, 1 to enable, controlled from GUI
static int POSITION_DEBUG_ON = 1;  
static int visualize_contours = 1;

#endif

static int visualization = 0;  // 0 = default, 1 = RGB, 2 = BLACK, 3 = YELLOW

// for visualization
static const cv::Scalar black_color(0, 0, 0);
static const cv::Scalar red_color(255, 0, 0);
static const cv::Scalar green_color(0, 255, 0);
static const cv::Scalar blue_color(0, 0, 255);
static const cv::Scalar yellow_color(255, 255, 50);

// camera
static  int default_camera_center_x = 592;  
static  int default_camera_center_y = 333;
static  int camera_center_x;  
static  int camera_center_y;
//static const float camera_focal_length = 0.0064;   // m
static const float camera_focal_length = 0.0067;   // m
static float camera_pixel_size;
static const float default_pixel_size = 0.0000075;  // m
//static const float default_pixel_size = 0.000003375;  // m

// corners detected this close (relative) to the border are ignored unless not enough other are found
static const float UNREASONABLE_BORDER = 0.025;    

// camera color detection thresholds
static int black_maxRGB_t, black_chroma_t, red_t, green_t, blue_t, yellow_t;


void init_image_parameters(int drone_id)
{
	IMAGE_MINIMUM_VALID_X = border_horizontal[drone_id];
    IMAGE_MAXIMUM_VALID_X = screen_width[drone_id] - border_horizontal[drone_id] - 1;
    IMAGE_MINIMUM_VALID_Y = border_vertical[drone_id];
    IMAGE_MAXIMUM_VALID_Y = screen_height[drone_id] - border_vertical[drone_id] - 1;
	
	IMAGE_MINIMUM_REASONABLE_X = IMAGE_MINIMUM_VALID_X + (IMAGE_MINIMUM_VALID_X - IMAGE_MINIMUM_VALID_X) * UNREASONABLE_BORDER;
	IMAGE_MINIMUM_REASONABLE_Y = IMAGE_MINIMUM_VALID_Y + (IMAGE_MINIMUM_VALID_Y - IMAGE_MINIMUM_VALID_Y) * UNREASONABLE_BORDER;
	IMAGE_MAXIMUM_REASONABLE_X = IMAGE_MAXIMUM_VALID_X - (IMAGE_MINIMUM_VALID_X - IMAGE_MINIMUM_VALID_X) * UNREASONABLE_BORDER;
	IMAGE_MAXIMUM_REASONABLE_Y = IMAGE_MAXIMUM_VALID_Y + (IMAGE_MINIMUM_VALID_Y - IMAGE_MINIMUM_VALID_Y) * UNREASONABLE_BORDER;	
	
	camera_center_x = screen_width[drone_id] / 2;
	camera_center_y = screen_height[drone_id] / 2;
	MIN_CORNER_SEGMENT_LENGTH_SQR = (int)(0.09  * (screen_width[drone_id] - 2 * border_horizontal[drone_id]));  //original coef 0.12
	MIN_CORNER_SEGMENT_LENGTH_SQR *= MIN_CORNER_SEGMENT_LENGTH_SQR;
	MAX_CLOSE_NEIGHBOR_POINTS_SQR = (int)(0.072 * (screen_width[drone_id] - 2 * border_horizontal[drone_id]));
	MAX_CLOSE_NEIGHBOR_POINTS_SQR *= MAX_CLOSE_NEIGHBOR_POINTS_SQR;
	
	MIN_CORNER_DISTANCE = (int)(0.02  * (screen_width[drone_id] - 2 * border_horizontal[drone_id]));
	     
	camera_pixel_size = default_pixel_size * (float)(screen_width[2] - 2 * border_horizontal[2]) / (IMAGE_MAXIMUM_VALID_X - IMAGE_MINIMUM_VALID_X + 1); 
}

void print_image_parameters()
{
	FILE *f = fopen(cpp_log_file, "a+");
	fprintf(f, "IMAGE_MINIMUM_VALID_X=%d, IMAGE_MAXIMUM_VALID_X=%d, IMAGE_MINIMUM_VALID_Y=%d, IMAGE_MAXIMUM_VALID_Y=%d\n", 
	           IMAGE_MINIMUM_VALID_X, IMAGE_MAXIMUM_VALID_X,IMAGE_MINIMUM_VALID_Y, IMAGE_MAXIMUM_VALID_Y);
	fprintf(f, "camera_center_x=%d, camera_center_y=%d, MIN_CORNER_SEGMENT_LENGTH_SQR=%ld, MAX_CLOSE_NEIGHBOR_POINTS_SQR=%ld\n",
	           camera_center_x, camera_center_y, MIN_CORNER_SEGMENT_LENGTH_SQR, MAX_CLOSE_NEIGHBOR_POINTS_SQR);
	fprintf(f, "pixel_size=%f\n", camera_pixel_size);
	fprintf(f, "black_maxRGB_t=%d, black_chroma_t=%d, red_t=%d, green_t=%d, blue_t=%d, yellow_t=%d\n", black_maxRGB_t, black_chroma_t, red_t, green_t, blue_t, yellow_t);
	fprintf(f, "---\n");
	fclose(f);
}

// world locations of the verteces - indexed with corner ID 
static const cv::Vec2f world_coordinates[20] = { {-1.05f, 1.05f}, {-0.10f, 1.05f},    {0.10f, 1.05f}, {1.05f, 1.05f},
	                                             {-1.05f, 0.10f}, {-0.10f, 0.10f},    {0.10f, 0.10f}, {1.05f, 0.10f},
						                         {-1.05f, -0.10f}, {-0.10f, -0.10f},  {0.10f, -0.10f}, {1.05f, -0.10f},
                                                 {-1.05f, -1.05f}, {-0.10f, -1.05f},  {0.10f, -1.05f}, {1.05f, -1.05f},
                                                 {-0.575f, 0.575f}, {0.575f, 0.575f},  {-0.575f, -0.575f}, {0.575f, -0.575f}
											   };


// special return value that indicates that localization was not possible
static const cv::Vec4f unknown_camera_pos = cv::Vec4f(999.0f, 999.0f, 999.0f, 999.0f);
	    
/********************************************************** relative corners positions inference begin **************************************/

// COLOR ENCODING: blue = 0, black = 1, red = 2, green = 3
// actual layout:
//  BE BE BK BK
//  BE BE BK BK
//   G  G  R  R
//   G  G  R  R
//
// vertex IDs:
//
//  0  1  2  3
//  4  5  6  7
//  8  9 10 11
// 12 13 14 15

// yellow then follow from up-to down from left-to right 16..19

uint8_t color_ids[5][4] = { {0, 1, 4, 5}, {2, 3, 6, 7}, {10, 11, 14, 15}, {8, 9, 12, 13}, {16, 17, 18, 19} };

// index: [a:2][b:2][c:2][d:2][e:2], where a=color(P1), b=color(P2), c=angle4(outgoing_of_P1,outgoing_of_P2), d=bin_cross(out1,P1P2), e=bin_cross(in1,P1P2)
//                                   angle4(alpha) = 0,1,2,3 for alpha=0,90,180,270 (approx)
//                                   bin_cross(u,v)=sgn_plus_one_f(cross_product(u,v))
//    ambiguous case: 0/7 vs. 4/3 => determined by whether angle(P1P2,out2) > angle(in2,P2P1) => if true, set e=3
uint8_t id_inference1[1024];
uint8_t id_inference2[1024];

// these are for yellow-nonyellow pairs
uint8_t Y_id_inference1[256];
uint8_t Y_id_inference2[256];

int8_t id2dx[] = {1, 0, 0, -1};
int8_t id2dy[] = {0, 1, -1, 0};

static const float dot_cross_eps = 0.1736;   // corresponds to about 10 degrees error tolerance

static char str[1000];  // for debug prints

// calculated between a yellow and non-yellow corners, used in special ambiguous cases
static float min_distance;

long distance_sqr(cv::Point *a, cv::Point *b)
{
	return (a->x - (long)b->x) * (a->x - (long)b->x) + (a->y - (long)b->y) * (a->y - (long)b->y);
}

double current_millis_time()
{
    struct timespec tm;
	clock_gettime(CLOCK_REALTIME, &tm);
	return tm.tv_sec * 1000.0 + tm.tv_nsec / 1000000.0;
}

void init_cpp_debug(int drone_id)
{
    int first_run = 0;
    if (time_debug_started == 0) {
        time_debug_started = current_millis_time();
        first_run = 1;
    }
    if (first_run) init_image_parameters(drone_id);
		
    if (CPP_DEBUG_ON)
	{
		if (!first_run)
		{
			FILE *f = fopen(cpp_log_file, "a+");
			double tajm = current_millis_time() - time_debug_started;
			fprintf(f, "%10.2lf INIT: -----\n", tajm);
			fclose(f);
			return;
		}
		FILE *f = fopen(cpp_log_file, "w+");
		if (f == 0)
		{
#ifndef RELEASE_VERSION
			CPP_DEBUG_ON = 0;
#endif
			return;
		}
		fprintf(f, "Starting cpp debug (droneId=%d)...\n", drone_id);
		fclose(f);
		time_debug_started = current_millis_time();
		print_image_parameters();
	}
	
	if (POSITION_DEBUG_ON)
	{
		if (first_run)
		{
			FILE *f = fopen(position_log_file, "w+");
            if (f == 0)
            {
#ifndef RELEASE_VERSION
                POSITION_DEBUG_ON = 0;
#endif
                return;
            }
			fprintf(f, "%10.2lf INIT: -----\n", time_debug_started);
			fclose(f);  
		}
	}	
}

void log_position(cv::Vec4f pos)
{
	if (POSITION_DEBUG_ON == 0) return;
	double pos_time = current_millis_time() - time_debug_started;
	FILE *f = fopen(position_log_file, "a+");
	fprintf(f, "%10.2lf x=%6.2lf, y=%6.2lf, z=%6.2lf, alpha=%6.lf\n", pos_time, pos[0], pos[1], pos[2], pos[3] / (float)M_PI * 180.0);
	fclose(f);
}

void cpp_debug(const char *tag, const char *msg)
{
	if (CPP_DEBUG_ON == 0) return;
	FILE *f = fopen(cpp_log_file, "a+");
	double tajm = current_millis_time() - time_debug_started;
	fprintf(f, "%10.2lf %s: %s\n", tajm, tag, msg);
	fclose(f);
}

void cpp_debug_f(const char *tag, const char *msg, float num)
{
	if (CPP_DEBUG_ON == 0) return;
	FILE *f = fopen(cpp_log_file, "a+");
	double tajm = current_millis_time() - time_debug_started;
	fprintf(f, "%10.2lf %s: %s%.4lf\n", tajm, tag, msg, num);
	fclose(f);
}

void cpp_debug(const char *tag, const char *msg, long num)
{
	if (CPP_DEBUG_ON == 0) return;
	FILE *f = fopen(cpp_log_file, "a+");
	double tajm = current_millis_time() - time_debug_started;
	fprintf(f, "%10.2lf %s: %s%ld\n", tajm, tag, msg, num);
	fclose(f);
}

void cpp_debug(const char *tag, const char *msg, long num1, long num2)
{
	if (CPP_DEBUG_ON == 0) return;
	FILE *f = fopen(cpp_log_file, "a+");
	double tajm = current_millis_time() - time_debug_started;
	fprintf(f, "%10.2lf %s: %s%ld %ld\n", tajm, tag, msg, num1, num2);
	fclose(f);
}

void cpp_debug_f(const char *tag, const char *msg, float num1, float num2)
{
	if (CPP_DEBUG_ON == 0) return;
	FILE *f = fopen(cpp_log_file, "a+");
	double tajm = current_millis_time() - time_debug_started;
	fprintf(f, "%10.2lf %s: %s%.2f %.2f\n", tajm, tag, msg, num1, num2);
	fclose(f);
}


/** returns 0 for negative, 1 for 0, 2 for positive */
inline int sgn_plus_one(int8_t x)
{
	return (x > 0) ? 1 : ((x < 0) ? 2 : 0);
}

inline int sgn_plus_one_f(float x)
{
	return (x > dot_cross_eps) ? 1 : ((x < -dot_cross_eps) ? 2 : 0);
}

uint8_t angle_between(cv::Point2f v1, cv::Point2f v2)
{
	uint8_t output = 0;
	
	float dot = v1.dot(v2);
    float cross = v1.x * v2.y - v1.y * v2.x;
	
	if (fabs(cross) < dot_cross_eps) {
		if (dot > 0) output = 0;   // 0 deg
		else         output = 2;   // 180 deg
    } else if (fabs(dot) < dot_cross_eps) {
		if (cross > 0) output = 1; // 90 deg
        else           output = 3; // 270 deg
    }
	return output;
}

char *binrep(int i)
{
	static char br[20];
    sprintf(br, "%d%d|%d%d|%d%d|%d%d|%d%d", (i & 512) >> 9, (i & 256) >> 8, (i & 128) >> 7, (i & 64) >> 6, (i & 32) >> 5,
	                                          (i & 16) >> 4, (i & 8) >> 3, (i & 4) >> 2, (i & 2) >> 1, i & 1);
    return br;
}

char *binrep2(int i)
{
	static char br[20];
	sprintf(br, "%d%d|%d|%d|%d|%d|%d%d", (i & 128) >> 7, (i & 64) >> 6, (i & 32) >> 5, (i & 16) >> 4, (i & 8) >> 3, (i & 4) >> 2, (i & 2) >> 1, i & 1);
	return br;
}

cv::Point2f normalize_vector(cv::Point *v) 
{
    float len = std::sqrt(v->x * v->x + v->y * v->y);
	cv::Point2f n(v->x, v->y);
	
	if (len > 0) 
	{
		n.x /= len;
		n.y /= len;
	}
	
	return n;
}

cv::Point2f normalize_vector_f(cv::Point2f *v) 
{
    float len = std::sqrt(v->x * v->x + v->y * v->y);
	cv::Point2f n(v->x, v->y);
	
	if (len > 1e-6) 
	{
		n.x /= len;
		n.y /= len;
	}
	
	return n;
}

// this function works in pixel coordinate system ([0,0] is upper left corner, y grows down, x right)
void precompute_id_inference_tables()
{
	memset(id_inference1, 255, 1024);
	memset(id_inference2, 255, 1024);
	memset(Y_id_inference1, 255, 256);
	memset(Y_id_inference2, 255, 256);
	
	for (uint8_t c1 = 0; c1 < 4; c1++)
	{
		for (uint8_t c2 = 0; c2 < 4; c2++)
		{
			if (c1 == c2) continue;
			uint8_t neighboring_colors = (c1 + c2) & 1;
			
			for (uint8_t v1 = 0; v1 < 4; v1++)
			{
				uint8_t id1 = color_ids[c1][v1];
				int8_t x1 = id1 & 3;   // mod 4
				int8_t y1 = id1 >> 2;  // div 4

				uint8_t reduced_id1 = ((id1 & 4) >> 1) | (id1 & 1);
				int8_t vx = id2dx[reduced_id1];
				int8_t vy = id2dy[reduced_id1];

                // this produces vector turned 90 deg left in pixel coordinate system, that means 90 deg turn right from incoming to outgoing
				// which corresponds going clockwise in pixel coordinate system in the detected contours (one more occurence below)
				int8_t ux = vy;
		        int8_t uy = -vx;
				
		        // we have a directional outgoing vector from corner id1 of color c1 [vx, vy], and incoming vector to corner id1 [ux, uy], id1 is at coordinates [x1, y1]
                cv::Point2f out1(vx, vy);
				
				for (int v2 = 0; v2 < 4; v2++)
				{
					uint8_t id2 = color_ids[c2][v2];
					int8_t x2 = id2 & 3;   // mod 4
					int8_t y2 = id2 >> 2;  // div 4

					// id2 is at coordinates [x2,y2]
					// we want a sign of cross product of directional v = outgoing vector from corner id1 with w = vector (P2 - P1)
					int8_t wx = (x2 - x1);
					int8_t wy = (y2 - y1);
					
					uint8_t reduced_id2 = ((id2 & 4) >> 1) | (id2 & 1);
                    int8_t gx = id2dx[reduced_id2];
				    int8_t gy = id2dy[reduced_id2];
				
				    cv::Point2f out2(gx, gy);

					uint8_t v1_v2_angle = angle_between(out1, out2);
				
					// cross product: v × w = vx*wy - vy*wx
					int8_t cross_out1w = vx * wy - vy * wx;
					
					int8_t cross_in1w = ux * wy - uy * wx;
	
					uint16_t index = (c1 << 8) | (c2 << 6) | (v1_v2_angle << 4) | (sgn_plus_one(cross_out1w) << 2) | sgn_plus_one(cross_in1w);
	 
					if (neighboring_colors && ((index & 63) == 0b100101)) // special ambiguous case to be resolved by angle(P1P2,out2) > angle(in2, P2P1), ex. 0-13 vs. 1-12
					{
						//sprintf(str, "special case: c1=			\%hhu, c2=%hhu, index&63=%d, id1=%d, id2=%d", c1, c2, (index & 63), id1, id2);
						//cpp_debug("corners", str);
						
						cv::Point2f w(wx, wy);
				
				        // produce incoming vector from outgoing for this corner (see "occurence" above)
						int8_t hx = gy;
						int8_t hy = -gx;
						cv::Point2f in2(hx, hy);
						
						cv::Point2f r(-wx, -wy);

						float dot_P1P2_out2 = out2.dot(w);
						float dot_in2_P2P1 = in2.dot(r);
						
						//sprintf(str, "out2=[%.2lf,%.2lf], w=[%.2lf,%.2lf], in2=[%.2lf,%.2lf], r=[%.2lf,%.2lf]",
						//              out2.x, out2.y, w.x, w.y, in2.x, in2.y, r.x, r.y);
						//cpp_debug("corners", str);			  
						//cpp_debug_f("corners", "dot_P1P2_out2:", dot_P1P2_out2);
						//cpp_debug_f("corners", "dot_in2_P2P1:", dot_in2_P2P1);
		
						if (dot_P1P2_out2 > dot_in2_P2P1) 
						{
							index |= 0b11;  // set last part (rel(in1,P2) = 3) for the case 4,3 (blue,black) or similar (for other colors)
							//cpp_debug("corners", "AND dot_P1P2_out2 > dot_in2_P2P1! :)");
						}
						//else 
						//	cpp_debug("corners", "BUT dot_P1P2_out2 <= dot_in2_P2P1! :(");							
					}

					id_inference1[index] = id1;
					id_inference2[index] = id2;
				}
			}
		}
	}
	tables_precomputed = 1;
	
	char ln[100];
	for (int i = 0; i < 1024; i++)
	{	
        if (id_inference1[i] == 255) continue;
		sprintf(ln, "i=[%d~%s], id1=%3hhu, id2=%3hhu", i, binrep(i), id_inference1[i], id_inference2[i]); 
		cpp_debug("init", ln);
	}
	
	static const float yellow_x[4] = { 0.5, 2.5, 0.5, 2.5 };
	static const float yellow_y[4] = { 0.5, 0.5, 2.5, 2.5 };
	static const int8_t yellow_outgoing_x[4] = { 1, 0, 0, -1 };  
	static const int8_t yellow_outgoing_y[4] = { 0, 1, -1, 0 };
	
	for (uint8_t c2 = 0; c2 < 4; c2++)
	{
		for (uint8_t v1 = 0; v1 < 4; v1++)
		{
			uint8_t id1 = color_ids[4][v1];
			float x1 = yellow_x[v1];
			float y1 = yellow_y[v1];
			
			// outgoing
			int8_t vx = yellow_outgoing_x[v1];
			int8_t vy = yellow_outgoing_y[v1];
			cv::Point2f out1(vx, vy);
						
			// incoming
			int8_t ux = vy;
		    int8_t uy = -vx;
			cv::Point2f in1(ux, uy);
							
			for (uint8_t v2 = 0; v2 < 4; v2++)
			{
				uint8_t id2 = color_ids[c2][v2];
				float x2 = id2 & 3;   // mod 4
				float y2 = id2 >> 2;  // div 4
				// id2 is at coordinates [x2,y2]
				float wx = (x2 - x1);
				float wy = (y2 - y1);
				
				cv::Point2f w(wx, wy);
				w = normalize_vector_f(&w);
				
				// V2V1 
				cv::Point2f r(-w.x, -w.y);
				
				uint8_t reduced_id2 = ((id2 & 4) >> 1) | (id2 & 1);
                
				// outgoing
				int8_t gx = id2dx[reduced_id2];
				int8_t gy = id2dy[reduced_id2];
				cv::Point2f out2(gx, gy);
				
				// incoming
				int8_t hx = gy;
				int8_t hy = -gx;
				cv::Point2f in2(hx, hy);

				// cross product: v × w = vx*wy - vy*wx
				float cross_out1w = vx * w.y - vy * w.x;
				float cross_in1w = ux * w.y - uy * w.x;
				float cross_in2r = hx * r.y - hy * r.x;
				float cross_out2r = gx * r.y - gy * r.x;
				
				int index = (c2 << 6) | (cross_in1w > 0) << 5 | (cross_out1w > 0) << 4 | (cross_in2r > 0) << 3 | (cross_out2r > 0) << 2;
				for (int i = 0; i < 4; i++)
				{
					if (Y_id_inference1[index + i] == 255) Y_id_inference1[index + i] = id1;
					if (Y_id_inference2[index + i] == 255) Y_id_inference2[index + i] = id2;
				}
				
				float dot_P1P2_out2 = out2.dot(w);
				float dot_in2_P2P1 = in2.dot(r);
				
				//sprintf(str, "in1=[%.2lf,%.2lf], out1=[%.2lf,%.2lf], out2=[%.2lf,%.2lf], in2=[%.2lf,%.2lf], w=[%.2lf,%.2lf], r=[%.2lf,%.2lf]", in1.x, in1.y, out1.x, out1.y, out2.x, out2.y, in2.x, in2.y, w.x, w.y, r.x, r.y);
				//cpp_debug("init", str);
				//cpp_debug_f("init", "YPRE dot_P1P2_out2=", dot_P1P2_out2);
				//cpp_debug_f("init", "YPRE dot_in2_P2P1=", dot_in2_P2P1);

				// static const float about_40 = cos(40.0 / 180.0 * M_PI);   // 0.766
				// static const float about_50 = cos(50.0 / 180.0 * M_PI);   // 0.643
				// so if difference is larger than say 0.1, then they are not the same
				if (fabs(dot_P1P2_out2 - dot_in2_P2P1) < 0.1) // they are about the same => difficult case
				{
					// if they see each other in about the middle of the diagonal, there is still
					// ambiguity (e.g. case (16,5) vs. (19,0) ) and we distinguish between the 
					// two cases based on distance (about the smallest visible distance => 11, otherwise 01)
					// in this case we just look it up based on id1,id2, in real inference, we need to look
					// at the actual distances...
					if (((id1 == 16) && (id2 == 5)) || // these are neighbors within same color square
					    ((id1 == 17) && (id2 == 6)) || 
						((id1 == 18) && (id2 == 9)) || 
						((id1 == 19) && (id2 == 10)) || 
					    ((id1 == 19) && (id2 == 14)) ||  // but also many other
						((id1 == 18) && (id2 == 13)) ||  // cases which are ambiguous
						((id1 == 17) && (id2 == 2)) ||   // and return about minimal
						((id1 == 16) && (id2 == 1)) ||   // yellow-nonyellow distance
						((id1 == 19) && (id2 == 11)) ||  // when we fly and look
						((id1 == 18) && (id2 == 8)) ||
						((id1 == 17) && (id2 == 7)) ||
						((id1 == 16) && (id2 == 4)))
						index |= 0b11;
					else
					    index |= 0b01;
				}
				else if (dot_P1P2_out2 > dot_in2_P2P1)
				{
					index |= 0b10;
				}
				// the last implicit case (d1 < d2) is 0b00
                Y_id_inference1[index] = id1;
                Y_id_inference2[index] = id2;
				
				//DBGDBG
				//sprintf(ln, "index=[%d~%s], id1=%3hhu, id2=%3hhu", index, binrep2(index), Y_id_inference1[index], Y_id_inference2[index]); 
		        //cpp_debug("init", ln);
			}
		}
	}
	
	cpp_debug("init", "--------yellow-------");
	for (int i = 0; i < 256; i++)
	{	
        if (Y_id_inference1[i] == 255) continue;
		sprintf(ln, "i=[%d~%s], id1=%3hhu, id2=%3hhu", i, binrep2(i), Y_id_inference1[i], Y_id_inference2[i]); 
		cpp_debug("init", ln);
	}
}

/********************************************************** relative corners positions inference end **************************************/

// sets positive to 1, if u->v angle is counter-clockwise in world coordinates (that means clockwise in pixel coordinates)
int are_segments_parallel(std::pair<cv::Point *, cv::Point *> *u, std::pair<cv::Point *, cv::Point *> *v, int *is_direction_positive)
{
	cv::Point u_vector = (*u->second) - (*u->first);
	cv::Point v_vector = (*v->second) - (*v->first);
	
	cv::Point2f u_normalized = normalize_vector(&u_vector);
	cv::Point2f v_normalized = normalize_vector(&v_vector);
	
	// Parallel => cross = 0,  Perpendicular => cross = |u| * |v|, Small angle => small cross value, Cross sign => rotation direction 
	float cross = u_normalized.x * v_normalized.y - u_normalized.y * v_normalized.x;   
	*is_direction_positive = cross > 0;
	//cpp_debug_f("corners", "cross: ", cross);
	
	return (fabs(cross) < parallel_vectors_cross_epsilon);
}

int intersection(std::pair<cv::Point *, cv::Point *> *AB, std::pair<cv::Point *, cv::Point *> *CD, std::pair<cv::Point, std::pair<cv::Point2f,cv::Point2f>> *intersect)
{
	char dbgstr[200];
	sprintf(dbgstr, "AB: [%d,%d] - [%d,%d]; CD: [%d,%d] - [%d,%d]", AB->first->x, AB->first->y, AB->second->x, AB->second->y, CD->first->x, CD->first->y, CD->second->x, CD->second->y);
	cpp_debug("intersect", dbgstr);
	
	cv::Point u = *(AB->second) - *(AB->first);
    cv::Point v = *(CD->second) - *(CD->first);
    cv::Point w = *(AB->first) - *(CD->first);

    cv::Point2f u_normalized = normalize_vector(&u);
	cv::Point2f v_normalized = normalize_vector(&v);
	
    float denom = u_normalized.x * v_normalized.y - u_normalized.y * v_normalized.x; 

    if (std::abs(denom) < EPSILON_INTERSECTION_PARALLEL_LINES) { 
	  return 0; 
      // Lines are parallel or coincident
    }

    float t = (v_normalized.x * w.y - v_normalized.y * w.x) / denom; 

    intersect->first.x = AB->first->x + (int)(0.5 + t * u_normalized.x);
    intersect->first.y = AB->first->y + (int)(0.5 + t * u_normalized.y);
	intersect->second.first = cv::Point2f(u_normalized.x, u_normalized.y);
	intersect->second.second = cv::Point2f(v_normalized.x, v_normalized.y);
	
    return 1;
}

cv::Vec3f get_direction_vector_from_pixel(cv::Point *pixel, float cosAlpha, float sinAlpha) 
{
    float x = pixel->x - camera_center_x;
    float y = camera_center_y - pixel->y;   // here is the place we invert y-axis of camera frame, it grows opposite to the world coordinates y-axis

    float vx = camera_pixel_size * (x * cosAlpha + y * sinAlpha);
    float vy = camera_pixel_size * (-x * sinAlpha + y * cosAlpha);
    float vz = camera_focal_length;

    return cv::normalize(cv::Vec3f(vx, vy, vz));
}

// Estimates the 3D point closest to all given 3D lines
// Each line is given by a point (P) and a direction (V), V must be normalized
cv::Point3f estimate_camera_position(const std::vector<cv::Point3f>& points,
                                     const std::vector<cv::Point3f>& directions)
{
    CV_Assert(points.size() == directions.size() && points.size() >= 2);

    cv::Matx33f A = cv::Matx33f::zeros();
    cv::Vec3f b = cv::Vec3f(0, 0, 0);

    for (size_t i = 0; i < points.size(); ++i)
	{
        const cv::Vec3f p(points[i].x, points[i].y, points[i].z);
        const cv::Vec3f v = cv::normalize(cv::Vec3f(directions[i].x, directions[i].y, directions[i].z));

        // Projection matrix: I - v * v^T
        cv::Matx33f I = cv::Matx33f::eye();
        cv::Matx33f vvT = v * v.t(); // outer product
        cv::Matx33f A_i = I - vvT;

        A += A_i;
        b += A_i * p;
    }

    // Solve A * x = b
    cv::Vec3f cam_pos;
    cv::solve(A, b, cam_pos, cv::DECOMP_SVD);

    return cv::Point3f(cam_pos);
}

inline float dotproduct(cv::Point2f &u, cv::Point2f &v)
{
	return u.x * v.x + u.y * v.y;
}

int far_enough_from_border(cv::Point p)
{
	if (p.x < IMAGE_MINIMUM_VALID_X + 5) return 0;
	if (p.x > IMAGE_MAXIMUM_VALID_X - 5) return 0;
	if (p.y < IMAGE_MINIMUM_VALID_Y + 5) return 0;
	if (p.y > IMAGE_MAXIMUM_VALID_Y - 5) return 0;
	return 1;
}

void find_corners(cv::Mat &thresholded_image, cv::Mat &input, const cv::Scalar &corner_color, std::vector<std::pair<cv::Point,std::pair<cv::Point2f, cv::Point2f>>> &corner_points)
{
	std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
		
    // Step 1: find contours in the thresholded image, and approximate them with polygons
	cv::findContours(thresholded_image, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE); //, offset);
	
	//DBGDBG
	cpp_debug("corners", "step 1, #of contours=", contours.size());
	

    // replace contours with polygon approximations

    for (size_t i = 0; i < contours.size(); ++i)
	{
		std::vector<cv::Point>& contour = contours[i];
		std::vector<cv::Point> poly;
		
        cv::approxPolyDP (contour, poly, 10.0, true);
		
		contours[i] = std::move(poly);
	}

    // optional: visualize the contours
	
	if (visualize_contours)
	{
		for (size_t i = 0; i < contours.size(); ++i) 
		{
			std::vector<cv::Point>& contour = contours[i];
			if (contour.size() < 3) continue;		

			cv::Point *last = &contour.back();	
			
			for (size_t j = 0; j < contour.size(); ++j)	{
				cv::Point *pt = &contour[j];			
				cv::line(input, *last, *pt, cv::Scalar(100, 100, 25), 4, cv::LINE_4);
				last = pt;
			}
		}
	}
	
    // Step 2: extract segments that are long enough (min_corner_segment_length) to a list of segments (for each contour separately)
	//     segments are represented as pairs of points
	//DBGDBG
	cpp_debug("corners", "step 2, #of contours=", contours.size());
	
	std::vector<std::vector<std::pair<cv::Point *, cv::Point *>>> segments_from_contours;
	
	for (size_t i = 0; i < contours.size(); ++i) 
	{
		std::vector<cv::Point>& contour = contours[i];
		if (contour.size() < 3) continue;		

		cv::Point *last = &contour.back();
		std::vector<std::pair<cv::Point *, cv::Point *>> set_of_segments_from_contour;
		
		for (size_t j = 0; j < contour.size(); ++j)	
		{
			cv::Point *pt = &contour[j];
            long dist_sqr = distance_sqr(last, pt);
			if (dist_sqr >= MIN_CORNER_SEGMENT_LENGTH_SQR)
			{
				set_of_segments_from_contour.push_back(std::move(std::make_pair(last, pt)));
			}
			last = pt;
		}
		//DBGDBG
		cpp_debug("corners", "extracted from contour of size long segments of size: ", contour.size(), set_of_segments_from_contour.size());
		
		if (set_of_segments_from_contour.size() > 1) 
		    segments_from_contours.push_back(std::move(set_of_segments_from_contour));
	}

    // Step 3: now all consecutive segment pairs in the extracted sets of segments form corners 
	//        unless they are (almost - wrt. perspective) parallel. add them to list of corners, if so
	//  corner is a pair of pairs of points, i.e. a segment pair
	//    TODO: the last float is only legacy and should be removed
	std::vector<std::tuple<std::pair<cv::Point *, cv::Point *>, std::pair<cv::Point *, cv::Point *>, float>> corners;
	
	cpp_debug("corners", "step 3, remaining #of contours with long segments=", segments_from_contours.size());
	
	for (size_t i = 0; i < segments_from_contours.size(); ++i) 
	{
        std::vector<std::pair<cv::Point *, cv::Point *>>& contour = segments_from_contours[i];
		//DBGDBG
		cpp_debug("corners", "browsing next contour with length = ", contour.size());	
		
		std::pair<cv::Point *, cv::Point *> *last_segment = &contour.back();
		
		//DBGDBG
		cpp_debug("corners", "considering corner in next contour i=", (long)i);		
		for (size_t j = 0; j < contour.size(); ++j)			
		{
			std::pair<cv::Point *, cv::Point *> *current_segment = &contour[j];
			
			//DBGDBG
			cpp_debug("corners", "u p1 = ", last_segment->first->x, last_segment->first->y);
			cpp_debug("corners", "u p2 = ", last_segment->second->x, last_segment->second->y);
			cpp_debug("corners", "v p1 = ", current_segment->first->x, current_segment->first->y);
			cpp_debug("corners", "v p2 = ", current_segment->second->x, current_segment->second->y);

            int positive_direction;			
			if (!are_segments_parallel(last_segment, current_segment, &positive_direction))
			{
				//TODO: update the representation since the last 0 is not used => drop tuple outer shell
				if (positive_direction)
				{
					// clockwise in pixel coordinates -> take it as is
					std::tuple<std::pair<cv::Point *, cv::Point *>, std::pair<cv::Point *, cv::Point *>, float> a_new_corner =  
						std::make_tuple(std::move(*last_segment), std::move(*current_segment), 0);
					corners.push_back(std::move(a_new_corner));
				}
				else
				{
					// counter-clockwise in pixel coordinates -> take in reverse order (then also reverse both vectors)
					std::pair<cv::Point *, cv::Point *> new_segment_1(current_segment->second, current_segment->first);
					std::pair<cv::Point *, cv::Point *> new_segment_2(last_segment->second, last_segment->first);
					
					std::tuple<std::pair<cv::Point *, cv::Point *>, std::pair<cv::Point *, cv::Point *>, float> a_new_corner =  
						std::make_tuple(std::move(new_segment_1), std::move(new_segment_2), 0);
					corners.push_back(std::move(a_new_corner));					
				}
				//DBGDBG
				cpp_debug("corners", "  ----> taken");
			}
            //DBGDBG
			else cpp_debug("corners", "  --------");

			last_segment = current_segment;
		}
	}

	// DBG: visualize the corners found
	if (visualize_contours) {
		for (size_t i = 0; i < corners.size(); i++)
		{			
			cv::line(input, *(std::get<0>(corners[i]).first), *(std::get<0>(corners[i]).second), cv::Scalar(255, 30, 30), 5, cv::LINE_4);
			cv::line(input, *(std::get<1>(corners[i]).first), *(std::get<1>(corners[i]).second), cv::Scalar(255, 30, 30), 5, cv::LINE_4);
		}
	}
	
	
	// Step 4: calculate the actual corner points from the line segment intersections
	//         and for such corner pairs that are close to each other (inner and outer line), take average of the two
	corner_points.reserve(corners.size());
	
	// calculate intersections of corners
	for (size_t i = 0; i < corners.size(); i++)
	{
		std::pair<cv::Point, std::pair<cv::Point2f,cv::Point2f>> intersect;   
		// point and the corresponding segments
		if (intersection(&(std::get<0>(corners[i])), &(std::get<1>(corners[i])), &intersect))
		{
			if (far_enough_from_border(intersect.first))
			    corner_points.push_back(intersect);
		}
	}
	
	// eliminate duplicity, but those that were not duplicit are unreliable,
	//   keep them only if we have less than 3 points
	
	std::vector<int> unreliable;
	
	for (size_t i = 0; i < corner_points.size(); i++)
	{
		int reliable = 0;
		for (size_t j = i + 1; j < corner_points.size(); j++)
		{
			if (distance_sqr(&corner_points[i].first, &corner_points[j].first) <= MAX_CLOSE_NEIGHBOR_POINTS_SQR)
			{
			    corner_points[j].first.x = (corner_points[i].first.x + corner_points[j].first.x + 0.5) / 2.0;	
				corner_points[j].first.y = (corner_points[i].first.y + corner_points[j].first.y + 0.5) / 2.0;
				// select the directional vectors of the one where they are "more perpendicular"
                if (fabs(dotproduct(corner_points[i].second.first, corner_points[i].second.second)) >
				    fabs(dotproduct(corner_points[j].second.first, corner_points[j].second.second)))
					corner_points[j].second = corner_points[i].second;
								
				corner_points.erase(corner_points.begin() + i);
                i--;
				reliable = 1;
                break;
			}
		}
		if (!reliable) unreliable.push_back(i);
	}
	
	// POSSIBLE IMPROVEMENT: this only considers corners of one color, but we should remove the unreliable ones
	//                       even if we have no other corners of this color, but suffient others - but we do not
	//                       know that yet - so the unreliable collection should be global, and removal take place
	//                       after all colors are processed
	if (corner_points.size() - unreliable.size() >= 3)
	{
		for (int i = unreliable.size() - 1; i >= 0; i--)
		{
			corner_points.erase(corner_points.begin() + unreliable[i]);
			cpp_debug("corners", "removed unreliable corner #", unreliable[i]);
		}
	}	
	
	if (visualize_contours)
	{
		// DBG: visualize the corner points found
		cpp_debug("corners", "--------------------corners found:");

		cv::Point delta(5, 5);
		cv::Point smalldelta(1, 1);
		for (size_t i = 0; i < corner_points.size(); i++)
		{
			cpp_debug("corners", "[xcam,ycam]: ", corner_points[i].first.x, corner_points[i].first.y);
            cpp_debug_f("corners", "       A: [dx,dy]: ", corner_points[i].second.first.x, corner_points[i].second.first.y);
            cpp_debug_f("corners", "       B: [dx,dy]: ", corner_points[i].second.second.x, corner_points[i].second.second.y);
			
			cv::rectangle(input, corner_points[i].first - delta, corner_points[i].first + delta, cv::Scalar(255, 255, 255), cv::FILLED);   
            cv::rectangle(input, corner_points[i].first - smalldelta, corner_points[i].first + smalldelta, corner_color, cv::FILLED);   			
		}
	}	
}

// this function works completely in pixel coordinate system ([0,0] is upper left corner, y grows down, x right)
void determine_ids(uint8_t c1, uint8_t c2, std::pair<cv::Point, std::pair<cv::Point2f, cv::Point2f>> &corner1, std::pair<cv::Point, std::pair<cv::Point2f, cv::Point2f>> &corner2, uint8_t &id1, uint8_t &id2)
{
    sprintf(str, "determine_ids(c1=%hhu, c2=%hhu,\n               P1=[%d,%d], P1in=(%.3f,%.3f), P1out=(%.3f,%.3f)\n               P2=[%d,%d], P2in=(%.3f,%.3f), P2out=(%.3f,%.3f)",
	             c1, c2, corner1.first.x, corner1.first.y, corner1.second.first.x, corner1.second.first.y, corner1.second.second.x, corner1.second.second.y,
				 corner2.first.x, corner2.first.y, corner2.second.first.x, corner2.second.first.y, corner2.second.second.x, corner2.second.second.y);
	cpp_debug("corners", str);

    cv::Point2f *out1_ref = &(corner1.second.second);  // outgoing
	cv::Point2f *out2_ref = &(corner2.second.second);  // outgoing
				 
    cv::Point2f out1 = normalize_vector_f(out1_ref);
	cv::Point2f out2 = normalize_vector_f(out2_ref);
    uint8_t v1_v2_angle = angle_between(out1, out2);
	
	cpp_debug("corners", "angle(out1,out2)=", (int)v1_v2_angle);
	
	cv::Point &P1 = corner1.first;
	cv::Point &P2 = corner2.first;
	
	int P1x = P1.x;
	int P1y = P1.y;    
	
	int P2x = P2.x;
	int P2y = P2.y;
	
	int wx = P2x - P1x;
	int wy = P2y - P1y;
	cv::Point w_int(wx, wy);
	cv::Point2f w = normalize_vector(&w_int);
	
	// cross product: v × w = vx*wy - vy*wx
	float cross_out1w = out1.x * w.y - out1.y * w.x;
	
	cv::Point2f *in1_ref = &corner1.second.first;  // incoming
	cv::Point2f in1 = normalize_vector_f(in1_ref);

	float cross_in1w = in1.x * w.y - in1.y * w.x;
	
    uint16_t index = (c1 << 8) | (c2 << 6) | (v1_v2_angle << 4) | (sgn_plus_one_f(cross_out1w) << 2) | sgn_plus_one_f(cross_in1w);
	
	uint8_t neighboring_colors = (c1 + c2) & 1;
	
	if (neighboring_colors && ((index & 63) == 0b100101)) // special ambiguous case to be resolved by angle(P1P2,out2) > angle(in2, P2P1)
	{
		cv::Point2f *in2_ref = &(corner2.second.first);  // incoming
		cv::Point2f in2 = normalize_vector_f(in2_ref);

		cv::Point2f r(-w.x, -w.y);

		float dot_P1P2_out2 = out2.dot(w);
		float dot_in2_P2P1 = in2.dot(r);
		
		if (dot_P1P2_out2 > dot_in2_P2P1) index |= 0b11;  // set last part (rel(in1,P2) = 3) for the case 4,3 (blue,black) or similar (for other colors)
	}
	
	sprintf(str, "    index=%s", binrep(index));
	cpp_debug("corners", str);
	
	id1 = id_inference1[index];
	id2 = id_inference2[index];
}

// other color, yellow corner, other corner, out: id1, out: id2
void Y_determine_ids(uint8_t c2, std::pair<cv::Point, std::pair<cv::Point2f, cv::Point2f>> &corner1, std::pair<cv::Point, std::pair<cv::Point2f, cv::Point2f>> &corner2, uint8_t &id1, uint8_t &id2)
{
    sprintf(str, "Y_determine_ids(c2=%hhu,\n               P1=[%d,%d], P1in=(%.3f,%.3f), P1out=(%.3f,%.3f)\n               P2=[%d,%d], P2in=(%.3f,%.3f), P2out=(%.3f,%.3f)",
	             c2, corner1.first.x, corner1.first.y, corner1.second.first.x, corner1.second.first.y, corner1.second.second.x, corner1.second.second.y,
				 corner2.first.x, corner2.first.y, corner2.second.first.x, corner2.second.first.y, corner2.second.second.x, corner2.second.second.y);
	cpp_debug("corners", str);

    cv::Point2f *out1_ref = &(corner1.second.second);  // outgoing
	cv::Point2f *out2_ref = &(corner2.second.second);  // outgoing

    cv::Point2f out1 = normalize_vector_f(out1_ref);
	cv::Point2f out2 = normalize_vector_f(out2_ref);
		
	cv::Point &P1 = corner1.first;
	cv::Point &P2 = corner2.first;
	
	int P1x = P1.x;
	int P1y = P1.y;
	
	int P2x = P2.x;
	int P2y = P2.y;
	
	int wx = P2x - P1x;
	int wy = P2y - P1y;
	cv::Point w_int(wx, wy);
	cv::Point2f w = normalize_vector(&w_int);
	cv::Point2f r(-w.x, -w.y);
	
	cv::Point2f *in1_ref = &corner1.second.first;  // incoming
	cv::Point2f in1 = normalize_vector_f(in1_ref);

	cv::Point2f *in2_ref = &(corner2.second.first);  // incoming
	cv::Point2f in2 = normalize_vector_f(in2_ref);
	
	// cross product: v × w = vx*wy - vy*wx
	float cross_out1w = out1.x * w.y - out1.y * w.x;
	float cross_in1w = in1.x * w.y - in1.y * w.x;
	float cross_out2r = out2.x * r.y - out2.y * r.x;
	float cross_in2r = in2.x * r.y - in2.y * r.x;
				
    // so far we have this, but missing more delicate details
    int index = (c2 << 6) | (cross_in1w > 0) << 5 | (cross_out1w > 0) << 4 | (cross_in2r > 0) << 3 | (cross_out2r > 0) << 2;
	
	float dot_P1P2_out2 = out2.dot(w);
	float dot_in2_P2P1 = in2.dot(r);
	
	cpp_debug_f("corners", "dot_P1P2_out2=", dot_P1P2_out2);
	cpp_debug_f("corners", "dot_in2_P2P1=", dot_in2_P2P1);

    // BIG ISSUE:  one more ambiguity: 17-15 vs 19-14 (and many similar - 8 in fact)  
	//             all above indicators are the same for both pairs
	//             and the following will consider them minimum yellow-nonyellow distance,
	//              so in precomputed tables we have to take that into account for these
	//              very specific cases

	if (fabs(dot_P1P2_out2 - dot_in2_P2P1) < 0.1f) // they are about the same => difficult case
	{
		// if they see each other in about the middle of the diagonal, there is still
		// ambiguity (e.g. case (16,5) vs. (19,0) ) and we distinguish between the 
		// two cases based on distance (about the smallest visible distance => 11, otherwise 01)
		float dist = std::sqrt(w_int.x * w_int.x + w_int.y * w_int.y);
		cpp_debug_f("corners", "dist=", dist);
		cpp_debug_f("corners", "min_dist=", min_distance);
        if (dist / min_distance < 1.3f)  // 30% tolerance for minimum distance (two corners could look nearer in another part of image)
			index |= 0b11;
		else
			index |= 0b01;
	}
	else if (dot_P1P2_out2 > dot_in2_P2P1)
	{
		index |= 0b10;
	}
	// the last implicit case (d1 < d2) is 0b00
	
	sprintf(str, "    index=%s", binrep2(index));
	cpp_debug("corners", str);
	
	id1 = Y_id_inference1[index];
	id2 = Y_id_inference2[index];
}

void normalize_all_vectors_in_corner_points(std::vector<std::pair<cv::Point,std::pair<cv::Point2f,cv::Point2f>>> *corner_points)
{
        for (int i = 0; i < 5; i++)
        {
                std::vector<std::pair<cv::Point,std::pair<cv::Point2f,cv::Point2f>>> *cp = &(corner_points[i]);
                int num_corners = cp->size();
                for (int j = 0; j < num_corners; j++)
                {
                        cv::Point2f &in_vec = (*cp)[j].second.first;
                        float n = cv::norm(in_vec);
                        if (n > 1e-6) in_vec /= n;

                        cv::Point2f &out_vec = (*cp)[j].second.second;
                        n = cv::norm(out_vec);
                        if (n > 1e-6) out_vec /= n;
                }
        }
}

float filter_yaw(float yaw)
{
	static float last_reported_yaw = 0.0f;
	static int yaw_counter = 0;
	static const float MAX_ALLOWED_YAW_JUMP = 35.0f / 180.0f * M_PI;
	// TODO: estimate the following based on FPS rate
	static const int MAX_BLOCKED_ITEMS_WHEN_YAW_JUMPS = 6;
	
	if (fabs(yaw - last_reported_yaw) <= MAX_ALLOWED_YAW_JUMP)
	{
		last_reported_yaw = yaw;
		yaw_counter /= 2;
		return yaw;
	}
	else
	{
		yaw_counter++;
		if (yaw_counter > MAX_BLOCKED_ITEMS_WHEN_YAW_JUMPS)
		{
			yaw_counter = 0;
			last_reported_yaw = yaw;
			return yaw;
		}
	}
	return last_reported_yaw;
}

float filter_height(float height)
{
	static float last_reported_height = 0.0f;
	static int height_counter = 0;
	static const float MAX_ALLOWED_HEIGHT_JUMP = 0.45;  // 45 cm
	// TODO: estimate the following based on FPS
	static const int MAX_BLOCKED_ITEMS_WHEN_HEIGHT_JUMPS = 6;
	
	if (fabs(height - last_reported_height) <= MAX_ALLOWED_HEIGHT_JUMP)
	{
		last_reported_height = height;
		height_counter /= 2;
		return height;
	}
	else
	{
		height_counter++;
		if (height_counter > MAX_BLOCKED_ITEMS_WHEN_HEIGHT_JUMPS)
		{
			height_counter = 0;
			last_reported_height = height;
			return height;
		}
	}
	return last_reported_height;
}

cv::Vec2d filter_position(cv::Vec2d &position)
{
	static double last_reported_x = 0.0;
	static double last_reported_y = 0.0;
	static const float MAX_ALLOWED_POSITION_JUMP_SQR = 0.35 * 0.35;  // 35 cm
	// TODO: estimate the following based on FPS
	static const int MAX_BLOCKED_ITEMS_WHEN_POSITION_JUMPS = 6;
	static int pos_counter = MAX_BLOCKED_ITEMS_WHEN_POSITION_JUMPS;
	
	cv::Vec2d result(position);
	
	double jump_size = (last_reported_x - position[0]) * (last_reported_x - position[0]) + (last_reported_y - position[1]) * (last_reported_y - position[1]);
	
	if (jump_size <= MAX_ALLOWED_POSITION_JUMP_SQR)
	{
		last_reported_x = position[0];
		last_reported_y = position[1];
		pos_counter /= 2;
		return result;
	}
	else
	{
		pos_counter++;
		if (pos_counter > MAX_BLOCKED_ITEMS_WHEN_POSITION_JUMPS)
		{
			pos_counter = 0;
			last_reported_x = position[0];
			last_reported_y = position[1];
			return result;
		}
	}
	result[0] = last_reported_x;
	result[1] = last_reported_y;
	return result;
}

extern "C"
JNIEXPORT void JNICALL
Java_sk_uniba_krucena_NativeBridge_setupColors(JNIEnv *env,
													 jobject,
													 jint new_black_maxRGB_t, 
                                                      jint new_black_chroma_t, 
													  jint new_red_t, 
													  jint new_green_t, 
													  jint new_blue_t, 
													  jint new_yellow_t)
{
	black_maxRGB_t = new_black_maxRGB_t;
	black_chroma_t = new_black_chroma_t; 
	red_t = new_red_t; 
    green_t = new_green_t;
	blue_t = new_blue_t; 
	yellow_t = new_yellow_t;
}

extern "C"
JNIEXPORT void JNICALL
Java_sk_uniba_krucena_NativeBridge_setMode(JNIEnv *env,
												 jobject,
												 jint visualization_mode,
												 jint show_contours,
												 jint cpp_debug,
												 jint position_debug)
{
    visualization = visualization_mode;
#ifndef RELEASE_VERSION
	visualize_contours = show_contours;
	CPP_DEBUG_ON = cpp_debug;
	POSITION_DEBUG_ON = position_debug;
#endif
}

extern "C"
JNIEXPORT void JNICALL
Java_sk_uniba_krucena_NativeBridge_localization(
        JNIEnv *env,
        jobject,
        jlong matAddrInput,
        jfloatArray cameraPosition,
		jint drone_id
		) {

    // Static Mats for color extraction 
    static cv::Mat maxRG, maxRB, maxGB, minVAR, maxRGB;
    static cv::Mat black, red, green, blue, yellow;
    static cv::Size lastSize;
	static std::vector<cv::Mat> channels(3);

	init_cpp_debug(drone_id);

    if (!tables_precomputed)
        precompute_id_inference_tables();
    	
    cv::Mat &input = *(cv::Mat *) matAddrInput;
	
	// on first call or input size changed, reallocate
    if (black.empty() || input.size() != lastSize) {
        lastSize = input.size();
        black = cv::Mat(input.size(), CV_8UC1);
        red = cv::Mat(input.size(), CV_8UC1);
        green = cv::Mat(input.size(), CV_8UC1);
        blue = cv::Mat(input.size(), CV_8UC1);
        yellow = cv::Mat(input.size(), CV_8UC1);
        maxRG = cv::Mat(input.size(), CV_8UC1);
        maxRB = cv::Mat(input.size(), CV_8UC1);
        maxGB = cv::Mat(input.size(), CV_8UC1);	
		minVAR = cv::Mat(input.size(), CV_8UC1);
		maxRGB = cv::Mat(input.size(), CV_8UC1);		
    }

    if (channels[0].empty() || 
		channels[0].rows != input.rows || 
		channels[0].cols != input.cols || 
		channels[0].type() != CV_8UC1) {

		// Reallocate only if needed
		for (int i = 0; i < 3; ++i) {
			channels[i] = cv::Mat(input.size(), CV_8UC1);  // assuming input is CV_8UC3
		}
	}

    cv::split(input, channels);  // uses existing memory
					
    // maxRG = max(R, G)
    cv::max(channels[0], channels[1], maxRG);

    // maxRB = max(R, B)
    cv::max(channels[0], channels[2], maxRB);

    // maxGB = max(G, B)
    cv::max(channels[1], channels[2], maxGB);

	// minVAR = min(R, G)   (minRG)
	cv::min(channels[0], channels[1], minVAR);
	
	// yellow = minRG - (maxRG - minRG) - B = 2 * minRG - maxRG - B   
	cv::subtract(maxRG, minVAR, yellow);
	cv::subtract(minVAR, yellow, yellow);
	cv::subtract(yellow, channels[2], yellow); 
	
	// minVAR = min(minRG, B)   (minRGB)
	cv::min(minVAR, channels[2], minVAR);
	
    // maxRGB = R + G + B
    cv::add(maxRG, channels[2], maxRGB);
	
	double brightness = cv::mean(maxRGB)[0];
	
	// clearing black borders with grey to avoid black detection there
	if (IMAGE_MINIMUM_VALID_X > 0)
	{
	    cv::rectangle(maxRGB, cv::Point(0, 0), cv::Point(IMAGE_MINIMUM_VALID_X - 1, maxRGB.rows - 1), cv::Scalar(200, 255, 255), cv::FILLED);
	    cv::rectangle(maxRGB, cv::Point(IMAGE_MAXIMUM_VALID_X + 1, 0), cv::Point(maxRGB.cols - 1, maxRGB.rows - 1), cv::Scalar(200, 255, 255), cv::FILLED);
	}
	if (IMAGE_MINIMUM_VALID_Y > 0)
	{
	    cv::rectangle(maxRGB, cv::Point(0, 0), cv::Point(maxRGB.cols - 1, IMAGE_MINIMUM_VALID_Y - 1), cv::Scalar(200, 255, 255), cv::FILLED);
	    cv::rectangle(maxRGB, cv::Point(0, IMAGE_MAXIMUM_VALID_Y + 1), cv::Point(maxRGB.cols - 1, maxRGB.rows - 1), cv::Scalar(240, 240, 240), cv::FILLED);
	}
	
	// minVAR = maxRGB - minRGB  (chroma)
	cv::subtract(maxRGB, minVAR, minVAR);
	
	// minVAR = minVAR + blue   - better distinguish between black and blue
    cv::add(minVAR, channels[2], minVAR);

    // red = red - maxGB 
	cv::subtract(channels[0], maxGB, red);

    // green = green - maxRB 
	cv::subtract(channels[1], maxRB, green);

    // blue = blue - maxRG 
	cv::subtract(channels[2], maxRG, blue);
	
	// alternate for yellow:
	//   yellow = min(red,green) - blue
	// (that would save 2 full-image operations)
	    
	//cv::blur(maxRGB, maxRGB, cv::Size(10, 10));
	
	cpp_debug_f("corners", "mean br=", brightness);
	
	/* this worked in the lab, but does not work in steelpark: 
	black_maxRGB_t = brightness / 1.32;
	black_chroma_t = brightness / 2.22;
	red_t = brightness / 3.2;
	green_t = brightness / 4.2;
	blue_t = brightness / 4.2;
	yellow_t = brightness / 8;  */
	
	cv::threshold(maxRGB, maxRGB, black_maxRGB_t, 200.0, cv::THRESH_BINARY_INV);  
	cv::threshold(minVAR, minVAR, black_chroma_t, 200.0, cv::THRESH_BINARY_INV); 
	cv::bitwise_and(maxRGB, minVAR, black);
	
	cv::threshold(red, red, red_t, 200.0, cv::THRESH_BINARY);            
	cv::threshold(green, green, green_t, 200.0, cv::THRESH_BINARY);  
	cv::threshold(blue, blue, blue_t, 200.0, cv::THRESH_BINARY); 
	cv::threshold(yellow, yellow, yellow_t, 200.0, cv::THRESH_BINARY);     
	
	// representation of corners in camera frame system: (corner_point, (incoming vector, outgoing vector)) 
	std::vector<std::pair<cv::Point,std::pair<cv::Point2f,cv::Point2f>>> corner_points[5];  // index is color (see COLOR ENCODING)
	
	cpp_debug("corners", "blue");
    find_corners(blue, input, blue_color, corner_points[0]);
    cpp_debug("corners", "black");
	find_corners(black, input, black_color, corner_points[1]);
    cpp_debug("corners", "red");
	find_corners(red, input, red_color, corner_points[2]);
    cpp_debug("corners", "green");
	find_corners(green, input, green_color, corner_points[3]);
	cpp_debug("corners", "yellow");
	find_corners(yellow, input, yellow_color, corner_points[4]);
	
	// let's remove those corners that are on the edge of the camera view - these are often not precise,
	// but only if we have enough corners in total
	int total_corners_we_have = corner_points[0].size() + corner_points[1].size() + corner_points[2].size() + corner_points[3].size() + corner_points[4].size();
    if (total_corners_we_have > 3)
	{
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < corner_points[i].size(); j++)
				if ((corner_points[i][j].first.x < IMAGE_MINIMUM_REASONABLE_X) ||
					(corner_points[i][j].first.x > IMAGE_MAXIMUM_REASONABLE_X) ||
					(corner_points[i][j].first.y < IMAGE_MINIMUM_REASONABLE_Y) ||
					(corner_points[i][j].first.y > IMAGE_MAXIMUM_REASONABLE_Y))
					{
						cpp_debug("corners", "removed a corner close to the edge");
						corner_points[i].erase(corner_points[i].begin() + j);
						total_corners_we_have--;
						if (total_corners_we_have == 3) break;
					}
	}
	if (visualization == 2)
		cv::merge(std::vector<cv::Mat>{maxRGB, minVAR, black}, input);
	else if (visualization == 1)
		cv::merge(std::vector<cv::Mat>{red, green, blue }, input);
	else if (visualization == 3)
        cv::merge(std::vector<cv::Mat>{yellow, yellow, channels[2] }, input);
		
	
	normalize_all_vectors_in_corner_points(corner_points);
	
	uint8_t corner_counts[5] = { (uint8_t)corner_points[0].size(), (uint8_t)corner_points[1].size(), (uint8_t)corner_points[2].size(), 
	                             (uint8_t)corner_points[3].size(), (uint8_t)corner_points[4].size() };
	
	if (corner_counts[0] + corner_counts[1] + corner_counts[2] + corner_counts[3] + corner_counts[4] < 2) // we see only 1 corner in total => no localization this time
	{
		env->SetFloatArrayRegion(cameraPosition, 0, 4, unknown_camera_pos.val);
		return;
    }
	
    uint8_t votes_for_id[5][4][20];
    memset(votes_for_id, 0, sizeof(votes_for_id));

	for (uint8_t c1 = 0; c1 < 4; c1++)
		for (uint8_t c2 = c1 + 1; c2 < 4; c2++)
			if (corner_counts[c1] * corner_counts[c2])
		    {
				uint8_t neighboring_colors = (c1 + c2) & 1;
			
				for (uint8_t i = 0; i < corner_counts[c1]; i++)
					for (uint8_t j = 0; j < corner_counts[c2]; j++)
					{
						uint8_t id1, id2;
					
					    determine_ids(c1, c2, corner_points[c1][i], corner_points[c2][j], /*out*/ id1, /*out*/ id2);
						if ((id1 == 255) || (id2 == 255))
						{
							cpp_debug("corners", "unrecognized pair of corner points!");
						    continue;
						}
					
						sprintf(str, "determine ids: c1=%hhu, c2=%hhu, i=%hhu, j=%hhu, id1=%hhu, id2=%hhu", c1, c2, i, j, id1, id2);
					    cpp_debug("corners", str);
						
						votes_for_id[c1][i][id1]++;
						votes_for_id[c2][j][id2]++;
					}
				//TODO: pairs of same color vertexes do here
			}
		
    if (corner_counts[4])  // found some yellow corners?		
	{
		min_distance = IMAGE_MAXIMUM_VALID_X + IMAGE_MAXIMUM_VALID_Y;
		for (uint8_t c2 = 0; c2 < 4; c2++)
			if (corner_counts[c2])
			{
				for (uint8_t i = 0; i < corner_counts[4]; i++)
				{
					cv::Point &p1 = corner_points[4][i].first;					
					for (uint8_t j = 0; j < corner_counts[c2]; j++)
					{
						cv::Point &p2 = corner_points[c2][j].first;
						float len = std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
						if ((len < min_distance) && (len >= MIN_CORNER_DISTANCE)) min_distance = len;
					}
				}
			}
		for (uint8_t c2 = 0; c2 < 4; c2++)
			if (corner_counts[c2])
			{
				for (uint8_t i = 0; i < corner_counts[4]; i++)
					for (uint8_t j = 0; j < corner_counts[c2]; j++)
					{
						uint8_t id1, id2;
						
						Y_determine_ids(c2, corner_points[4][i], corner_points[c2][j], /* out */ id1, /* out */ id2);
						if ((id1 == 255) || (id2 == 255))
						{
							cpp_debug("corners", "unrecognized pair of corner points!");
							continue;
						}
						sprintf(str, "YELLOW determine ids: c2=%hhu, i=%hhu, j=%hhu, id1=%hhu, id2=%hhu", c2, i, j, id1, id2);
						cpp_debug("corners", str);
						
						votes_for_id[4][i][id1]++;
						votes_for_id[c2][j][id2]++;
					}
			}
	}	
		
	uint8_t determined_ids[5][4];
	memset(determined_ids, 255, sizeof(determined_ids));
	
	// points in 3D world (corners) with directional vectors towards camera
	// ( (color,index_in_color), (2Dpoint, 3Dvector_in_world_space))
	std::vector<std::pair<std::pair<int,int>,std::pair<cv::Vec2f, cv::Vec3f>>> camera_incoming_world_vectors_normalized;   
	
	cpp_debug("corners", "votes for IDs");
	// for each corner on the ground find the most popular from all votes for its ID
	for (int c = 0; c < 5; c++)
		for (int i = 0; i < corner_counts[c]; i++)
		{
			uint8_t max_id = 255;
			uint8_t max = 0;
		    for (int id = 0; id < 20; id++)
		    {
				uint8_t num_votes = votes_for_id[c][i][id]; 
				sprintf(str, "votes_for_id[c=%d][i=%d][id=%d]=%hhu", c, i, id, votes_for_id[c][i][id]);
				cpp_debug("corners", str);
          	    if (num_votes > max)
				{
					max = num_votes;
					max_id = id;
				}
			}
			determined_ids[c][i] = max_id;
			
			sprintf(str, "determined_ids[%d][%d] = %d", c, i, max_id);
			cpp_debug("corners", str);
		}
		
	// now we need to find the yaw: for each two corners of different colors, look at the angles at the floor and in the camera, finally possibly remove outliers and make average
	
	float collected_yaws[320];        // max 20 x 16 two-color pairs
	int num_yaws = 0;
	
	for (int c1 = 0; c1 < 4; c1++)
		for (int i = 0; i < corner_counts[c1]; i++)
		{
			int P1_camera_x = corner_points[c1][i].first.x;
			int P1_camera_y = -corner_points[c1][i].first.y;  // here is the place we invert y-axis of camera frame, because it grows opposite to the world coordinates y-axis
			
			int id1 = determined_ids[c1][i];
			if (id1 == 255) continue;  
			
			float P1_ground_x = world_coordinates[id1][0];
			float P1_ground_y = world_coordinates[id1][1];
			
	        for (int c2 = c1 + 1; c2 < 5; c2++)
		        for (int j = 0; j < corner_counts[c2]; j++)
				{
			        int id2 = determined_ids[c2][j];
                    if (id2 == 255) continue;
					
					int P2_camera_x = corner_points[c2][j].first.x;
					int P2_camera_y = -corner_points[c2][j].first.y;  // here is the place we invert y-axis of camera frame, it grows opposite to the world coordinates y-axis
			
					float P2_ground_x = world_coordinates[id2][0];
					float P2_ground_y = world_coordinates[id2][1];
				
				    float alpha_camera = atan2(P2_camera_y - P1_camera_y, P2_camera_x - P1_camera_x);
				    float alpha_ground = atan2(P2_ground_y - P1_ground_y, P2_ground_x - P1_ground_x);
					
					// we do not worry about outside of -PI,PI interval since sin,cos will bring up the correct unit-vector anyway
					collected_yaws[num_yaws++] = alpha_ground - alpha_camera;
					sprintf(str, "collecting yaw (c1=%d,i=%d,c2=%d,j=%d) P1cam=[%d,%d], P2cam=[%d,%d], P1gnd=[%.2f,%.2f], P2gnd[%.2f,%.2f], alphaCam=%.2f, alphaGnd=%.2f => yaw=%.2f(%.2f deg)", 
					         c1, i, c2, j, 
							 P1_camera_x, P1_camera_y, P2_camera_x, P2_camera_y, 
							 P1_ground_x, P1_ground_y, P2_ground_x, P2_ground_y, 
							 alpha_camera, alpha_ground, 
							 (float)(alpha_ground - alpha_camera), (float)((alpha_ground - alpha_camera) / (float)M_PI * 180.0f));
					cpp_debug("corners", str);
				}
		}
	
	cpp_debug("corners", "num_yaws", num_yaws);
	
	float camera_yaw;
	if (num_yaws == 0)  // no two-color pair is seen
	{
		env->SetFloatArrayRegion(cameraPosition, 0, 4, unknown_camera_pos.val);
		return;
	}
	if (num_yaws == 1)
	{
		camera_yaw = collected_yaws[0];
	}
	else // at least two candidate yaws, calculate average
	{
		float sum_x = 0.0f, sum_y = 0.0f;
		for (int i = 0; i < num_yaws; ++i) {
			sum_x += cos(collected_yaws[i]);
			sum_y += sin(collected_yaws[i]);
		}
		float avg_x = sum_x / num_yaws;
		float avg_y = sum_y / num_yaws;
		camera_yaw = atan2(avg_y, avg_x);

        // OUTLIER REMOVAL
		// do:
		//   find the one that is most away from the average of the rest
		//   if it is more than X deg away from the average of the rest:
		//      remove it, 
		//      update average, 
		// while removal took place
		
        // if at least 3 yaws found, keep only those that are not more than 10 degrees deviant from average, and then average again
		
		int removed = 1;
		
		while ((num_yaws > 2) && removed)
		{
			removed = 0;
	        float max_error = 0.0;
		    int max_index = 0;
		
			for (int i = 0; i < num_yaws; i++)
			{			
		        float this_x = cos(collected_yaws[i]);
			    float this_y = sin(collected_yaws[i]);
				float avg_without_this = atan2((sum_y - this_y) / (num_yaws - 1), (sum_x - this_x) / (num_yaws - 1));
				
				float diff = collected_yaws[i] - avg_without_this;

				// Normalize difference to [-PI, PI]
				while (diff > M_PI) diff -= 2 * M_PI;
				while (diff < -M_PI) diff += 2 * M_PI;
				
				sprintf(str, "considering yaw[%d] as outlier: diff=%.3f, avg_without_this=%.3lf, max_error=%.3f", i, diff, avg_without_this, max_error);
				if (fabs(diff) > max_error)
				{
					max_error = fabs(diff);
					max_index = i;
				}
			}
			
			if (max_error > (10.0f * M_PI / 180.0f))   // 10 degrees or more error? => remove it
			{
				removed = 1;
			    
				sprintf(str, "to remove yaw outlier #%d (%.3f), err=%.3f deg", max_index, collected_yaws[max_index], max_error / M_PI * 180.0);
			    cpp_debug("corners", str);
				
				float that_x = cos(collected_yaws[max_index]);
			    float that_y = sin(collected_yaws[max_index]);
				
				sum_x -= that_x;
				sum_y -= that_y;
				num_yaws--;
				
				camera_yaw = atan2(sum_y / num_yaws, sum_x / num_yaws);
				
				collected_yaws[max_index] = collected_yaws[num_yaws];
			}		
		}
	}
	
	cpp_debug_f("corners", "prefinal yaw(rad,deg)=", camera_yaw, camera_yaw / M_PI * 180.0f);
	camera_yaw = filter_yaw(camera_yaw);
	cpp_debug_f("corners", "filtered yaw(rad,deg)=", camera_yaw, camera_yaw / M_PI * 180.0f);

	// construct and collect all the real-world 3D vectors from detected corners together with their origin in the corner into one data structure
	// while rotating them based on camera yaw
		
	//float cos_alpha = cos(camera_yaw);
	//float sin_alpha = sin(camera_yaw);
	
	for (int c = 0; c < 5; c++)
		for (int i = 0; i < corner_counts[c]; i++)
		{			
			if (determined_ids[c][i] != 255)
			{
				//cv::Vec3f corner_vector = get_direction_vector_from_pixel(&corner_points[c][i].first, cos_alpha, sin_alpha);			
			    cv::Vec3f corner_vector(0.0f, 0.0f, 1.0f);  // corner vectors are not needed in this version of algorithm			    
				const cv::Vec2f &point = world_coordinates[determined_ids[c][i]];
				sprintf(str, "point world[%d,%d]=[%.3f, %.3f]", c, i, point[0], point[1]);
				cpp_debug("corners", str);
  			    camera_incoming_world_vectors_normalized.push_back(std::make_pair(std::make_pair(c,i),std::make_pair(point, corner_vector)));
			}
		}
		
	int num_corners = camera_incoming_world_vectors_normalized.size();
	
	if (num_corners < 2)  // we need at least two corners
	{
		env->SetFloatArrayRegion(cameraPosition, 0, 4, unknown_camera_pos.val);
		return;
	}
	
	std::vector<float> heights;
	double height_sum = 0.0;
	
	//--------------- first, determine the height of camera (for all pairs of known points)
	for (int i = 0; i < num_corners; i++)
		for (int j = i + 1; j < num_corners; j++)
		{
			cv::Vec2f &A = camera_incoming_world_vectors_normalized[i].second.first;
			cv::Vec2f &B = camera_incoming_world_vectors_normalized[j].second.first;
			
			cv::Point &U = corner_points[camera_incoming_world_vectors_normalized[i].first.first][camera_incoming_world_vectors_normalized[i].first.second].first;
			cv::Point &V = corner_points[camera_incoming_world_vectors_normalized[j].first.first][camera_incoming_world_vectors_normalized[j].first.second].first;
            // U,V are in pixel coordinates orientation, but the orientation does not matter, because only their distance does			
			
			float world_distance = cv::norm(A - B);
			float camera_distance = cv::norm(U - V);
			float camera_sensor_distance = camera_distance * camera_pixel_size;
			
			if (camera_distance >= MIN_CORNER_DISTANCE) 
			{	
				float height = camera_focal_length * world_distance / camera_sensor_distance;
				height_sum += height;
				heights.push_back(height); 
				sprintf(str, "height candidate(%d,%d)=%f (A=[%.2f,%.2f], B=[%.2f,%.2f], U=[%d,%d], V=[%d,%d] wd=%.2f, cd=%.5f", i, j, height, A[0], A[1], B[0], B[1], U.x, U.y, V.x, V.y, world_distance, camera_sensor_distance);
				cpp_debug("corners", str);
			}
			else 
			{
				sprintf(str, "ignored height candidate(%d,%d) (A=[%.2f,%.2f], B=[%.2f,%.2f], U=[%d,%d], V=[%d,%d] wd=%.2f, cd=%.5f", i, j, A[0], A[1], B[0], B[1], U.x, U.y, V.x, V.y, world_distance, camera_distance);
				cpp_debug("corners", str);
			} 
		}
		
		
	// removing outliers this way:
	// do:
	//   find the one that is most away from the average of the rest
	//   if it is more than X% away from the average of the rest:
    //      remove it, 
	//      update average, 
	//      store the removed index for later use
	// while removal took place
	
	int cnt = heights.size();
	uint16_t to_remove[cnt];
	memset(to_remove, 0, sizeof(uint16_t) * cnt);

	static const float HEIGHT_ERROR_OUTLIERS_TOLERANCE = 0.15f;  // 15% error in height estimation is allowed, higher error is an outlier to be removed
	int removed = 1;
	int cnt2 = cnt;
	
	cpp_debug_f("corners", "before height outlier removal height_sum=", (float)height_sum);
		
	while ((cnt2 >= 3) && removed)
	{
		removed = 0;
	    float max_error_rate = 0;
		int max_index = 0;
		
		for (int i = 0; i < cnt; i++)
        {
			if (to_remove[i]) continue;
			
			float average_without_this_one = (height_sum - heights[i]) / (cnt2 - 1);
		    float error = fabs(heights[i] - average_without_this_one);
			if (average_without_this_one > 1e-6)
			{
				float error_rate = error / average_without_this_one;
				if (error_rate > max_error_rate)
				{
					max_error_rate = error_rate;
					max_index = i;
				}
			}
		}
		if (max_error_rate > HEIGHT_ERROR_OUTLIERS_TOLERANCE)
		{
			removed = 1;
			height_sum -= heights[max_index];
			to_remove[max_index] = 1;
			cnt2--;
			sprintf(str, "to remove height outlier %.3f, err_rate=%.3f, new height_sum=%.3f", heights[max_index], max_error_rate, height_sum);
			cpp_debug("corners", str);
		}
	}
	
	if (cnt2 < 1)  // no heights survived
	{
		env->SetFloatArrayRegion(cameraPosition, 0, 4, unknown_camera_pos.val);
		return;
	}
	
	float average_height = height_sum / cnt2; 
	cpp_debug_f("corners", "prefinal height estimate=", average_height);
	average_height = filter_height(average_height);
	cpp_debug_f("corners", "final height estimate=", average_height);
	
	//------------end of height estimation
	
	cv::Vec2d camera_position(0.0, 0.0);
	std::vector<cv::Vec2d> cam_pos_estimate;
	
	//--------------now determine the camera x,y position above the ground (for each corner separately, relative to camera center)
	for (int i = 0; i < num_corners; i++)
	{
		cv::Point &U = corner_points[camera_incoming_world_vectors_normalized[i].first.first][camera_incoming_world_vectors_normalized[i].first.second].first;
		cv::Point C(camera_center_x, camera_center_y);
		// both C and U are in camera pixel coordinate orientation, therefore the y-part of the resulting vector C-U will have the opposite sign
		cv::Point2f w = (C-U);
		w *= camera_pixel_size;
		w.y *= -1.0;   // therefore here is the place we change the orientation from pixel to world orientation
		
		cv::Point2f w_rotated;
		w_rotated.x =  w.x * cos(camera_yaw) - w.y * sin(camera_yaw);
		w_rotated.y =  w.x * sin(camera_yaw) + w.y * cos(camera_yaw);
		
		float scaling_factor = (average_height / camera_focal_length);
		cv::Vec2f ground_vector(w_rotated.x * scaling_factor, w_rotated.y * scaling_factor);
		cv::Vec2f &A = camera_incoming_world_vectors_normalized[i].second.first;
		
		cv::Vec2d new_camera_position_estimate(A + ground_vector);
		camera_position += new_camera_position_estimate;
		cam_pos_estimate.push_back(new_camera_position_estimate);
		
		sprintf(str, "position estimate(%d)=[%f,%f]; U=[%d,%d], C=[%d,%d], w=(%.7f,%.7f), w_rot=(%.7f,%.7f), scale=%.3f, gndvec=(%.3f,%.3f), A=[%.3f,%.3f]", 
					   i, new_camera_position_estimate[0], new_camera_position_estimate[1],
					   U.x, U.y, C.x, C.y, w.x, w.y, w_rotated.x, w_rotated.y, scaling_factor, ground_vector[0], ground_vector[1], A[0], A[1]);
		cpp_debug("corners", str);
	}

	sprintf(str, "before outlier removal camera position estimate=[%lf,%lf]", camera_position[0] / num_corners, camera_position[1] / num_corners);
    cpp_debug("corners", str);
	
	// finally remove the outliers from the average position
	cnt = num_corners;
	removed = 1;
	
	static const double POSITION_ERROR_OUTLIERS_TOLERANCE = 0.3;   // if the point is 30 cm off, remove it 
	
    while ((cnt >= 3) && removed)
	{
		removed = 0;
	    float max_error = 0;
		int max_index = 0;
		
		for (int i = 0; i < cnt; i++)
        {
			cv::Vec2d average_without_this_one = (camera_position - cam_pos_estimate[i]) / (cnt - 1);
			double error = cv::norm(cam_pos_estimate[i] - average_without_this_one);

			if (error > max_error)
			{
				max_error = error;
				max_index = i;
			}
		}
		if (max_error > POSITION_ERROR_OUTLIERS_TOLERANCE)
		{
			removed = 1;
			sprintf(str, "removed pos outlier [%.3f,%.3f], err=%.3f", cam_pos_estimate[max_index][0], cam_pos_estimate[max_index][1], max_error);
			cpp_debug("corners", str);
			camera_position -= cam_pos_estimate[max_index];
			cam_pos_estimate.erase(cam_pos_estimate.begin() + max_index);
			cnt--;
		}
	}
	
	camera_position /= cnt; 
	sprintf(str, "prefinal camera position estimate=[%lf,%lf]", camera_position[0], camera_position[1]);
	cpp_debug("corners", str);
	camera_position = filter_position(camera_position);
	sprintf(str, "filtered camera position estimate=[%lf,%lf]", camera_position[0], camera_position[1]);
	cpp_debug("corners", str);
	
	cv::Vec4f cameraPos = cv::Vec4f(camera_position[0], camera_position[1], average_height, camera_yaw);
	env->SetFloatArrayRegion(cameraPosition, 0, 4, cameraPos.val);
	
	log_position(cameraPos);
}

